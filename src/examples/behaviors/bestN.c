/* Kilobot control software for the simple ALF experment : clustering
 * author: Fabio Oddi (Università la Sapienza di Roma) fabio.oddi@diag.uniroma1.it
 */

#include "kilolib.h"
#include <stdlib.h>
#include<stdio.h>
#include "tree_structure.h"
#include "message_structure.h"
#include "quorum_structure.h"

#define PI 3.14159265358979323846
/* used only for noisy data generation */
const float DBL_MIN = 0.15;
typedef enum {
    MAX_UTILITY = 10,
    NOISE = 1
} signal;

/* Enum for messages type */
typedef enum {
  ARK_BROADCAST_MSG = 0,
  ARK_INDIVIDUAL_MSG = 1,
  KILO_BROADCAST_MSG = 255,
  KILO_IDENTIFICATION = 120
} received_message_type;

typedef enum {
  MSG_A = 0,
  MSG_B = 1,
  MSG_C = 2,
  MSG_D = 3,
  MSG_E = 4
} message_type;
/* Enum for motion */

typedef enum {
    FORWARD = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    STOP = 3,
} motion_t;

/* Enum for boolean flags */
typedef enum {
    false = 0,
    true = 1,
} bool;

/* struct for the robot states */
typedef struct state
{
    int current_node,previous_node,commitment_node;
}state_t;

/* struct for the robot position */
typedef struct position
{
    float position_x,position_y;
}position_t;

/* offsets of map axes*/
float offset_x, offset_y;

/* current motion type */
motion_t current_motion_type = STOP;

/* goal position */
position_t goal_position={0,0};

/* position and angle given from ARK */
position_t gps_position={0,0};
int gps_angle=-1;
float RotSpeed=38.0;
float min_dist;
uint32_t lastWaypointTime;
uint32_t maxWaypointTime=3600;//3600; // about 2 minutes

/* current state */
state_t my_state={0,0,0};

unsigned int turning_ticks = 0;
unsigned int straight_ticks = 0;
const uint8_t max_turning_ticks = 160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint16_t max_straight_ticks = 320; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
uint32_t last_motion_ticks = 0;

/* Variables for Smart Arena messages */
int sa_type = 0;
int sa_payload = 0;

bool init_received_A=false;
bool init_received_B=false;
bool init_received_GPS_A=false;
bool init_received_GPS_B=false;

/* map of the environment */
tree_a *the_tree=NULL;

/* counters for broadcast a message */
const uint16_t max_broadcast_ticks = 3;
const uint16_t words_4_a_message = 3;
const uint16_t ticks_for_a_chain=max_broadcast_ticks*(words_4_a_message*2);
uint32_t last_broadcast_ticks = 0;
const uint16_t broadcasting_ticks = ticks_for_a_chain*50;
unsigned int last_sensing_ticks=-broadcasting_ticks;
unsigned int chain_timeout=0;

/* Flag for decision to send a word */
bool sending_msg = false;
int sending_type = MSG_A;
message_t my_message;

/* lists for decision handling */
int msg_chain_check_in=-1;
unsigned int received_id;
unsigned int broadcast_counter=0;

unsigned int received_node;
unsigned int received_leaf;
float received_utility;
float received_control_parameter;
unsigned int received_counter;
bool sem_talking=false;
bool sem_talking_re=false;

message_a *messages_list=NULL;
quorum_a *quorum_list=NULL;
message_a *chosen_message=NULL;

bool start=false;
bool start_chain=false;

float control_parameter=0;
float gain_h=0;
float gain_k=1;

/* used only for noisy data generation */
unsigned int leafs_size=0;
unsigned int leafs_id[16];
unsigned int last_sample_id=-1;
float last_sample_utility=-1;

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
    bool calibrated = true;
    if ( current_motion_type != new_motion_type ){
        switch( new_motion_type ) {
        case FORWARD:
            spinup_motors();
            if (calibrated)
                set_motors(kilo_straight_left,kilo_straight_right);
            else
                set_motors(70,70);
            break;
        case TURN_LEFT:
            spinup_motors();
            if (calibrated)
                set_motors(kilo_turn_left,0);
            else
                set_motors(70,0);
            break;
        case TURN_RIGHT:
            spinup_motors();
            if (calibrated)
                set_motors(0,kilo_turn_right);
            else
                set_motors(0,70);
            break;
        case STOP:
        default:
            set_motors(0,0);
        }
        current_motion_type = new_motion_type;
    }
}

/*-------------------------------------------------------------------*/
/* Function to sample a random number from a Gaussian distribution   */
/*-------------------------------------------------------------------*/
float generate_gaussian_noise(float mu, float std_dev )
{
    const float epsilon = DBL_MIN;
    const float two_pi = 2.0*PI;
    float u1;
    do
    {
        u1 = rand() * (1.0 / RAND_MAX);
    }
    while ( u1 <= epsilon );

    float z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u1);
    // printf("%f___%f\n",mu,mu + z0*std_dev);
    return z0 * std_dev + mu;
}

bool bot_isin_node(tree_a **Node)
{
    if(((*Node)!=NULL)&&(gps_position.position_x >= (*Node)->tlX && gps_position.position_x <= (*Node)->brX)&&(gps_position.position_y >= (*Node)->tlY && gps_position.position_y <= (*Node)->brY)) return true;
    return false;
}

/*-------------------------------------------------------------------*/
/* Send current kb status to the swarm                               */
/*-------------------------------------------------------------------*/
message_t *message_tx()
{    
    if (sending_msg) return &my_message;
    return 0;
}

/*-------------------------------------------------------------------*/
/* Callback function for successful transmission                     */
/*-------------------------------------------------------------------*/
void message_tx_success()
{
    sending_msg = false;
}

/*-------------------------------------------------------------------*/
/* Function to broadcast a message                                        */
/*-------------------------------------------------------------------*/
void broadcast()
{
    if (!sending_msg && kilo_ticks > last_broadcast_ticks + max_broadcast_ticks)
    {
        int utility_to_send;
        sa_type=0;
        for (int i = 0; i < 9; ++i) my_message.data[i]=0;
        switch (sending_type)\
        {
            case MSG_A:
                utility_to_send = (int)(last_sample_utility*100);
                my_message.data[0] = kilo_uid | sa_type<<7;
                my_message.data[1] = (sa_type>>1)<<7 | utility_to_send>>3;
                my_message.data[2] = (utility_to_send & 0b00000111)<<5 | (last_sample_id-1)<<3;
                sending_type=MSG_B;
                break;
            case MSG_B:
                sa_type = sending_type;
                my_message.data[0] = kilo_uid | sa_type<<7;
                my_message.data[1] = (sa_type>>1)<<7;
                my_message.data[2] = broadcast_counter;
                sending_type = MSG_C;
                break;
            case MSG_C:
                sa_type = sending_type;
                my_message.data[0] = kilo_uid | sa_type<<7;
                my_message.data[1] = (sa_type>>1)<<7 |(int)(control_parameter*100);
                my_message.data[2] = my_state.current_node;
                sending_type = MSG_A;
                break;
        }
        my_message.crc = message_crc(&my_message);
        last_broadcast_ticks = kilo_ticks;
        set_color(RGB(3,3,0));
        sending_msg = true;
    }
}

/*-----------------------------------------------------------------------------------*/
/* sample a value, update the map, decide if change residence node                   */
/*-----------------------------------------------------------------------------------*/
void sample_and_decide(tree_a **leaf)
{
    float random_sample = generate_gaussian_noise((*leaf)->gt_utility,NOISE);
    bottom_up_utility_update(&the_tree,(*leaf)->id,random_sample);
    last_sensing_ticks = kilo_ticks;
    last_sample_utility = get_utility((*leaf)->node_filter);
    if(last_sample_utility > 10) last_sample_utility=10;
    else if(last_sample_utility < 0) last_sample_utility=0;
    // decide to commit or abandon

    // printf("K:%d_____%f\n",kilo_uid,the_tree->node_filter->utility);
}

/*-----------------------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk with the random waypoint model */
/*-----------------------------------------------------------------------------------*/
void select_new_point(bool force)
{
    /* if the robot arrived to the destination, a new goal is selected and a noisy sample is taken from the correspective leaf*/
    if (force || ((abs((int)((gps_position.position_x-goal_position.position_x)*100))*.01<.001) && (abs((int)((gps_position.position_y-goal_position.position_y)*100))*.01<.001)))
    {
        tree_a *leaf=NULL;
        if(force) set_color(RGB(0,0,0));
        else
        {
            for(unsigned int l=0;l<leafs_size;l++)
            {
                leaf = get_node(&the_tree,leafs_id[l]);
                if(bot_isin_node(&leaf)) break;
            }
            if(leaf==NULL)
            {
                last_sample_utility = -1;
                last_sample_id = -1;
                gps_angle=-1;
                printf("ERROR____Agent:%d___NOT_ON_LEAF__\n",kilo_uid);
            }
            else
            {
                last_sample_id = get_id(&leaf);
                sample_and_decide(&leaf);
            }
        }
        lastWaypointTime = kilo_ticks;
        tree_a *actual_node = get_node(&the_tree,my_state.current_node);
        float flag;
        flag=abs((int)((actual_node->brX-actual_node->tlX)*100))*.0025;
        min_dist=abs((int)((actual_node->brY-actual_node->tlY)*100))*.0025;
        if(flag>min_dist) min_dist=flag;
        do {
            goal_position.position_x=(float)(random_in_range((int)((actual_node->tlX)*100),(int)((actual_node->brX)*100)))*.01;
            goal_position.position_y=(float)(random_in_range((int)((actual_node->tlY)*100),(int)((actual_node->brY)*100)))*.01;
            float dif_x,dif_y;
            dif_x = abs((int)((gps_position.position_x-goal_position.position_x)*100))*.01;
            dif_y = abs((int)((gps_position.position_y-goal_position.position_y)*100))*.01;
            if (dif_x >= min_dist || dif_y >= min_dist ) break;
        } while(true);

    }
}

/*-------------------------------------------------------------------*/
/* Parse smart messages                            */
/*-------------------------------------------------------------------*/
void parse_smart_arena_message(uint8_t data[9], uint8_t kb_index)
{
    // index of first element in the 3 sub-blocks of data
    uint8_t shift = kb_index * 3;
    sa_type = ((data[shift] >> 5) & 0b00000100) | (data[shift + 1] >> 6);
    sa_payload = ((data[shift + 1] & 0b00111111) << 8) | (data[shift + 2]);
    switch(sa_type)
    {
        case MSG_C:
            gps_position.position_x = (sa_payload >> 7) * .01;
            gps_position.position_y = ((uint8_t)sa_payload & 0b01111111) * .01;
            gps_angle=-1;
            // printf("%d_____%f_%f___%f_%f\n",kilo_uid,gps_position.position_x,gps_position.position_y,goal_position.position_x,goal_position.position_y);
            break;
        case MSG_D:
            gps_angle = (((uint8_t)sa_payload) * .1) * 30;
            break;
    }
}

/*-------------------------------------------------------------------*/
/* Bufferize incoming data                                           */
/*-------------------------------------------------------------------*/
void update_messages()
{
    // update buffer if the messages chain is valid
    if(msg_chain_check_in>=0)
    {
        int freshness;
        freshness = is_fresh_message(&messages_list,received_id,received_node,received_leaf,received_counter,received_utility);
        if(freshness>0)
        {
            if(freshness == 1) add_a_message(&messages_list,NULL,received_id,received_node,received_leaf,received_counter,received_utility);
            printf("%d__%d+\n\n",kilo_uid,received_id);
        }
        msg_chain_check_in=-1;
    }
}

/*-------------------------------------------------------------------*/
/* Parse smart messages                                              */
/*-------------------------------------------------------------------*/
void parse_kilo_message(uint8_t data[9])
{
    // Agents wait for 3 messages by the same teammate
    // if the chain is broken the agent forgets partial data
    sa_type = (data[1] & 0b10000000)>>6 | data[0] >> 7;
    sa_payload = (data[1] & 0b01111111) << 8 | data[2];
    int flag_id;
    switch(sa_type)
    {
        case MSG_A:
            flag_id = data[0] & 0b01111111;
            if(msg_chain_check_in==-1 && flag_id!=received_id && flag_id!=kilo_uid)
            {
                start_chain=true;
                chain_timeout=kilo_ticks;
                received_id = flag_id;
                received_utility = (sa_payload >> 5)*.01;
                received_leaf = (uint8_t)sa_payload & 0b00011111;
                msg_chain_check_in = -2;
            }
            break;
        case MSG_B:
            flag_id = data[0] & 0b01111111;
            if(msg_chain_check_in==-2 && flag_id==received_id)
            {
                received_counter = sa_payload;
                msg_chain_check_in = -3;
            }
            else
            {
                msg_chain_check_in = -2;
            }
            break;
        case MSG_C:
            flag_id = data[0] & 0b01111111;
            if(msg_chain_check_in==-3 && flag_id==received_id)
            {
                received_counter=received_counter+1;
                received_control_parameter = (sa_payload >> 8)*.01;
                msg_chain_check_in = (uint8_t)sa_payload;
                received_node = msg_chain_check_in;
                start_chain=false;
            }
            else
            {
                msg_chain_check_in = -3;
            }
            break;
    }
    update_messages();
}

void parse_smart_arena_broadcast(uint8_t data[9])
{   
    sa_type = ((data[0] >> 5) & 0b00000100) | (data[1] >> 6);
    switch (sa_type){
        case MSG_A:
            if(!init_received_A)
            {   
                set_color(RGB(3,3,0));
                float k = (data[0] & 0b01111111)*.01;
                int best_leaf_id = (data[1] & 0b00011111)+1;
                int branches = (data[2]>>2)+1;
                int depth = (data[2] & 0b00000011)+1;
                complete_tree(&the_tree,depth,branches,&leafs_id,&leafs_size,best_leaf_id,MAX_UTILITY,k);
                init_received_A=true;
            }
            break;
        case MSG_B:
            if(!init_received_B && init_received_A)
            {   
                float brX = (float)(data[0] & 0b01111111) * .1;
                float brY = (float)(data[1] & 0b00011111) * .1;
                offset_x=brX/2;
                offset_y=brY/2;
                goal_position.position_x=offset_x;
                goal_position.position_y=offset_y;
                set_vertices(&the_tree,brX,brY);
                int expiring_dist = brX*100;
                if(brY>brX) expiring_dist=brY*100;
                set_expiring_time_message(expiring_dist*TICKS_PER_SEC*1.5);
                init_received_B=true;
            }
            break;
        case MSG_C:
            if(!init_received_GPS_A && init_received_A && init_received_B)
            {
                int id1 = data[0] & 0b01111111;
                int id2 = data[3] & 0b01111111;
                int id3 = data[6] & 0b01111111;
                if (id1 == kilo_uid)
                {
                    parse_smart_arena_message(data, 0);
                }
                else if (id2 == kilo_uid)
                {
                    parse_smart_arena_message(data, 1);
                }
                else if (id3 == kilo_uid)
                {
                    parse_smart_arena_message(data, 2);
                }
                init_received_GPS_A=true;
            }
            break;
        case MSG_D:
            if(!init_received_GPS_B && init_received_GPS_A && init_received_A && init_received_B)
            {
                int id1 = data[0] & 0b01111111;
                int id2 = data[3] & 0b01111111;
                int id3 = data[6] & 0b01111111;
                if (id1 == kilo_uid)
                {
                    parse_smart_arena_message(data, 0);
                }
                else if (id2 == kilo_uid)
                {
                    parse_smart_arena_message(data, 1);
                }
                else if (id3 == kilo_uid)
                {
                    parse_smart_arena_message(data, 2);
                }
                init_received_GPS_B=true;
                select_new_point(true);
                set_motion(FORWARD);
                start = true;
            }
            break;
        case MSG_E:
            
            break;
    }
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx(message_t *msg, distance_measurement_t *d) {
    sa_type = 0;
    sa_payload = 0;
    // printf("%d__\n\n",msg->type);
    switch (msg->type)
    {
        case ARK_BROADCAST_MSG:{
            parse_smart_arena_broadcast(msg->data);
            break;
        }
        case ARK_INDIVIDUAL_MSG:{
            int id1 = msg->data[0] & 0b01111111;
            int id2 = msg->data[3] & 0b01111111;
            int id3 = msg->data[6] & 0b01111111;

            if (id1 == kilo_uid)
            {
                parse_smart_arena_message(msg->data, 0);
            }
            else if (id2 == kilo_uid)
            {
                parse_smart_arena_message(msg->data, 1);
            }
            else if (id3 == kilo_uid)
            {
                parse_smart_arena_message(msg->data, 2);
            }
            break;
        }
        case KILO_BROADCAST_MSG:{
            parse_kilo_message(msg->data);
            break;
        }
        case KILO_IDENTIFICATION:{
            int id = msg->data[0] & 0b01111111;
            if (id == kilo_uid) {
                set_color(RGB(0,0,3));
            } else {
                set_color(RGB(3,0,0));
            }
            break;
        }
    }
}

/*-------------------------------------------------------------------*/
/* Compute angle to Goal                                             */
/*-------------------------------------------------------------------*/
void NormalizeAngle(int* angle)
{
    while(*angle>180){
        *angle=*angle-360;
    }
    while(*angle<-180){
        *angle=*angle+360;
    }
    *angle=*angle*-1;
}
int AngleToGoal()
{
    int angletogoal=(atan2(goal_position.position_y-gps_position.position_y,goal_position.position_x-gps_position.position_x)/PI)*180 - (gps_angle - 180);
    NormalizeAngle(&angletogoal);
    return angletogoal;
}

int random_in_range(int min, int max
){
   return min + (rand()%(max-min));
}

/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
    switch( current_motion_type ) {
    case TURN_LEFT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            set_motion(FORWARD);
            straight_ticks = rand()%max_straight_ticks + 1;
        }
    case TURN_RIGHT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            set_motion(FORWARD);
            straight_ticks = rand()%max_straight_ticks + 1;
        }
        break;
    case FORWARD:
        if( kilo_ticks > last_motion_ticks + straight_ticks ) {
            /* perform a radnom turn */
            last_motion_ticks = kilo_ticks;
            turning_ticks = rand()%max_turning_ticks + 1;
            if( rand()%2 ) {
                set_motion(TURN_LEFT);
            }
            else {
                set_motion(TURN_RIGHT);
            }
        }
        break;
    case STOP:
    default:
        set_motion(STOP);
    }
}

void random_way_point_model()
{   
    if(gps_angle!=-1)
    {
        int angleToGoal = AngleToGoal();
        if(fabs(angleToGoal) <= 20)
        {
            set_motion(FORWARD);
            last_motion_ticks = kilo_ticks;
        }
        else{
            if(angleToGoal>0){
                set_motion(TURN_LEFT);
                last_motion_ticks = kilo_ticks;
                turning_ticks=(unsigned int) ( fabs(angleToGoal)/RotSpeed*32.0 );
            }
            else{
                set_motion(TURN_RIGHT);
                last_motion_ticks = kilo_ticks;
                turning_ticks=(unsigned int) ( fabs(angleToGoal)/RotSpeed*32.0 );
            }
        }
        select_new_point(false);
        switch( current_motion_type )
        {
            case TURN_LEFT:
                if( kilo_ticks > last_motion_ticks + turning_ticks ) {
                    /* start moving forward */
                    last_motion_ticks = kilo_ticks;  // fixed time FORWARD
                    set_motion(FORWARD);
                }
                break;
            case TURN_RIGHT:
                if( kilo_ticks > last_motion_ticks + turning_ticks ) {
                    /* start moving forward */
                    last_motion_ticks = kilo_ticks;  // fixed time FORWARD
                    set_motion(FORWARD);
                }
                break;
            case FORWARD:
                break;
            case STOP:
            default:
                set_motion(STOP);
        }
    }
    else if(init_received_GPS_B)
    {
        set_motion(FORWARD);
        random_walk();
    }
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
    /* Initialise LED and motors */
    set_color(RGB(0,0,0));
    set_motors(0,0);

    /* Initialise state and message type*/
    my_state.current_node=0;
    my_state.previous_node=0;
    my_state.commitment_node=0;
    my_message.type=KILO_BROADCAST_MSG;

    /* Initialise random seed */
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    /* Initialise motion variables */
    last_motion_ticks=rand()%max_straight_ticks;
    set_motion(STOP);
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
    increment_messages_counter(&messages_list);
    // erase_expired_messages(&messages_list);
    random_way_point_model();
    if(kilo_ticks < last_sensing_ticks + broadcasting_ticks)
    {
        sem_talking = true;
        broadcast();
        broadcast_counter=broadcast_counter+1;
    }
    else
    {
        broadcast_counter=0;
        if(start) set_color(RGB(0,0,0));
        sem_talking = false;
    }
    if(kilo_ticks>chain_timeout + 3*ticks_for_a_chain && msg_chain_check_in<-1)
    {
        start_chain=false;
        msg_chain_check_in = -1;
    }
    if(start_chain) set_color(RGB(3,0,3));
}

int main()
{
    kilo_init();
    
    // register message transmission callback
    kilo_message_tx = message_tx;

    // register tranmsission success callback
    kilo_message_tx_success = message_tx_success;

    // register message reception callback
    kilo_message_rx = message_rx;

    kilo_start(setup, loop);
    
    // erase_tree(&the_tree);
    return 0;
}
