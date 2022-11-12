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
typedef enum {
    MAX_UTILITY = 10,
    NOISE = 1
} signal;

/* Enum for messages type */
typedef enum {
  ARK_INIT_MSG = 0,
  ARK_INDIVIDUAL_MSG = 1,
  KILO_BROADCAST_MSG = 255,
  KILO_IDENTIFICATION = 120
} received_message_type;

typedef enum {
  MSG_A = 0,
  MSG_B = 1,
  MSG_GPS_A = 2,
  MSG_GPS_B = 3
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
uint32_t maxWaypointTime=1800;//3600; // about 2 minutes

/* current state */
state_t my_state={0,0,0};

unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint16_t max_straight_ticks = 320; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
uint32_t last_motion_ticks = 0;

/* Variables for Smart Arena messages */
int sa_type = 0;
int sa_payload = 0;

bool init_received_A=false;
bool init_received_B=false;
bool received_GPS_A=false;
bool received_GPS_B=false;

/* map of the environment */
tree_a *theTree=NULL;

/* lists for decision handling */
message_a *inputBuffer=NULL;
message_a *messagesList=NULL;
quorum_a *quorumList=NULL;

/* used only for noisy data generation */
unsigned int leafs_size=0;
unsigned int leafs_id[16];

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

void parse_smart_arena_message(uint8_t data[9], uint8_t kb_index)
{
    // index of first element in the 3 sub-blocks of data
    uint8_t shift = kb_index * 3;
    sa_type = data[shift + 1] >> 6;
    sa_payload = ((data[shift + 1] & 0b00111111) << 8) | (data[shift + 2]);
    switch(sa_type)
    {
        case MSG_GPS_A:
            gps_position.position_x=((sa_payload & 0b1111111110000000)>>7)*.01;
            gps_position.position_y=(sa_payload & 0b0000000001111111)*.01;
            break;
        case MSG_GPS_B:
            gps_angle=((sa_payload & 0b0000000001111111)*.1)*30;
            break;
    }
}

void parse_smart_arena_broadcast(uint8_t data[9])
{   
    sa_type = (int)(data[1] >> 6);
    switch (sa_type){
        case MSG_A:
            if(!init_received_A)
            {   
                float k = data[0]*.01;
                int best_leaf_id = (data[1] & 0b00011111)+1;
                int branches = (data[2]>>2)+1;
                int depth = (data[2] & 0b00000011)+1;
                complete_tree(&theTree,depth,branches,&leafs_id,&leafs_size,best_leaf_id,MAX_UTILITY,k);
                init_received_A=true;
                set_color(RGB(0,3,0));
            }
            break;
        case MSG_B:
            if(!init_received_B)
            {   
                float brX = (float)data[0] * .1;
                float brY = (float)(data[1] & 0b00011111) * .1;
                offset_x=brX/2;
                offset_y=brY/2;
                goal_position.position_x=offset_x;
                goal_position.position_y=offset_y;
                set_vertices(&theTree,brX,brY);
                init_received_B=true;
            }
            break;
        case MSG_GPS_A:
            if(!received_GPS_A)
            {
                int id1 = data[0];
                int id2 = data[3];
                int id3 = data[6];
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
                received_GPS_A=true;
            }
            break;
        case MSG_GPS_B:
            if(!received_GPS_B)
            {
                int id1 = data[0];
                int id2 = data[3];
                int id3 = data[6];
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
                set_motion(FORWARD);
                set_color(RGB(0,0,0));
                select_new_point(true);
                received_GPS_B=true;
            }
            break;
    }
}
/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx(message_t *msg, distance_measurement_t *d) {
    sa_type = 0;
    sa_payload = 0;
    switch (msg->type)
    {
        case ARK_INIT_MSG:{
            parse_smart_arena_broadcast(msg->data);
            break;
        }
        case ARK_INDIVIDUAL_MSG:{
            int id1 = msg->data[0];
            int id2 = msg->data[3];
            int id3 = msg->data[6];

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
            break;
        }
        case KILO_IDENTIFICATION:{
            int id = (msg->data[0] << 8) | msg->data[1];
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
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk()
{
    switch( current_motion_type ) {
    case TURN_LEFT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            // last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
            set_motion(FORWARD);
        }
    case TURN_RIGHT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            // last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
            set_motion(FORWARD);
        }
        break;
    case FORWARD:
        if( kilo_ticks > last_motion_ticks + max_straight_ticks ) {
            /* perform a radnom turn */
            last_motion_ticks = kilo_ticks;
            if( rand()%2 ) {
                set_motion(TURN_LEFT);
            }
            else {
                set_motion(TURN_RIGHT);
            }
            turning_ticks = rand()%max_turning_ticks + 1;
        }
        break;
    case STOP:
    default:
        set_motion(STOP);
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
int AngleToGoal() {

    int angletogoal=(atan2(goal_position.position_y-gps_position.position_y,goal_position.position_x-gps_position.position_x)/PI)*180 - (gps_angle - 180);
    NormalizeAngle(&angletogoal);
    return angletogoal;
}

int random_in_range(int min, int max){
   return min + (rand()%(max-min));
}
/*-----------------------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk with the random waypoint model */
/*-----------------------------------------------------------------------------------*/
void select_new_point(bool force)
{
    
    /* if the robot arrived to the destination, a new goal is selected and a noisy sample is taken from the correspective leaf*/
    if (force || (gps_position.position_x==goal_position.position_x) && (gps_position.position_y==goal_position.position_y) || kilo_ticks >= lastWaypointTime + maxWaypointTime)
    {
        if(kilo_ticks >= lastWaypointTime + maxWaypointTime) set_color(RGB(1,2,0));
        else if(force) set_color(RGB(0,0,0));
        else
        {
            set_color(RGB(3,0,3));
            tree_a *leaf=NULL;
            // for(unsigned int l=0;l<leafs_size;l++)
            // {
            //     leaf = get_node(&theTree,leafs_id[l]);
            //     if((gps_position.position_x >= leaf->tlX && gps_position.position_x <= leaf->brX)&&(gps_position.position_y >= leaf->tlY && gps_position.position_y <= leaf->brY)) break;
            //     else leaf=NULL;
            // }
        }
        lastWaypointTime = kilo_ticks;
        tree_a *actual_node = get_node(&theTree,my_state.current_node);
        float flag=abs(actual_node->brX-actual_node->tlX);
        min_dist=abs(actual_node->brY-actual_node->tlY);
        if(flag>min_dist) min_dist=flag;
        do {
            goal_position.position_x=(float)(random_in_range((int)((actual_node->tlX)*100),(int)((actual_node->brX)*100)))*.01;
            goal_position.position_y=(float)(random_in_range((int)((actual_node->tlY)*100),(int)((actual_node->brY)*100)))*.01;
            if ( abs(gps_position.position_x-goal_position.position_x) >= min_dist || abs(gps_position.position_y-goal_position.position_y) >= min_dist ) break;
        } while(true);
    }
    else set_color(RGB(0,0,0));
}

void random_way_point_model()
{   
    if(gps_angle!=-1)
    {
        double angleToGoal = AngleToGoal();
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
    else set_motion(STOP);
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
    /* Initialise LED and motors */
    set_color(RGB(0,0,0));
    set_motors(0,0);
    my_state.current_node=0;
    my_state.previous_node=0;
    my_state.commitment_node=0;
    
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
    random_way_point_model();
}

int main()
{
    kilo_init();
    // register message reception callback
    kilo_message_rx = message_rx;

    kilo_start(setup, loop);
    
    // erase_tree(&theTree);
    return 0;
}
