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
/* Enum for different motion types */
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

/* Enum for the robot states */
typedef struct state
{
    int current_node,previous_node,commitment_node;
}state_t;


typedef struct position
{
    uint8_t position_x,position_y;
}position_t;

/* current motion type */
motion_t current_motion_type = STOP;

/* goal position */
position_t goal_position={0,0};

/* position and orientation given from ARK */
position_t gps_position={0,0};
double gps_orientation;

float RotSpeed=38.0;

uint8_t GPS_maxcell=16;
uint8_t minDist=4;

uint32_t lastWaypointTime;
uint32_t maxWaypointTime=3600; // about 2 minutes

/* current state */
state_t my_state={0,0,0};

unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint16_t max_straight_ticks = 320; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
uint32_t last_motion_ticks = 0;

/* Variables for Smart Arena messages */
bool new_sa_msg_BOT = false;
bool new_sa_msg_GPS = false;
int sa_type = 0;
int sa_payload = 0;

/* map of the environment */
tree_a *theTree=NULL;

/* lists for decision handling */
message_a *inputBuffer=NULL;
message_a *messagesList=NULL;
quorum_a *quorumList=NULL;

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

int random_in_range(float min, float max){
   return min + rand() / (RAND_MAX / (max - min + 1) + 1);
}
/*-------------------------------------------------------------------*/
/* Parse ARK received messages                                       */
/*-------------------------------------------------------------------*/
void parse_smart_arena_message(uint8_t data[9], uint8_t kb_index)
{
    // index of first element in the 3 sub-blocks of data
    uint8_t shift = kb_index * 3;

    sa_type = data[shift + 1] >> 2 & 0x0F;
    sa_payload = ((data[shift + 1] & 0b11) << 8) | (data[shift + 2]);
    // printf("sa_type: %d\n", sa_type);
    // printf("sa_payload: %d\n", sa_payload);
    switch (sa_type)
    {
        case 0: //ARK init hierarchical struct parameters
            float brX = (float)((data[0]>>2)*.1);
            float brY = (float)((data[1]>>3)*.1);
            int branches = (int)(data[0] ^ ((int)(brX*10)<<2));
            int depth = (int)(data[1] ^ ((int)(brY*10)<<3));
            complete_tree(&theTree,depth,branches);
            set_vertices(&theTree,brX,brY);
            chose_new_goal_position(true);
            break;
        case 1: // ARK init message 4 noisy data generation

            break;
        case 2: // GPS agent position from ARK
            new_sa_msg_GPS=true;
            break;

        default:
            break;
    }

    if (sa_payload != 0)
    {
        
    }
}
/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx(message_t *msg, distance_measurement_t *d) {
    
    if (msg->type == 0)
    {
        int id1 = msg->data[0] >> 1;
        int id2 = msg->data[3] >> 1;
        int id3 = msg->data[6] >> 1;

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
    }
    else if(msg->type==1) //BOT broadcast message
    {
        new_sa_msg_BOT=true;
    }
    else if(msg->type==2) 
    {
        // unpack message
        int id1 = msg->data[0];
        int id2 = msg->data[3];
        int id3 = msg->data[6];
        if (id1 == kilo_uid) {
            // unpack data
            gps_position.position_x = msg->data[1]>>2 & 0x0F;
            gps_position.position_y = (msg->data[1] & 0x03) << 2 | (msg->data[2]>>6) ;
            gps_orientation = (msg->data[2] & 0x3F)*12;
            new_sa_msg_GPS = true;
        }
        if (id2 == kilo_uid) {
            // unpack data
            gps_position.position_x = msg->data[4]>>2 & 0x0F;
            gps_position.position_y = (msg->data[4] & 0x03) << 2 | (msg->data[5]>>6) ;
            gps_orientation = (msg->data[5] & 0x3F)*12;
            new_sa_msg_GPS = true;
        }
        if (id3 == kilo_uid) {
            // unpack data
            gps_position.position_x = msg->data[7]>>2 & 0x0F;
            gps_position.position_y = (msg->data[7] & 0x03) << 2 | (msg->data[8]>>6) ;
            gps_orientation = (msg->data[8] & 0x3F)*12;
            new_sa_msg_GPS = true;
        }
    }
    else if (msg->type == 120) {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }
    }
    // printf("Agent: %d, node_tl:_%f,%f___br:_%f,%f\n",kilo_uid,(*theTree).children[0].tlX,(*theTree).children[0].tlY,(*theTree).children[0].brX,(*theTree).children[0].brY);
}

/*-------------------------------------------------------------------*/
/* Compute angle to Goal                                             */
/*-------------------------------------------------------------------*/
void NormalizeAngle(double* angle)
{
    while(*angle>180){
        *angle=*angle-360;
    }
    while(*angle<-180){
        *angle=*angle+360;
    }
}

double AngleToGoal() {
    NormalizeAngle(&gps_orientation);
    double angletogoal=atan2(goal_position.position_y-gps_position.position_y,goal_position.position_x-gps_position.position_y) / (PI*180-gps_orientation);
    NormalizeAngle(&angletogoal);
    return angletogoal;
}

/*-----------------------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk with the random waypoint model */
/*-----------------------------------------------------------------------------------*/
void chose_new_goal_position(bool selectNewWaypoint)
{
    /* if the robot arrived to the destination, OR too much time has passed, a new goal is selected */
    if ( selectNewWaypoint || ((goal_position.position_x==gps_position.position_x) && (goal_position.position_y==gps_position.position_y)) || kilo_ticks >= lastWaypointTime + maxWaypointTime) {
        lastWaypointTime = kilo_ticks;
        do {
            goal_position.position_x=(random_in_range(get_node(&theTree,my_state.current_node)->tlX,get_node(&theTree,my_state.current_node)->brX)) % ( GPS_maxcell-2 )+1; // getting a random number in the range [1,GPS_maxcell-1] to avoid the border cells (upper bound is -2 because index is from 0)
            goal_position.position_y=(random_in_range(get_node(&theTree,my_state.current_node)->tlY,get_node(&theTree,my_state.current_node)->brY)) % ( GPS_maxcell-2 )+1;
            if ( abs(gps_position.position_x-goal_position.position_x) >= minDist || abs(gps_position.position_y-goal_position.position_y) >= minDist ){ // if the selected cell is enough distant from the current location, it's good
                break;
            }
        } while(true);
    }
}
/*-------------------------------------------------------------------*/
/* Function to go to the Goal location (e.g. to resample an option)  */
/*-------------------------------------------------------------------*/
void GoToGoalLocation()
{
    if(new_sa_msg_GPS){
        new_sa_msg_GPS=false;
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
    }

    switch( current_motion_type ) {
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
/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk()
{
    switch( current_motion_type ) {
    case TURN_LEFT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            //	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
            set_motion(FORWARD);
        }
    case TURN_RIGHT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            //	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
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
    set_motion( FORWARD );
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
    random_walk();
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
