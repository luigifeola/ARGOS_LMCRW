#include "kilolib.h"
#include <math.h>
#include "distribution_functions.c"
// ****************************************
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#if REAL
#define DEBUG
#include <stdarg.h>
#include "debug.h"
#include <avr/eeprom.h>
#else
#include <inttypes.h>
#endif
// # define NDEBUG 
# include <assert.h> 



#define COLLISION_BITS 8
#define SECTORS_IN_COLLISION 2
#define ARGOS_SIMULATION


/* Enum for different motion types */
typedef enum {
  FORWARD = 0,
  TURN_LEFT = 1,
  TURN_RIGHT = 2,
  STOP = 3,
  RANDOM_ROTATION = 4,
  BOUNCING_ANGLE = 5,
  POINTING_HOME = 6,
} motion_t;

/* Enum for boolean flags */
typedef enum {
  false = 0,
  true = 1,
} bool;

typedef enum {
  ARK_MSG = 0,
  KILOBOTS_MSG = 1,
  CONFIG_PARAMETERS_MSG = 2,
  IDLE = 111
} received_message_type;

typedef enum {
  STATE_MESSAGE=0,
  PROXIMITY_MESSAGE=1,
  RANDOM_ANGLE_MESSAGE=2,
  BIAS_MESSAGE=3,
} SA_TYPE;


/* Enum for the robot states */
typedef enum {
    OUTSIDE_TARGET = 0,
    DISCOVERED_TARGET = 1,
    COMMUNICATED_TARGET = 2,
    WAITING_BIAS = 3,
} state_t;

/* Remember last free side when wall avoidance happened */
typedef enum
{
  LEFT = 1,
  RIGHT = 2,
} Free_space;

/* current motion type */
motion_t current_motion_type = STOP;

/* current state */
state_t current_state = OUTSIDE_TARGET;
state_t current_state_backup = OUTSIDE_TARGET;

/* Message send to the other kilobots */
message_t messageA;

/* Variables for Smart Arena messages */
uint8_t sa_type = 0; 
uint8_t sa_payload;
bool new_sa_msg = false;

/* rotation command */
motion_t rotation_direction = STOP;
double rotation_amount = 0.0;

/* Flag for decision to send a word */
bool sending_msg = false;

/*Flag for the existence of information*/
bool new_information = false;

/* counters for motion, turning and random_walk */
const double std_motion_steps = 5*16;
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 120; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
uint16_t straight_ticks = 0; // keep count of ticks of going straight

/*Parameters from ARK*/
double levy_exponent = -1; 
double crw_exponent = -1; 
bool openSpace_exp = false;
bool received_coefficients = false;



/***********WALL AVOIDANCE***********/
// the kb is "equipped" with a proximity sensor
uint8_t sector_base = 0;
uint8_t left_side = 0;
uint8_t right_side = 0;
uint8_t proximity_sensor = 0;
uint32_t bouncing_ticks = 0;
uint32_t straight_ticks_backup = 0;

uint32_t last_motion_ticks = 0;
/* counters for broadcast a message */
const uint8_t max_broadcast_ticks = 2 * 16; /* n_s*0.5*32 */
uint32_t last_broadcast_ticks = 0;
const uint16_t max_info_ticks = 7 * 16;
uint32_t last_info_ticks = 0;

void my_printf(const char *fmt, ...)
{
#if REAL
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);

  va_end(args);
#endif
}

/*-------------------------------------------------------------------*/
/* Print Kilobot state                                               */
/*-------------------------------------------------------------------*/
void print_state()
{
  printf("%d State: ", kilo_uid);
  switch (current_state)
  {
    case OUTSIDE_TARGET:
      printf("NOT_TARGET_FOUND\n");
      break;
    case DISCOVERED_TARGET:
      printf("TARGET_FOUND\n");
      break;
    case COMMUNICATED_TARGET:
      printf("TARGET_COMMUNICATED\n");
      break;
    case WAITING_BIAS:
      printf("WAITING_BIAS\n");
      break;
      
    default:
      printf("Error, no one of the possible state happens\n");
        break;
  }
}

/*-------------------------------------------------------------------*/
/* Update state only if robot is not waiting a rotation angle        */
/*-------------------------------------------------------------------*/
void update_state(new_state)
{
  if(current_state!=WAITING_BIAS)
    current_state = new_state;
}

/*-------------------------------------------------------------------*/
/* Turn on the right led color                                       */
/*-------------------------------------------------------------------*/
void check_state()
{
  switch (current_state)
  {
    case OUTSIDE_TARGET:
      set_color(RGB(0,0,0));
      break;
    case DISCOVERED_TARGET:
      set_color(RGB(3,0,0));
      break;
    case COMMUNICATED_TARGET:
      set_color(RGB(0,3,0));
      break;
    case WAITING_BIAS:
      set_color(RGB(0,0,3));
      break;
      
    default:
      set_color(RGB(3,3,3));
      break;
  }
}

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type)
{
  if (current_motion_type != new_motion_type)
  {
    switch (new_motion_type)
    {
    case FORWARD:
      spinup_motors();
      set_motors(kilo_straight_left, kilo_straight_right);
      break;
    case TURN_LEFT:
      spinup_motors();
      set_motors(kilo_turn_left, 0);
      break;
    case TURN_RIGHT:
      spinup_motors();
      set_motors(0, kilo_turn_right);
      break;
    case STOP:
    default:
      set_motors(0, 0);
    }
    current_motion_type = new_motion_type;
  }
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{

  /* Initialise LED and motors */
  set_motors(0, 0);
  /* Initialise random seed */
  uint8_t seed = rand_hard();
  rand_seed(seed);
  srand(seed);
  
  /* Initialise motion variables */
  set_motion(STOP);

  /* Initialise KBots message */
  messageA.type = 1;  //0 for ARK, 1 for KBots
  messageA.data[0] = kilo_uid;  
  messageA.crc = message_crc(&messageA);
}



/*-------------------------------------------------------------------*/
/* count 1s after decimal to binary conversion                       */
/*-------------------------------------------------------------------*/
uint8_t countOnes(uint8_t n)
{
  uint8_t count = 0;
  // array to store binary number
  // uint8_t binaryNum[8];

  // counter for binary array
  int i = 0;
  while (n > 0)
  {

    // storing remainder in binary array
    // binaryNum[i] = n % 2;
    if ((n % 2) == 1)
      count++;
    n = n / 2;
    i++;
  }

  return count;
}

/*-------------------------------------------------------------------*/
/* Function implementing wall avoidance procedure                    */
/*-------------------------------------------------------------------*/
uint32_t wall_avoidance_procedure(uint8_t sensor_readings)
{
  uint8_t count_ones = countOnes(sensor_readings);
  if (count_ones > SECTORS_IN_COLLISION){
    right_side = sensor_readings & sector_base;
    left_side = (sensor_readings >> (COLLISION_BITS / 2)) & sector_base;

    if (right_side < left_side){
      rotation_direction = RIGHT;
    }
    else{
      rotation_direction = LEFT;
    }

    current_motion_type = BOUNCING_ANGLE;
    return (uint32_t) ( ((float)count_ones / (float)COLLISION_BITS)  * max_turning_ticks);
  }

  else{
    return 0;
  }
}

/*-------------------------------------------------------------------*/
/* Send current kb status to the swarm                               */
/*-------------------------------------------------------------------*/
message_t *message_tx() {
  if (sending_msg)
  {
    /* this one is filled in the loop */
    return &messageA;
  }
  return 0;
}

/*-------------------------------------------------------------------*/
/* Callback function for successful transmission                     */
/*-------------------------------------------------------------------*/
void message_tx_success() {
  sending_msg = false;
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/* as in the crwlevy_ALF.cpp there are 3 kilobots messages per    */
/* message with 3 different kilobots ids.                            */
/*                                                                   */
/* type 0 for ark / type 1 for kbs interactive msgs                  */
/*                                                                   */
/* A message structure is 12 bytes in length and is composed of      */
/* three parts: the payload (9 bytes), the message type (1 byte),    */
/* and a CRC (2 bytes).                                              */
/*-------------------------------------------------------------------*/
void message_rx(message_t *msg, distance_measurement_t *d) {
  
  /* For testing communication */
  // if (msg->type == 253)
  // {
  //   /* Blinking behaviour */
  //   set_color(RGB(0, 3, 0));
  //   delay(50);
  //   set_color(RGB(3, 0, 0));
  //   delay(50);
  // }

  // printf("Message type: %d\n",msg->type);

  switch (msg->type)
  {
  case CONFIG_PARAMETERS_MSG:
  {
    int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
    int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
    int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);
    if (id1 == kilo_uid) {
      // unpack type
      sa_type = msg->data[1] >> 2 & 0x0F;
      // unpack payload
      sa_payload = ((msg->data[1]&0b11) << 8) | (msg->data[2]);
      levy_exponent = (double) (sa_payload & 0x1F) / 10.0;
      crw_exponent = (double) ((sa_payload >> 5) & 0x1F) / 10.0;
      if(sa_type == 3){
        openSpace_exp = true;
      }
      printf("kilo_uid %d, sa_type %d, crw_exponent %f, levy_exponent %f\n",kilo_uid, sa_type, crw_exponent, levy_exponent);
      // printf("kilo_uid %d, crw_exponent %f, levy_exponent %f\n",kilo_uid, crw_exponent, levy_exponent);
    }
    
    if (id2 == kilo_uid) {
      // unpack type
      sa_type = msg->data[4] >> 2 & 0x0F;
      // unpack payload
      sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
      levy_exponent = (double) (sa_payload & 0x1F) /10;
      crw_exponent = (double) ((sa_payload >> 5) & 0x1F) /10;
      if(sa_type == 3){
        openSpace_exp = true;
      }
      printf("kilo_uid %d, sa_type %d, crw_exponent %f, levy_exponent %f\n",kilo_uid, sa_type, crw_exponent, levy_exponent);
      // printf("kilo_uid %d, crw_exponent %f, levy_exponent %f\n",kilo_uid, crw_exponent, levy_exponent);
    }

    if (id3 == kilo_uid) {
      // unpack type
      sa_type = msg->data[7] >> 2 & 0x0F;
      // unpack payload
      sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
      levy_exponent = (double) (sa_payload & 0x1F) /10;
      crw_exponent = (double) ((sa_payload >> 5) & 0x1F) /10;
      if(sa_type == 3){
        openSpace_exp = true;
      }
      printf("kilo_uid %d, sa_type %d, crw_exponent %f, levy_exponent %f\n",kilo_uid, sa_type, crw_exponent, levy_exponent);
      // printf("kilo_uid %d, crw_exponent %f, levy_exponent %f\n",kilo_uid, crw_exponent, levy_exponent);

    }
    break;
  }

  case ARK_MSG: {
    // unpack message
    int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
    int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
    int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);
    if (id1 == kilo_uid) {
        // unpack type
        sa_type = msg->data[1] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[1]&0b11) << 8) | (msg->data[2]);
        new_sa_msg = true;
    }
    if (id2 == kilo_uid) {
        // unpack type
        sa_type = msg->data[4] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
        new_sa_msg = true;
    }
    if (id3 == kilo_uid) {
        // unpack type
        sa_type = msg->data[7] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
        new_sa_msg = true;
    }

    if(new_sa_msg==true) {
      if(sa_type==STATE_MESSAGE){
        update_state(DISCOVERED_TARGET);
        new_information = true;
        set_color(RGB(3, 0, 0));
      }

      else if(sa_type==PROXIMITY_MESSAGE && sa_payload!=0 && bouncing_ticks==0){
        /*wall avoidance*/
        proximity_sensor = sa_payload;
        bouncing_ticks = wall_avoidance_procedure(proximity_sensor);
        if (bouncing_ticks != 0){
          current_motion_type = BOUNCING_ANGLE;
          if (kilo_ticks < last_motion_ticks + straight_ticks) {
            straight_ticks_backup = straight_ticks - (kilo_ticks - last_motion_ticks);
          }
        }
      }

      else if(sa_type==RANDOM_ANGLE_MESSAGE && rotation_amount == 0.0){
        rotation_amount = (sa_payload & 0x7F) * M_PI / 127.0;
        if (((sa_payload >> 7) & 0x01) == 0){
          rotation_direction = LEFT;
        }
        else{
          rotation_direction = RIGHT;
        }

        current_motion_type = RANDOM_ROTATION;
      }

      else if(sa_type==BIAS_MESSAGE && rotation_amount == 0.0){
        rotation_amount = (sa_payload & 0x7F) * M_PI / 127.0;
        if (((sa_payload >> 7) & 0x01) == 0){
          rotation_direction = LEFT;
        }
        else{
          rotation_direction = RIGHT;
        }

        current_motion_type = POINTING_HOME;
      }

      new_sa_msg = false;
    }

    break;
  }

  case KILOBOTS_MSG:
  {
    //If distance is too much, the message will be discarded
    uint8_t cur_distance = estimate_distance(d);
    if (cur_distance > 100) //100 mm  
    {
      break;
    }

    /* ----------------------------------*/
    /* KB interactive message            */
    /* ----------------------------------*/
    else if (msg->data[0]!=kilo_uid && msg->crc==message_crc(msg)) 
    {
      new_information = true;
      if (current_state != DISCOVERED_TARGET)
      {
        update_state(COMMUNICATED_TARGET);
        set_color(RGB(0, 3, 0));
      }
    }
    break;
  }
  
  

  case IDLE:
  default:
    break;
  }

}

/*-------------------------------------------------------------------*/
/* Function to broadcast a message                                        */
/*-------------------------------------------------------------------*/
void broadcast()
{

  if (new_information && !sending_msg && kilo_ticks > last_broadcast_ticks + max_broadcast_ticks)
  {
    last_broadcast_ticks = kilo_ticks;
    sending_msg = true;
  }
}

/*-------------------------------------------------------------------*/
/* Function implementing the crwlevy random walk                     */
/*-------------------------------------------------------------------*/
void random_walk()
{
  switch (current_motion_type) 
  {
    case TURN_LEFT:
      // printf("LEFT\n");
    case TURN_RIGHT:
      // printf("RIGHT\n");
      if (kilo_ticks > last_motion_ticks + turning_ticks) {
        /* start moving forward */
        last_motion_ticks = kilo_ticks;

        if(rotation_amount != 0.0) {
          rotation_amount = 0.0;
          current_state = current_state_backup;
          straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
        }

        if(bouncing_ticks != 0.0) {
          bouncing_ticks = 0.0;
          if(straight_ticks_backup > 0){
            straight_ticks = straight_ticks_backup;
            straight_ticks_backup = 0;
          }
          else
            straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
        }

        set_motion(FORWARD);
      }
      break;

    case FORWARD:
      // printf("FORWARD\n");
      /* if moved forward for enough time turn */
      if (kilo_ticks > last_motion_ticks + straight_ticks){
        /* perform a random turn */
        last_motion_ticks = kilo_ticks;
        
        if (rand_soft() % 2){
          set_motion(TURN_LEFT);
        }
        else{
          set_motion(TURN_RIGHT);
        }

        /* rand_soft() returns a random number in [0,254]
           so if you want evaluate the 20% (100/5) --> ~= 51 (254/5)
        */
        if(rand_soft() < 255/5 && openSpace_exp){
          current_state_backup = current_state;
          update_state(WAITING_BIAS);
          set_color(RGB(0,0,3));
          set_motion(STOP);
          // printf("Waiting for the angle\n");
        }

        double angle = 0;
        if(crw_exponent == 0) {
          angle = (uniform_distribution(0, (M_PI)));
        }
        else{
          angle = fabs(wrapped_cauchy_ppf(crw_exponent));
        }
        turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
        straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
      }
      break;

    case POINTING_HOME:
    case RANDOM_ROTATION:
      set_motion(rotation_direction);
      current_motion_type = rotation_direction;
      last_motion_ticks = kilo_ticks;
      turning_ticks = (uint32_t)((rotation_amount / M_PI) * max_turning_ticks);
      break;

    case BOUNCING_ANGLE:
      set_motion(rotation_direction);
      current_motion_type = rotation_direction;
      last_motion_ticks = kilo_ticks;
      turning_ticks = bouncing_ticks;
      break;


    case STOP:
    default:
      set_motion(STOP);
  }
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop()
{  
  //turn on the right led color
  check_state();
  // print_state();

  if(crw_exponent!=-1 && levy_exponent!=-1)
  {
    if(received_coefficients == false){
      set_motion(FORWARD);
      received_coefficients = true;
    }
    random_walk();
    broadcast();  
  }

}

/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main()
{
    sector_base =  pow(2, COLLISION_BITS / 2) - 1;
    
    kilo_init();
    // register message reception callback
    kilo_message_rx = message_rx;
    // register message transmission callback
    kilo_message_tx = message_tx;
    // register transmission success callback
    kilo_message_tx_success = message_tx_success;

     
    kilo_start(setup, loop);

    return 0;
}
