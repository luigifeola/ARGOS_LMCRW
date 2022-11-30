#include "kilolib.h"
#include <stdlib.h>
#include<stdio.h>
#include <math.h>

typedef enum {  // Enum for different motion types
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    STOP = 3,
    FORWARD = 4,
} motion_t;


typedef enum {  // Enum for boolean flags
    false = 0,
    true = 1,
} bool;


motion_t current_motion_type = STOP;            // Current motion type

/***********WALK PARAMETERS***********/
const float std_motion_steps = 20*16; // variance of the gaussian used to compute forward motion
const float levy_exponent = 1.4; // 2 is brownian like motion (alpha)
const float  crw_exponent = 0.9; // higher more straight (rho)
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 120; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;
bool rotate = false;


/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type) {
  if(current_motion_type != new_motion_type ) 
  {
    switch( new_motion_type ) {
    case FORWARD:
      spinup_motors();
      set_motors(kilo_straight_left,kilo_straight_right);
      break;
    case TURN_LEFT:
      spinup_motors();
      set_motors(kilo_turn_left,0);
      break;
    case TURN_RIGHT:
      spinup_motors();
      set_motors(0,kilo_turn_right);
      break;
    case STOP:
    default:
      set_motors(0,0);
    }
    current_motion_type = new_motion_type;
  }
}

/*-------------------------------------------------------------------*/
/* Function implementing the LMCRW random walk                       */
/*-------------------------------------------------------------------*/
void random_walk()
{
  switch (current_motion_type) 
  {
  case TURN_LEFT:
  case TURN_RIGHT:
    /* if turned for enough time move forward */
    if (kilo_ticks > last_motion_ticks + turning_ticks) {
      /* start moving forward */
      last_motion_ticks = kilo_ticks;
      set_motion(FORWARD);
    }
    break;

    case FORWARD:
    /* if moved forward for enough time turn */
    if (kilo_ticks > last_motion_ticks + straight_ticks) {
      /* perform a random turn */
      last_motion_ticks = kilo_ticks;
      
      set_motion(TURN_LEFT);
      
      turning_ticks = (uint32_t)((M_PI_2 / M_PI) * max_turning_ticks);
      straight_ticks = max_straight_ticks/3;
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
void setup() {
    /* Initialise LED and motors */
    set_color(RGB(0,0,0));
    set_motors(0,0);

    /* Initialise random seed */
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    set_motion(FORWARD);
}

void loop()
{
    
  random_walk();
    
}

int main()
{
    // Initialize the hardware.
    kilo_init();
    // Register the program.
    kilo_start(setup, loop);
    
    return 0;
}
