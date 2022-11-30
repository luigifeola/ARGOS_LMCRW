#include <kilolib.h>

/*Max number of ticks*/
//const double max_time = 9288; //10^3
/* counters for motion, turning and random_walk */
const double std_motion_steps = 5*16;
double levy_exponent = 1.2; // 2 is brownian like motion
double crw_exponent = 0.8; // go straight often
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 120;//160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint16_t max_straight_ticks = 320; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
uint16_t straight_ticks = 0; // keep count of ticks of going straight


uint32_t last_motion_ticks = 0;


void setup()
{
}

void loop()
{
    // // Set the LED green.
    // set_color(RGB(0, 1, 0));
    // // Spinup the motors to overcome friction.
    // spinup_motors();
    // // Move straight for 2 seconds (2000 ms).
    // set_motors(kilo_straight_left, kilo_straight_right);
    // delay(2000);
    
    // Set the LED red.
    set_color(RGB(1, 0, 0));
    // Spinup the motors to overcome friction.
    spinup_motors();
    if(kilo_ticks <= max_turning_ticks)
    {
      // Turn left for 2 seconds (2000 ms).
      set_motors(kilo_turn_left, 0);
      
    }
    else
    {
      // Set the LED off.
      set_color(RGB(0, 0, 0));
      // Stop for half a second (500 ms).
      set_motors(0, 0);
    }
    
    // // Set the LED blue.
    // set_color(RGB(0, 0, 1));
    // // Spinup the motors to overcome friction.
    // spinup_motors();
    // // Turn right for 2 seconds (2000 ms).
    // set_motors(0, kilo_turn_right);
    // delay(2000);
    
    // // Set the LED off.
    // set_color(RGB(0, 0, 0));
    // // Stop for half a second (500 ms).
    // set_motors(0, 0);
    // delay(500);
}

int main()
{
    kilo_init();
    kilo_start(setup, loop);
    
    return 0;
}
