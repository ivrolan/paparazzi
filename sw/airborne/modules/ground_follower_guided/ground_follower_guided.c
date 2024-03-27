// @ivrolan

#include "modules/ground_follower_guided/ground_follower_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
// #include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define GROUND_FOLLOWER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[ground_follower_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GROUND_FOLLOWER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  IDLE,
  STOPPING_TO_OBS,
  STOPPING_TO_SAFE
};

// define settings
float oa_color_count_frac = 0.18f;

// define and initialise global variables
enum navigation_state_t navigation_state = OBSTACLE_FOUND; //SEARCH_FOR_SAFE_HEADING;
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]

float avoidance_turning_direction = 0;

float compensate_fwd = -0.5f;
float compensate_ang = -0.1f;

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

struct ground_filter_msg_t rec_ground_filter_msg;


/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef GROUND_FILTER_DETECTION_ID
#define GROUND_FILTER_DETECTION_ID ABI_BROADCAST
#endif
static abi_event ground_filter_detection_ev;
static void ground_filter_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                    int16_t count_left, 
                                    int16_t count_center, 
                                    int16_t count_right,
                                    int16_t __attribute__((unused)) extra)
{
    rec_ground_filter_msg.count_left = count_left; 
    rec_ground_filter_msg.count_center = count_center; 
    rec_ground_filter_msg.count_right = count_right; 
}

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void ground_follower_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the ground filter outputs
  AbiBindMsgGROUND_FILTER_DETECTION(CUSTOM_GROUND_FILTER_DETECTION_ID, &ground_filter_detection_ev, ground_filter_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void ground_follower_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  PRINT("left count: %d  risk: %d right count: %d \n", rec_ground_filter_msg.count_left, rec_ground_filter_msg.count_center, rec_ground_filter_msg.count_right);

  int32_t max_risk = 36;  // 0.45 * 80, 0.45 was chosen based on ROC curve from Python script
  int32_t occ_pix_max = 7000;

  float speed_sp = 0.6f;  
  int ang_vel = 40;
  // update our safe confidence using color threshold
  // if(rec_ground_filter_msg.count_center > max_risk){ // we're gonna hit
  //   navigation_state = OBSTACLE_FOUND;
  //   // obstacle_free_confidence-= 2;
  // } else {
  //   // obstacle_free_confidence++;
  //   // let's ignore the confidence for now
  //   navigation_state = SAFE;  // be more cautious with positive obstacle detections
  // }

  // //   bound obstacle_free_confidence
  // Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  // float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);
    // navigation_state = IDLE;
  
  PRINT("state %d ", navigation_state);
  
  switch (navigation_state){
    case SAFE:
      // Move forward
      guidance_h_set_body_vel(speed_sp, 0);

      if(rec_ground_filter_msg.count_center > max_risk){ // we're gonna hit
        guidance_h_set_body_vel(0, 0); 

        navigation_state = STOPPING_TO_OBS;
      }
      break;
    case STOPPING_TO_OBS:
      // apply a little of backward velocity to compensate for the forward velocity
      guidance_h_set_body_vel(compensate_fwd * speed_sp, 0);
      navigation_state = OBSTACLE_FOUND;

      // chooose avoidance_turning_direction
      // positive angle means turning right

      if (rec_ground_filter_msg.count_right > rec_ground_filter_msg.count_left) {
        avoidance_turning_direction = 1.0f;
        VERBOSE_PRINT("Turn right\n");
      } else{
        avoidance_turning_direction = -1.0f;
        VERBOSE_PRINT("Turn left\n");
      }

      // with a small chance (10%), override this for a random value
      int chance = rand() % 10;
      
      if (chance < 1) {
        chooseRandomIncrementAvoidance();
        VERBOSE_PRINT("Random direction chosen");
      }

      break;
    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      
      //chooseMaxFreeIncrementAvoidance();
      
      // start turn
      guidance_h_set_heading_rate(avoidance_turning_direction *  RadOfDeg(ang_vel));

      //navigation_state = SEARCH_FOR_SAFE_HEADING;
      if(rec_ground_filter_msg.count_center < max_risk){ // We're safe
        guidance_h_set_body_vel(0, 0);

        navigation_state = STOPPING_TO_SAFE;
      }
      break;
    case STOPPING_TO_SAFE:
      // compensate for the turning velocity
      guidance_h_set_heading_rate(compensate_ang * avoidance_turning_direction * RadOfDeg(ang_vel));
      navigation_state = SAFE;
      break;
    // ---- for now only make use of the SAFE and OBSTACLE_FOUND
   
    case IDLE:
        break;
    default:
      break;
  }
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_turning_direction = 1.f;
    // VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else {
    avoidance_turning_direction = -1.f;
    // VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  return false;
}
