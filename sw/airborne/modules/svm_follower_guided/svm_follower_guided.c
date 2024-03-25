// @ivrolan

#include "modules/svm_follower_guided/svm_follower_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
// #include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define SVM_FOLLOWER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[svm_follower_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if SVM_FOLLOWER_VERBOSE
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
enum navigation_state_t navigation_state = IDLE;
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]

float avoidance_turning_direction = 0;

float compensate_fwd = -0.5f; //-0.2f;
float compensate_ang = -0.1f;

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

struct svm_decision_msg_t rec_svm_decision_msg;


/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef SVM_DECISION_MSG_ID
#define SVM_DECISION_MSG_ID ABI_BROADCAST
#endif
static abi_event svm_decision_msg_ev;
static void svm_decision_msg_cb(uint8_t __attribute__((unused)) sender_id,
                                    float value, 
                                    bool decision,
                                    int16_t __attribute__((unused)) extra)
{
    rec_svm_decision_msg.value = value; 
    rec_svm_decision_msg.decision = decision;
    rec_svm_decision_msg.updated = true;
}

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void svm_follower_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();
  rec_svm_decision_msg.updated = false;


  // bind our colorfilter callbacks to receive the ground filter outputs
  AbiBindMsgSVM_DECISION_MSG(CUSTOM_SVM_DECISION_MSG_ID, &svm_decision_msg_ev, svm_decision_msg_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void svm_follower_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  //PRINT("left count: %d  risk: %d right count: %d \n", rec_ground_filter_msg.count_left, rec_ground_filter_msg.count_center, rec_ground_filter_msg.count_right);
  PRINT("value: %.2f, decision: %d\n", rec_svm_decision_msg.value, rec_svm_decision_msg.decision);
  int32_t max_risk = 36;  // 0.45 * 80, 0.45 was chosen based on ROC curve from Python script
  int32_t occ_pix_max = 7000;

  float speed_sp = 0.5f;  
  int ang_vel = 15;
  
  PRINT("state %d ", navigation_state);
  
  switch (navigation_state){
    case SAFE:
      // Move forward
      guidance_h_set_body_vel(speed_sp, 0);

      if(!rec_svm_decision_msg.decision){ // we're gonna hit
        guidance_h_set_body_vel(0, 0);

        navigation_state = STOPPING_TO_OBS;
      }
      break;
    case STOPPING_TO_OBS:
      // apply a little of backward velocity to compensate for the forward velocity
      guidance_h_set_body_vel(compensate_fwd * speed_sp, 0);
      navigation_state = OBSTACLE_FOUND;

      // chooose avoidance_turning_direction

      chooseRandomIncrementAvoidance();
      VERBOSE_PRINT("Random direction chosen");

      break;
    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      
      //chooseMaxFreeIncrementAvoidance();
      
      // start turn
      guidance_h_set_heading_rate(avoidance_turning_direction *  RadOfDeg(ang_vel));

      //navigation_state = SEARCH_FOR_SAFE_HEADING;
      if(rec_svm_decision_msg.decision){ // We're safe
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
      guidance_h_set_body_vel(0, 0);
      // if(rec_ground_filter_msg.count_center > max_risk){ // we're gonna hit
      //   navigation_state = OBSTACLE_FOUND;
      // } else {
      //   // let's ignore the confidence for now
      //   navigation_state = SAFE;  // be more cautious with positive obstacle detections
      // }
      if (rec_svm_decision_msg.updated) {
        if (!rec_svm_decision_msg.decision) {
          navigation_state = OBSTACLE_FOUND;
        } else {
          navigation_state = SAFE;
        }
      }
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
