#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[test_receiver->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define ORANGE_AVOIDER_VERBOSE TRUE

#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  TURN_LEFT,
  TURN_RIGHT,
  OUT_OF_BOUNDS
};

// Copied from orange avoider
static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);


// global vars
int32_t ground_count_left = 0;
int32_t ground_count_center = 0;
int32_t ground_count_right = 0;   
enum navigation_state_t navigation_state = OBSTACLE_FOUND;
float waypoint_step = 1.0f;  // [m]
int center_threshold = 2000;  // px TO BE TUNED
float heading_increment = 5.f;          // heading angle increment [deg]



// for the communication with the GROUND_FILTER
#ifndef GROUND_FILTER_DETECTION_ID
#define GROUND_FILTER_DETECTION_ID ABI_BROADCAST
#endif
static abi_event ground_detection_ev;
static void ground_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) received_count_left,
                               int16_t __attribute__((unused)) received_count_center,
                               int16_t __attribute__((unused)) received_count_right,
                               int16_t __attribute__((unused)) extra)
{
  
  ground_count_left = received_count_left;
  ground_count_center = received_count_center;
  ground_count_right = received_count_right;

  //PRINT("Ground Detection Callback");
}

extern void test_receiver_init(void) {
  PRINT("test receiver init");

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgGROUND_FILTER_DETECTION(TEST_RECEIVER_GROUND_FILTER_DETECTION_ID, &ground_detection_ev, ground_detection_cb);

}
extern void test_receiver_periodic(void) {
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  switch (navigation_state) {
    case SAFE:
      VERBOSE_PRINT("Moving forward.\n");

      // Move trajectory waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * waypoint_step);

      //Check new state
       if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (ground_count_center <= center_threshold){
        navigation_state = OBSTACLE_FOUND;
      } else {
        // Move Goal waypoint forward
        moveWaypointForward(WP_GOAL, waypoint_step);
      }

      break;
    
    case OBSTACLE_FOUND:
      VERBOSE_PRINT("Obstacle encountered\n");

       // stop moving
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // Make decision based on number of ground pixels left or right (tie breaker included)
      if (ground_count_left > ground_count_right) {
        navigation_state = TURN_LEFT;
      } else {
        navigation_state = TURN_RIGHT;
      }

      break;

    case TURN_LEFT:
      VERBOSE_PRINT("Turning left.\n");
      increase_nav_heading(-heading_increment);

      // Check if it is safe to move
      if (ground_count_center >= center_threshold){
        navigation_state = SAFE;
      }

      break;

    case TURN_RIGHT:
      VERBOSE_PRINT("Turning right.\n");
      increase_nav_heading(heading_increment);

      // Check if it is safe to move
      if (ground_count_center >= center_threshold){
        navigation_state = SAFE;
      }

      break;
    
    case OUT_OF_BOUNDS:
      VERBOSE_PRINT("Out of bounds. \n");
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);

        // ensure direction is safe before continuing
        navigation_state = TURN_RIGHT;
      }
      break;
    
    default:
      break;
  }
  return;

  //PRINT("[countL:%d, countC:%d, countR:%d]\n", ground_count_left, ground_count_center, ground_count_right);
}








// Copied from Orange Avoider
/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  VERBOSE_PRINT("Changed heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}

