/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * This file contains the main navigation algorithm for obstacle avoidance of a drone flying in the cyberzoo, as part of the final assignment of AE4317 in TU Delft.
 * The code is aided by two computer vision algorithm that can be located in cv_detect_color_object.c and opencv_example.cpp. THe first file focuses on detecting the
 * the green pixels of an image corresponding to the floor. It uses this parameter to decide if it's safe to fly in a certain direction. The count of green pixels is
 * compared with two thresholds to check whether the green pixel count correspond to an obstacle detected or if the drone is getting out of boundaries. Further,
 * the green count is splitted into left and right so the drone can decide where to turn in case it is not safe to continue straight.
 *
 * The second algorithm that can be found in orangecv_example.cpp, although it's feedback is retrieved at all times, it's only used as an auxiliary method in cases of
 * emergency, where the previous algorithm fails to detect an unsafe situation. Through functions of OpenCV, this algorithm detects vertical lines that correspond
 * to edges of objects in the cyberzoo. Once a vertical line is detected, it will immediately force the drone to drop the velocity to zero and start turning to another
 * dircection.
 *
 * This file models the navigation behavior of the drone in a state machine.  JUmping from one state to another will depend on level of confidence whose values is
 * defined by the feedback from the computer vision algorithms. At the same time, this level of confidence is used to set the velocity at which the drone can fly.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/opencv_example.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "generated/airframe.h"
#include "state.h"
#include "pthread.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"
#include <stdio.h>


#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif
static pthread_mutex_t mutex;

uint8_t chooseRandomIncrementAvoidance(void);

//DEFINITION OF THE STATES IN THE STATE MACHINE
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};

// define settings
float oag_color_count_frac = 0.175f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.0625f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.7f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(50.f);  // heading change setpoint for avoidance [rad/s]


// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
uint32_t cnt_right_green;			    // local variable to store the count of green pixels at the right
uint32_t cnt_left_green;				   // local variable to store the count of green pixels at the left
uint8_t vertical;					   // auxiliary variable to force the drone to keep turning once a vertical line is detected
int vertical_lines[20];				  // local variable to store the location of the vertical lines detected.
float speed_sp;						  // velocity at which the drone flies.




//LEVEL OF CONFIDENCE
const int16_t max_trajectory_confidence = 3;    //Highest possible level of confidence


// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif


//This call back is used to receive the floor count (green pixels) from the CV DETECT COLOR OBJECT.
#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;


///The values coming from the computer vision algorithms are assigned to local variables
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;					//Total count of green pixels
  floor_centroid = pixel_y;					//Centroid
  cnt_left_green=cnt_left;					//Count of green at left
  cnt_right_green=cnt_right;				//Count of green at right
  for(uint16_t j = 0; j < 20 ; j++){		//Location of vertical lines detected.
	  vertical_lines[j]=array[j];}
}


  //  Initialisation function

void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  avoidance_heading_direction = 1.f;

  // Bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);


}



 //Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading

void orange_avoider_guided_periodic(void)
{


  // Only run the module if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }

  //Compute current color thresholds
  int32_t obstacle_count_threshold = oag_color_count_frac * 5 * (front_camera.output_size.h - 110);   //5 because it only checks the count in 5 rows and -110 to take out lateral bounds
  int32_t bound_count_threshold = oag_floor_count_frac * 5 * front_camera.output_size.h;			  //5 because it only checks the count in 5 rows
  float floor_centroid_frac = floor_centroid / (float)(front_camera.output_size.h) / 2.f;


  //Print to paparazzi window some important values to check in real-time.
  VERBOSE_PRINT("Line detection takes %d us \n", time_for_vertical_lines);
  VERBOSE_PRINT("Visual method takes %d us \n" , floor_detection_time);
  VERBOSE_PRINT("Confidence_ %d  State: %d \n", obstacle_free_confidence, navigation_state);
  VERBOSE_PRINT("Left count: %d   Right count: %d\n",cnt_left_green,cnt_right_green);
  uint8_t number_of_lines=0;
  for(uint16_t j = 0; j < 20 ; j++){
	  if (vertical_lines[j]!=0){
		  number_of_lines++;}
	  }
  VERBOSE_PRINT("Vertical lines detected: %d\n", number_of_lines);
  VERBOSE_PRINT("Vertical: %d\n", vertical);

  // Update our safe confidence using obstacle  threshold
  if(floor_count > obstacle_count_threshold){
    obstacle_free_confidence++;
  } else{
    obstacle_free_confidence -= 2;
  }

  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);  //Bound level of confidence from 0-3

  speed_sp = fminf(oag_max_speed, 0.25f * obstacle_free_confidence);  //Define speed as minimum value between max allowed speed and the one given by level of confidence

  switch (navigation_state){
    case SAFE:
      if (floor_count < bound_count_threshold){
        navigation_state = OUT_OF_BOUNDS;      //Activated when floor count is below the lower green threshold
      } else if ((obstacle_free_confidence == 0)||((number_of_lines!=0 && obstacle_free_confidence>=2))) {   //Activate depending on level of confidence or if a vertical line is detected
        navigation_state = OBSTACLE_FOUND;
      }else{
    	vertical=0;		//Reset vertical so the drone stops turning
        guidance_h_set_guided_body_vel(speed_sp, 0);
      }
      break;


    case OBSTACLE_FOUND:
    	//Choose a direction to turn
      chooseRandomIncrementAvoidance();
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;


    case SEARCH_FOR_SAFE_HEADING:
    	//If an object was detected speed lowers proportionally to the confidence, but if a green line was detected, speed drops to zero.
    	if (number_of_lines == 0 && vertical==0){
    	guidance_h_set_guided_body_vel(0.15f*speed_sp+0.1f, 0);
    	}else{
    	guidance_h_set_guided_body_vel(0, 0);
    	vertical=1;
    	}
    	guidance_h_set_guided_heading_rate(avoidance_heading_direction * oag_heading_rate);
      if (obstacle_free_confidence > 0 && number_of_lines==0){
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
      break;


    case OUT_OF_BOUNDS:
      // Stop
      guidance_h_set_guided_body_vel(0, 0);
      // start turn back into arena
      guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(30));
      navigation_state = REENTER_ARENA;

      break;

    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      if (floor_count >= bound_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SAFE;
      }
      break;
    default:
      break;
  }

  ///RESET GLOBAL VARIABLE SO THE VALUES DOESN'T ACCUMULATE FOR THE NEXT ITERATION
  pthread_mutex_lock(&mutex);
  cnt_right=0;
  cnt_left=0;
  pthread_mutex_unlock(&mutex);
  return;

}


//Function to choose the direction of turning when looking for a new direction
uint8_t chooseRandomIncrementAvoidance(void)
{

  // Turn to the side with the highest green count
  if (cnt_right_green<cnt_left_green){
    avoidance_heading_direction = -1.f; //Negative is left
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);

  } else  {
    avoidance_heading_direction = 1.f; //Positive is right
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);

  }
  return false;
}


