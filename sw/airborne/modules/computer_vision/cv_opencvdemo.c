/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "modules/computer_vision/lib/vision/image.h"
#include "navigation.h"
#include "generated/flight_plan.h"
#include "std.h"

#include <stdlib.h>
#include <string.h>

#include "subsystems/datalink/telemetry.h"
int eyes_closed_go = 0;
int wait_here_time=0;
int go_right_time=0;
int go_backwards_time=0;
int go_left_time=0;
enum DRONE_STATE dronerace_drone_state=GO_SAFETY;
//void guided_stay_wp(uint8_t wp){
//	 struct EnuCoor_f enu_i_wp = waypoints[wp].enu_f;
//	struct EnuCoor_f actually_ned;
//	ENU_OF_TO_NED(actually_ned, enu_i_wp);
//	guidance_h_set_guided_pos(actually_ned.x, actually_ned.y);
//}
uint8_t guided_stay_wp(uint8_t wp){
	 struct EnuCoor_f enu_i_wp = waypoints[wp].enu_f;
	struct EnuCoor_f actually_ned;
	ENU_OF_TO_NED(actually_ned, enu_i_wp);
	guidance_h_set_guided_pos(actually_ned.x, actually_ned.y);
	return 0;
}
float getPosErrorMeters(uint8_t wp){
	struct EnuCoor_f enu_i_wp = waypoints[wp].enu_f;

	struct EnuCoor_f my_place =*stateGetPositionEnu_f();

	double squared = fabs(my_place.x-enu_i_wp.x)*fabs(my_place.x-enu_i_wp.x)+\
			fabs(my_place.y-enu_i_wp.y)*fabs(my_place.y-enu_i_wp.y);
	return sqrt(squared);
//	NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_posi);
}
uint8_t start_fly_through(){
	dronerace_drone_state=GO_THROUGH_WINDOW;
	return 0;
}

uint8_t should_go_safety(){
	if(dronerace_drone_state==GO_SAFETY){
		return 1;
	}
	return 0;
}
// Function
struct image_t* opencv_func(struct image_t* img);
struct image_t* opencv_func(struct image_t* img)
{
  if (img->type == IMAGE_YUV422)
  {
    // Call OpenCV (C++ from paparazzi C function)
    opencv_example((char*) img->buf, img->w, img->h);
  }

  DOWNLINK_SEND_OBSTACLE_RACE_INFO(DefaultChannel, DefaultDevice, &distance_pixels,&center_pixels,&left_height,&right_height);



  float yaw = stateGetNedToBodyEulers_f()->psi;
  float viewingAngle=0.45;//radians
  float diff = loc_y-(img->h/2);

  float unexplainedOffset=50.0;
  diff+=unexplainedOffset;
  double pixelsPerDegree = viewingAngle/img->h;
  yaw += pixelsPerDegree * diff;
  float totalHeight = left_height + right_height;
  float ratio = left_height/totalHeight;
  float ratio_wanted = 0.44;
  int distance_pixels_between_just_go=125;
  totalHeight/=100.0;

  if(dronerace_drone_state==GO_THROUGH_WINDOW){
	  if(eyes_closed_go>0){
		  eyes_closed_go--;
		  if(eyes_closed_go<=0){
			  eyes_closed_go=0;
			  wait_here_time=40;
		  }
		  guidance_h_set_guided_body_vel(1.0,0);
	  }
	  else if(wait_here_time>0){
		  wait_here_time--;
		  if(wait_here_time<=0){
			  go_right_time=100;


		  }
		  dronerace_drone_state=GO_SAFETY;
	  }
  }
  else if(dronerace_drone_state == DETECT_WINDOW){
	  guidance_h_set_guided_heading(yaw);

	  if(too_close){
		  guidance_h_set_guided_body_vel(-1.0,0);
	  }
	  else{

		  if(ratio < ratio_wanted || ratio > (1.0-ratio_wanted)){
			  if(ratio < ratio_wanted){
				  guidance_h_set_guided_body_vel(0.0,-ratio/totalHeight);
			  }
			  else{
				  guidance_h_set_guided_body_vel(0.0,ratio/totalHeight);
			  }
		  }
		  else{
			  if(distance_pixels>distance_pixels_between_just_go){
				  eyes_closed_go=60;
				  dronerace_drone_state=GO_THROUGH_WINDOW;
				  guidance_h_set_guided_body_vel(0.5,0);
			  }
			  else{
				  guidance_h_set_guided_body_vel(0.5,diff/img->h);
			  }
		  }
	  }
  }

  return img;
}

void opencvdemo_init(void)
{
  cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func);
  opencv_init_rects();
}

