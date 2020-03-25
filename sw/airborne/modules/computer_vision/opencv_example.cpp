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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "opencv_example.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <algorithm>
#include <stdint.h>

cv::Mat
preprocess_image(const cv::Mat& image)
{
    // Rotate the images 90 degrees for better viewing
    cv::Mat rotated_image;
    cv::rotate(image, rotated_image, cv::ROTATE_90_COUNTERCLOCKWISE);
    // Convert images to gray scale
    cv::Mat gray_image;
    cv::cvtColor(rotated_image, gray_image, cv::COLOR_BGR2GRAY);
    // Blur images to reduce image noise
    cv::Mat blurred_image;
    cv::medianBlur(gray_image, blurred_image, 3);
    // Adaptive threshold method
    cv::Mat thresholded_image;
    cv::adaptiveThreshold(blurred_image, thresholded_image, 255,
                          cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY_INV,
                          11, 3);
    return thresholded_image;
}

int opencv_example(char *img, int width, int height)
{
	// int, float = 4
	// double = 4-8
	// short = 2-4
	// bool = 1-2
	// char = 1

  // Create a new image, using the original bebop image.
  Mat M(height, width, CV_8UC2, img);
  Mat image;

#if OPENCVDEMO_GRAYSCALE
  //  Grayscale image example
  cvtColor(M, image, CV_YUV2GRAY_Y422);
  // Canny edges, only works with grayscale image
  int edgeThresh = 35;
  Canny(image, image, edgeThresh, edgeThresh * 3);
  // Convert back to YUV422, and put it in place of the original image
  grayscale_opencv_to_yuv422(image, img, width, height);
#else // OPENCVDEMO_GRAYSCALE
  // Color image example
  // Convert the image to an OpenCV Mat
  cvtColor(M, image, CV_YUV2BGR_Y422);
  // Blur it, because we can
  blur(image, image, Size(5, 5));
  // Convert back to YUV422 and put it in place of the original image
  colorbgr_opencv_to_yuv422(image, img, width, height);
#endif // OPENCVDEMO_GRAYSCALE

  return 0;
}
