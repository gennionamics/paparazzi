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
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <algorithm>
#include <stdint.h>
#include "opencv_image_functions.h"



int s;
int array[20];


// FUNCTION 1: Pre-process image ready for obstacle edge detection
cv::Mat
preprocess_image(const cv::Mat& image)
{
    // Rotate the images 90 degrees for better viewing
    cv::Mat rotated_image;
    cv::rotate(image, rotated_image, cv::ROTATE_90_COUNTERCLOCKWISE);
    // Convert images to gray scale
    cv::Mat gray_image;
    cv::cvtColor(rotated_image, gray_image, CV_YUV2GRAY_Y422);
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

// FUNCTION 2: Check if a pixel is part of an edge
bool
is_edge_pixel(int grayscale_intensity)
{
    return grayscale_intensity == 255;
}

// FUNCTION 3: Scan the pre-processed image for obstacle vertical edge lines
// FUNCTION 3a
std::vector<int>
scan_for_vertical_lines_directional(const cv::Mat& image,
                                    int scan_height,
                                    int line_width,
                                    float min_ratio,
                                    bool start_from_top)
{
    std::vector<int> xcoords;
    int image_width  = image.cols;
    int image_height = image.rows;

    int yrange_start = 0;
    int yrange_end   = scan_height;
    int yrange_step  = 1;

    if (!start_from_top)
    {
        yrange_start = image_height - 1;
        yrange_end   = image_height - scan_height - 1;
        yrange_step  = -1;
    }

    for (int x = 0; x < image_width - line_width + 1; ++x) {
        int total_pixels_in_segment = line_width * scan_height;
        int edge_pixels_in_segment = 0;
        for (int y = yrange_start; y < yrange_end; y += yrange_step) {
            for (int i = 0; i < line_width; ++i) {
                int grayscale_intensity = image.at<uint8_t>(y, x+i);
                if (is_edge_pixel(grayscale_intensity)) {
                    ++edge_pixels_in_segment;
                }
            }
        }
        float edge_pixels_ratio = float(edge_pixels_in_segment) / float(total_pixels_in_segment);
        if (edge_pixels_ratio >= min_ratio) {
            xcoords.push_back(x);
        }
    }
    return xcoords;
}

// FUNCTION 3b
std::vector<int>
scan_for_vertical_lines_from_top(const cv::Mat& image,
                                 int scan_height,
                                 int line_width,
                                 float min_ratio)
{
    return scan_for_vertical_lines_directional(image, scan_height, line_width, min_ratio, true);
}

// FUNCTION 3c
std::vector<int>
scan_for_vertical_lines_from_bottom(const cv::Mat& image,
                                    int scan_height,
                                    int line_width,
                                    float min_ratio)
{
    return scan_for_vertical_lines_directional(image, scan_height, line_width, min_ratio, false);
}

// FUNCTION 3d
std::vector<int>
scan_for_vertical_lines(const cv::Mat& image,
                        int scan_height,
                        int line_width,
                        float min_ratio)
{
    std::vector<int> from_top    = scan_for_vertical_lines_from_top(image, scan_height, line_width, min_ratio);
    std::vector<int> from_bottom = scan_for_vertical_lines_from_bottom(image, scan_height, line_width, min_ratio);
    std::vector<int> lines;
    std::merge(from_top.begin(),    from_top.end(),
               from_bottom.begin(), from_bottom.end(),
               std::back_inserter(lines));
    // Remove duplicated lines scanned from top and from bottom
    std::sort(lines.begin(), lines.end());
    lines.erase(std::unique(lines.begin(), lines.end()), lines.end());

    // Remove neighboring x-coordinates that actually describe the same obstacle edge
    // e.g. xcoord = 123 or 124 actually refers to the same vertical edge line
    const int discretization_threshold = 1;
    std::vector<int> discretized;
    if (lines.size() == 1) {
        discretized.push_back(lines[0]);
    } else if (lines.size() > 1) {
        for (int i = 1; i < lines.size(); ++i) {
            if (lines[i] - lines[i-1] > discretization_threshold) {
                discretized.push_back(lines[i-1]);
            }
            if (i == lines.size()-1) {
                discretized.push_back(lines[i]);
            }
        }
    }

    return discretized;
}
void
draw_image_with_lines(cv::Mat* image,
                      int scan_height,
                      const std::vector<int> &lines)
{
    for (float xcoord : lines) {
        for (int row = 0; row <= scan_height; ++row) {
            using Vec3u8 = cv::Vec<uint8_t, 3>;
            image->at<Vec3u8>(xcoord, row) = Vec3u8(12, 255, 36);
        }
    }
    //cv::imshow(WINDOW_TITLE, *image);
   // cv::waitKey(0);
}


int* opencv_example(char *img, int width, int height)
{

  // Create a new image, using the original bebop image.
  cv::Mat M(height, width, CV_8UC2, img);
  //cv::Mat image;

/*#if OPENCVDEMO_GRAYSCALE
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
#endif // OPENCVDEMO_GRAYSCALE*/


    //cv::VideoCapture capture("/home/sunyi/Documents/vertical_test/%d.jpg");
   // cv::VideoCapture capture(struct image_t *img);
  /*  if (!capture.isOpened())
    {
        std::cout << "Could not open the video capture!" << std::endl;
        return -1;
    }*/
   // while (true)
    //{
        // Capture and display a full folder of images
       // cv::Mat frame;
      /*  if (!capture.read(frame)) {
            break;
        }*/
        // Rotate the original colorful image as a canvas for the function draw_image_with_lines
        cv::Mat rotated_original;
       cv::rotate(M, rotated_original, cv::ROTATE_90_COUNTERCLOCKWISE);
        // Generate the pre-processed images
        cv::Mat preprocessed = preprocess_image(M);
        // Set the scan_height, line_width, min_ratio for obstacle verticle edge detection
        std::vector<int> xcoords = scan_for_vertical_lines(preprocessed, 170, 6, 0.7);
        draw_image_with_lines(&M, 170, xcoords);
        for (uint16_t j = 0; j < 20 ; j++){   ///LINE 220-221 RESET POSITION OF VERTICAL LINES, COMMENT THEM IF YOU GER ONLY ZEROS.
        	array[j]=0;}
         s = static_cast<int>(xcoords.size());
        for (int k = 0; k < s ; k++){
        	array[k]=xcoords[k];
        }


    //}
    return 0;
}
