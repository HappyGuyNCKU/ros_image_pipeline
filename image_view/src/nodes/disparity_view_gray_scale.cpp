/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <image_view/ImageViewConfig.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

// For output data 
#include <iostream>                                                             
#include <fstream>


using namespace sensor_msgs;
using namespace stereo_msgs;

int g_count;
cv::Mat g_last_image;
boost::format g_filename_format;
boost::mutex g_image_mutex;
std::string g_window_name;
bool g_do_dynamic_scaling;
int g_colormap;
cv::Mat_<float> dmat;

static unsigned char colormap[768] = 
  { 150, 150, 150,
    107, 0, 12,
    106, 0, 18,
    105, 0, 24,
    103, 0, 30,
    102, 0, 36,
    101, 0, 42,
    99, 0, 48,
    98, 0, 54,
    97, 0, 60,
    96, 0, 66,
    94, 0, 72,
    93, 0, 78,
    92, 0, 84,
    91, 0, 90,
    89, 0, 96,
    88, 0, 102,
    87, 0, 108,
    85, 0, 114,
    84, 0, 120,
    83, 0, 126,
    82, 0, 131,
    80, 0, 137,
    79, 0, 143,
    78, 0, 149,
    77, 0, 155,
    75, 0, 161,
    74, 0, 167,
    73, 0, 173,
    71, 0, 179,
    70, 0, 185,
    69, 0, 191,
    68, 0, 197,
    66, 0, 203,
    65, 0, 209,
    64, 0, 215,
    62, 0, 221,
    61, 0, 227,
    60, 0, 233,
    59, 0, 239,
    57, 0, 245,
    56, 0, 251,
    55, 0, 255,
    54, 0, 255,
    52, 0, 255,
    51, 0, 255,
    50, 0, 255,
    48, 0, 255,
    47, 0, 255,
    46, 0, 255,
    45, 0, 255,
    43, 0, 255,
    42, 0, 255,
    41, 0, 255,
    40, 0, 255,
    38, 0, 255,
    37, 0, 255,
    36, 0, 255,
    34, 0, 255,
    33, 0, 255,
    32, 0, 255,
    31, 0, 255,
    29, 0, 255,
    28, 0, 255,
    27, 0, 255,
    26, 0, 255,
    24, 0, 255,
    23, 0, 255,
    22, 0, 255,
    20, 0, 255,
    19, 0, 255,
    18, 0, 255,
    17, 0, 255,
    15, 0, 255,
    14, 0, 255,
    13, 0, 255,
    11, 0, 255,
    10, 0, 255,
    9, 0, 255,
    8, 0, 255,
    6, 0, 255,
    5, 0, 255,
    4, 0, 255,
    3, 0, 255,
    1, 0, 255,
    0, 4, 255,
    0, 10, 255,
    0, 16, 255,
    0, 22, 255,
    0, 28, 255,
    0, 34, 255,
    0, 40, 255,
    0, 46, 255,
    0, 52, 255,
    0, 58, 255,
    0, 64, 255,
    0, 70, 255,
    0, 76, 255,
    0, 82, 255,
    0, 88, 255,
    0, 94, 255,
    0, 100, 255,
    0, 106, 255,
    0, 112, 255,
    0, 118, 255,
    0, 124, 255,
    0, 129, 255,
    0, 135, 255,
    0, 141, 255,
    0, 147, 255,
    0, 153, 255,
    0, 159, 255,
    0, 165, 255,
    0, 171, 255,
    0, 177, 255,
    0, 183, 255,
    0, 189, 255,
    0, 195, 255,
    0, 201, 255,
    0, 207, 255,
    0, 213, 255,
    0, 219, 255,
    0, 225, 255,
    0, 231, 255,
    0, 237, 255,
    0, 243, 255,
    0, 249, 255,
    0, 255, 255,
    0, 255, 249,
    0, 255, 243,
    0, 255, 237,
    0, 255, 231,
    0, 255, 225,
    0, 255, 219,
    0, 255, 213,
    0, 255, 207,
    0, 255, 201,
    0, 255, 195,
    0, 255, 189,
    0, 255, 183,
    0, 255, 177,
    0, 255, 171,
    0, 255, 165,
    0, 255, 159,
    0, 255, 153,
    0, 255, 147,
    0, 255, 141,
    0, 255, 135,
    0, 255, 129,
    0, 255, 124,
    0, 255, 118,
    0, 255, 112,
    0, 255, 106,
    0, 255, 100,
    0, 255, 94,
    0, 255, 88,
    0, 255, 82,
    0, 255, 76,
    0, 255, 70,
    0, 255, 64,
    0, 255, 58,
    0, 255, 52,
    0, 255, 46,
    0, 255, 40,
    0, 255, 34,
    0, 255, 28,
    0, 255, 22,
    0, 255, 16,
    0, 255, 10,
    0, 255, 4,
    2, 255, 0,
    8, 255, 0,
    14, 255, 0,
    20, 255, 0,
    26, 255, 0,
    32, 255, 0,
    38, 255, 0,
    44, 255, 0,
    50, 255, 0,
    56, 255, 0,
    62, 255, 0,
    68, 255, 0,
    74, 255, 0,
    80, 255, 0,
    86, 255, 0,
    92, 255, 0,
    98, 255, 0,
    104, 255, 0,
    110, 255, 0,
    116, 255, 0,
    122, 255, 0,
    128, 255, 0,
    133, 255, 0,
    139, 255, 0,
    145, 255, 0,
    151, 255, 0,
    157, 255, 0,
    163, 255, 0,
    169, 255, 0,
    175, 255, 0,
    181, 255, 0,
    187, 255, 0,
    193, 255, 0,
    199, 255, 0,
    205, 255, 0,
    211, 255, 0,
    217, 255, 0,
    223, 255, 0,
    229, 255, 0,
    235, 255, 0,
    241, 255, 0,
    247, 255, 0,
    253, 255, 0,
    255, 251, 0,
    255, 245, 0,
    255, 239, 0,
    255, 233, 0,
    255, 227, 0,
    255, 221, 0,
    255, 215, 0,
    255, 209, 0,
    255, 203, 0,
    255, 197, 0,
    255, 191, 0,
    255, 185, 0,
    255, 179, 0,
    255, 173, 0,
    255, 167, 0,
    255, 161, 0,
    255, 155, 0,
    255, 149, 0,
    255, 143, 0,
    255, 137, 0,
    255, 131, 0,
    255, 126, 0,
    255, 120, 0,
    255, 114, 0,
    255, 108, 0,
    255, 102, 0,
    255, 96, 0,
    255, 90, 0,
    255, 84, 0,
    255, 78, 0,
    255, 72, 0,
    255, 66, 0,
    255, 60, 0,
    255, 54, 0,
    255, 48, 0,
    255, 42, 0,
    255, 36, 0,
    255, 30, 0,
    255, 24, 0,
    255, 18, 0,
    255, 12, 0,
    255,  6, 0,
    255,  0, 0,
  };

void reconfigureCb(image_view::ImageViewConfig &config, uint32_t level)
{

  boost::mutex::scoped_lock lock(g_image_mutex);
  g_do_dynamic_scaling = config.do_dynamic_scaling;
  g_colormap = config.colormap;
}

void imageCb(const DisparityImageConstPtr& disparity_msg)//const sensor_msgs::ImageConstPtr& msg)
{
	boost::mutex::scoped_lock lock(g_image_mutex);

  // Convert to OpenCV native BGR color
/*  try {
    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = g_do_dynamic_scaling;
    options.colormap = g_colormap;
    g_last_image = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(msg), "",
                                                 options)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'",
                       msg->encoding.c_str(), e.what());
  }
*/
// Colormap and display the disparity image
    float min_disparity = disparity_msg->min_disparity;
    float max_disparity = disparity_msg->max_disparity;

	std::ofstream out;
	out.open( "/tmp/text.txt",std::ios::app );
	if( !out )
	{
		out.open( "/tmp/text.txt");
		if(!out)
		{
			ROS_INFO("Couldn't open file.");
		}
	}
	else
	{
		out << "min: " << min_disparity  << ", max: " << max_disparity  <<"\n";
		out.close();		
	}	

    float multiplier = 255.0f / (max_disparity - min_disparity);

	//assert(disparity_msg->image.encoding == enc::TYPE_32FC1);
    dmat = cv::Mat_<float> (disparity_msg->image.height, disparity_msg->image.width, (float*)&disparity_msg->image.data[0], disparity_msg->image.step);
	//cv::Mat_<char> disparity_gray(disparity_msg->image.height, disparity_msg->image.width);   
	cv::Mat_<cv::Vec3b> disparity_color_;
	disparity_color_.create(disparity_msg->image.height, disparity_msg->image.width); 
    cv::Mat_<char>  disparity_gray_;
    disparity_gray_.create(disparity_msg->image.height, disparity_msg->image.width);
	for (int row = 0; row < disparity_gray_.rows; ++row) {
      const float* d = dmat[row];
      for (int col = 0; col < disparity_gray_.cols; ++col) {
        int index = (d[col] - min_disparity) * multiplier + 0.5;
        index = std::min(255, std::max(0, index));
        // Fill as BGR
		disparity_color_(row, col)[2] = colormap[3*index + 0];
        disparity_color_(row, col)[1] = colormap[3*index + 1];
        disparity_color_(row, col)[0] = colormap[3*index + 2];
		disparity_gray_(row, col) = index;
      }
    }




  /*if (!g_last_image.empty())*/ {
    /*const*/ cv::Mat &image = disparity_color_;

    //std::string imageName("/home/jason/logo_2.png");
     //  cv::Mat image;   
    //image = imread(imageName.c_str(), cv::IMREAD_COLOR);
    cv::imshow(g_window_name, image);
    cv::waitKey(10);
  }
}

static void mouseCb(int event, int x, int y, int flags, void* param)
{
  if (event == cv::EVENT_LBUTTONDOWN) {
    ROS_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
    ROS_INFO_ONCE("Left-clicking is used to get pixel disparity value");
	ROS_INFO("X:%d  Y:%d ,disparity:%f",x,y,dmat(y,x));
	return;
  } else if (event != cv::EVENT_RBUTTONDOWN) {
    return;
  }

  boost::mutex::scoped_lock lock(g_image_mutex);

  const cv::Mat &image = g_last_image;

  if (image.empty()) {
    ROS_WARN("Couldn't save image, no data!");
    return;
  }

  std::string filename = (g_filename_format % g_count).str();
  if (cv::imwrite(filename, image)) {
    ROS_INFO("Saved image %s", filename.c_str());
    g_count++;
  } else {
    boost::filesystem::path full_path = boost::filesystem::complete(filename);
    ROS_ERROR_STREAM("Failed to save image. Have permission to write there?: " << full_path);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_view", ros::init_options::AnonymousName);
  if (ros::names::remap("image") == "image") {
    ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
             "\t$ rosrun image_view image_view image:=<image topic> [transport]");
  }

  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  // Default window name is the resolved topic name
  std::string topic = nh.resolveName("image");
  local_nh.param("window_name", g_window_name, topic);

  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
  g_filename_format.parse(format_string);
  // Handle window size
  bool autosize;
  local_nh.param("autosize", autosize, false);
  cv::namedWindow(g_window_name, autosize ? CV_WINDOW_AUTOSIZE  : 0);
  cv::setMouseCallback(g_window_name, &mouseCb);

  // Start the OpenCV window thread so we don't have to waitKey() somewhere
  cv::startWindowThread();

  // Handle transport
  // priority:
  //    1. command line argument
  //    2. rosparam '~image_transport'
  std::string transport;
  local_nh.param("image_transport", transport, std::string("raw"));
  ros::V_string myargv;
  ros::removeROSArgs(argc, argv, myargv);
  for (size_t i = 1; i < myargv.size(); ++i) {
    if (myargv[i][0] != '-') {
      transport = myargv[i];
      break;
    }
  }
//  message_filters::Subscriber<DisparityImage> disparity_sub_;
  
  ROS_INFO_STREAM("Using transport \"" << transport << "\"");
  image_transport::ImageTransport it(nh);  
//  image_transport::TransportHints hints(transport, ros::TransportHints(), local_nh);
  //sub = it.subscribe(topic, 1, imageCb, hints);
  //image_transport::Subscriber sub = it.subscribe(topic, 1, imageCb, hints);
  ros::Subscriber sub = nh.subscribe(topic, 1, imageCb);
  
  dynamic_reconfigure::Server<image_view::ImageViewConfig> srv;
  dynamic_reconfigure::Server<image_view::ImageViewConfig>::CallbackType f =
    boost::bind(&reconfigureCb, _1, _2);
  srv.setCallback(f);

  ros::spin();

  cv::destroyWindow(g_window_name);
  return 0;
}

