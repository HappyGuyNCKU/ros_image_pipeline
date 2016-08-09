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
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <opencv2/highgui/highgui.hpp>
#include "window_thread.h"
#include <boost/thread.hpp>
#include <boost/format.hpp>


#ifdef HAVE_GTK
#include <gtk/gtk.h>

// Platform-specific workaround for #3026: image_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that image_view exits.
static void destroyNode(GtkWidget *widget, gpointer data)
{
  exit(0);
}

static void destroyNodelet(GtkWidget *widget, gpointer data)
{
  // We can't actually unload the nodelet from here, but we can at least
  // unsubscribe from the image topic.
  reinterpret_cast<ros::Subscriber*>(data)->shutdown();
}
#endif


namespace image_view {

class DisparityGrayScaleNodelet : public nodelet::Nodelet
{
  // colormap for disparities, RGB order
  static unsigned char colormap[];

  std::string window_name_;
  ros::Subscriber sub_;
  cv::Mat_<uint8_t> disparity_gray_;
  //boost::format filename_format_;
  boost::mutex image_mutex_;
  virtual void onInit();
  
  void imageCb(const stereo_msgs::DisparityImageConstPtr& msg);


public:
  ~DisparityGrayScaleNodelet();
  void saveImage(const char* prefix, const cv::Mat& image);
  static void mouseCb(int event, int x, int y, int flags, void* param);
};

DisparityGrayScaleNodelet::~DisparityGrayScaleNodelet()
{
  cv::destroyWindow(window_name_);
}

void DisparityGrayScaleNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle local_nh = getPrivateNodeHandle();
  const std::vector<std::string>& argv = getMyArgv();

  // Internal option, should be used only by image_view nodes
  bool shutdown_on_close = std::find(argv.begin(), argv.end(),
                                     "--shutdown-on-close") != argv.end();
  ROS_INFO("gray_scale");
  // Default window name is the resolved topic name
  std::string topic = nh.resolveName("image");
  local_nh.param("window_name", window_name_, topic);

  bool autosize;
  local_nh.param("autosize", autosize, false);

  cv::namedWindow(window_name_, autosize ? cv::WND_PROP_AUTOSIZE : 0);
  cv::setMouseCallback(window_name_, &DisparityGrayScaleNodelet::mouseCb, this);
#if CV_MAJOR_VERSION ==2
#ifdef HAVE_GTK
  // Register appropriate handler for when user closes the display window
  GtkWidget *widget = GTK_WIDGET( cvGetWindowHandle(window_name_.c_str()) );
  if (shutdown_on_close)
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNode), NULL);
  else
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNodelet), &sub_);
#endif
#endif

  // Start the OpenCV window thread so we don't have to waitKey() somewhere
  startWindowThread();

  sub_ = nh.subscribe<stereo_msgs::DisparityImage>(topic, 1, &DisparityGrayScaleNodelet::imageCb, this);
}

void DisparityGrayScaleNodelet::imageCb(const stereo_msgs::DisparityImageConstPtr& msg)
{
  // Check for common errors in input
  if (msg->min_disparity == 0.0 && msg->max_disparity == 0.0)
  {
    NODELET_ERROR_THROTTLE(30, "Disparity image fields min_disparity and "
                           "max_disparity are not set");
    return;
  }
  if (msg->image.encoding != sensor_msgs::image_encodings::TYPE_32FC1)
  {
    NODELET_ERROR_THROTTLE(30, "Disparity image must be 32-bit floating point "
                           "(encoding '32FC1'), but has encoding '%s'",
                           msg->image.encoding.c_str());
    return;
  }
  
  // Colormap and display the disparity image
  float min_disparity = msg->min_disparity;
  float max_disparity = msg->max_disparity;
  float multiplier = 255.0f / (max_disparity - min_disparity);

  const cv::Mat_<float> dmat(msg->image.height, msg->image.width,
                             (float*)&msg->image.data[0], msg->image.step);
  
  disparity_gray_.create(msg->image.height, msg->image.width);
    
  for (int row = 0; row < disparity_gray_.rows; ++row) {
    const float* d = dmat[row];
    uint8_t *disparity_gray = disparity_gray_[row],*disparity_gray_end = disparity_gray + disparity_gray_.cols;

    for (; disparity_gray < disparity_gray_end; ++disparity_gray, ++d) {
      int index = (*d - min_disparity) * multiplier + 0.5;
      index = std::min(255, std::max(0, index));

      *disparity_gray = index;
      
    }
  }

  /// @todo For Electric, consider option to draw outline of valid window
#if 0
  sensor_msgs::RegionOfInterest valid = msg->valid_window;
  cv::Point tl(valid.x_offset, valid.y_offset), br(valid.x_offset + valid.width, valid.y_offset + valid.height);
  cv::rectangle(disparity_color_, tl, br, CV_RGB(255,0,0), 1);
#endif
  
  cv::imshow(window_name_, disparity_gray_);
}

void DisparityGrayScaleNodelet::saveImage(const char* prefix, const cv::Mat& image)
  {
    if (!image.empty()) {
      std::string filename = "disparity_view.jpg";
      cv::imwrite(filename, image);
      ROS_INFO("Saved image %s", filename.c_str());
    } else {
      ROS_WARN("Couldn't save %s image, no data!", prefix);
    }
  }
  
void DisparityGrayScaleNodelet::mouseCb(int event, int x, int y, int flags, void* param)
  {
    if (event == cv::EVENT_LBUTTONDOWN)
    {
      ROS_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
	  DisparityGrayScaleNodelet *sv = (DisparityGrayScaleNodelet*)param;
	  boost::lock_guard<boost::mutex> guard(sv->image_mutex_);
	  ROS_INFO("X:%d ,Y:%d ,value:%u", x,y ,sv->disparity_gray_(y,x));
      return;
    }
    if (event != cv::EVENT_RBUTTONDOWN)
      return;
    
    DisparityGrayScaleNodelet *sv = (DisparityGrayScaleNodelet*)param;
    boost::lock_guard<boost::mutex> guard(sv->image_mutex_);

    sv->saveImage("disp",  sv->disparity_gray_);

  }


} // namespace image_view

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( image_view::DisparityGrayScaleNodelet, nodelet::Nodelet)
