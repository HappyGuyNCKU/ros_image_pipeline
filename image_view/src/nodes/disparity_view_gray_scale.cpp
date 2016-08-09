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

#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#ifdef HAVE_GTK
#include <gtk/gtk.h>

// Platform-specific workaround for #3026: image_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that image_view exits.
static void destroy(GtkWidget *widget, gpointer data)
{
  ros::shutdown();
}
#endif

namespace enc = sensor_msgs::image_encodings;

// colormap for disparities, RGB
/*static unsigned char colormap[768] = 
  
  };*/

inline void increment(int* value)
{
  ++(*value);
}

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

// Note: StereoView is NOT nodelet-based, as it synchronizes the three streams.
class StereoView
{
private:
  //image_transport::SubscriberFilter left_sub_, right_sub_;
  //message_filters::Subscriber<DisparityImage> disparity_sub_;
  //typedef ExactTime<Image, Image, DisparityImage> ExactPolicy;
  //typedef ApproximateTime<Image, Image, DisparityImage> ApproximatePolicy;
  //typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  //typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  //boost::shared_ptr<ExactSync> exact_sync_;
  //boost::shared_ptr<ApproximateSync> approximate_sync_;
  int queue_size_;
  
  //ImageConstPtr last_left_msg_, last_right_msg_;
  //cv::Mat last_left_image_, last_right_image_;
  cv::Mat_<uint8_t>      disparity_gray_;
  //cv::Mat_<cv::Vec3b> disparity_color_;
  boost::mutex image_mutex_;
  
  boost::format filename_format_;
  int save_count_;

  ros::WallTimer check_synced_timer_;
  int /*left_received_, right_received_, */disp_received_, all_received_;

public:
  StereoView(const std::string& transport)
    : filename_format_(""), save_count_(0)/*,
      left_received_(0), right_received_(0)*/, disp_received_(0), all_received_(0)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    bool autosize;
    local_nh.param("autosize", autosize, true);
    
    std::string format_string;
    local_nh.param("filename_format", format_string, std::string("%s%04i.jpg"));
    filename_format_.parse(format_string);

    // Do GUI window setup
    int flags = autosize ? cv::WND_PROP_AUTOSIZE : 0;
    //cv::namedWindow("left", flags);
    //cv::namedWindow("right", flags);
    cv::namedWindow("disparity", flags);
    //cv::setMouseCallback("left",      &StereoView::mouseCb, this);
    //cv::setMouseCallback("right",     &StereoView::mouseCb, this);
    cv::setMouseCallback("disparity", &StereoView::mouseCb, this);
#if CV_MAJOR_VERSION == 2
#ifdef HAVE_GTK
    //g_signal_connect(GTK_WIDGET( cvGetWindowHandle("left") ),
                     "destroy", G_CALLBACK(destroy), NULL);
    //g_signal_connect(GTK_WIDGET( cvGetWindowHandle("right") ),
                     "destroy", G_CALLBACK(destroy), NULL);
    g_signal_connect(GTK_WIDGET( cvGetWindowHandle("disparity") ),
                     "destroy", G_CALLBACK(destroy), NULL);
#endif
    cvStartWindowThread();
#endif

    // Resolve topic names
    ros::NodeHandle nh;
    std::string stereo_ns = nh.resolveName("stereo");
    //std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    //std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image"));
    std::string disparity_topic = ros::names::clean(stereo_ns + "/disparity");
    //ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s", left_topic.c_str(), right_topic.c_str(),disparity_topic.c_str());
    ROS_INFO("Subscribing to: %s\n\t* %s",disparity_topic.c_str());

    ros::Subscriber disparity_sub_ = nh.subscribe(disparity_topic, 1, &StereoView::imageCb,NULL,NULL);
    // Subscribe to three input topics.
    //image_transport::ImageTransport it(nh);
    //left_sub_.subscribe(it, left_topic, 1, transport);
    //right_sub_.subscribe(it, right_topic, 1, transport);
    //disparity_sub_.subscribe(nh, disparity_topic, 1);

    // Complain every 30s if the topics appear unsynchronized
    //left_sub_.registerCallback(boost::bind(increment, &left_received_));
    //right_sub_.registerCallback(boost::bind(increment, &right_received_));
    //disparity_sub_.registerCallback(boost::bind(increment, &disp_received_));
    //check_synced_timer_ = nh.createWallTimer(ros::WallDuration(15.0),
                                             //boost::bind(&StereoView::checkInputsSynchronized, this));

    // Synchronize input topics. Optionally do approximate synchronization.
    /*local_nh.param("queue_size", queue_size_, 5);
    bool approx;
    local_nh.param("approximate_sync", approx, false);
    if (approx)
    {
      approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size_),/*left_sub_, right_sub_,*/ /*disparity_sub_) );
      approximate_sync_->registerCallback(boost::bind(&StereoView::imageCb, this, _1/*, _2, _3*//*));
    }
    else
    {
      exact_sync_.reset( new ExactSync(ExactPolicy(queue_size_),
                                       /*left_sub_, right_sub_,*/ /*disparity_sub_) );
      exact_sync_->registerCallback(boost::bind(&StereoView::imageCb, this, _1/*, _2, _3*//*));
    }*/
  }

  ~StereoView()
  {
    cv::destroyAllWindows();
  }

  void imageCb(/*const ImageConstPtr& left, const ImageConstPtr& right,*/
               const DisparityImageConstPtr& disparity_msg)
  {
    ++all_received_; // For error checking
    
    image_mutex_.lock();

    // May want to view raw bayer data
    /*if (left->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<Image>(left)->encoding = "mono8";
    if (right->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<Image>(right)->encoding = "mono8";
*/
    // Hang on to image data for sake of mouseCb
    /*last_left_msg_ = left;
    last_right_msg_ = right;
    try {
      last_left_image_ = cv_bridge::toCvShare(left, "bgr8")->image;
      last_right_image_ = cv_bridge::toCvShare(right, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("Unable to convert one of '%s' or '%s' to 'bgr8'",
                left->encoding.c_str(), right->encoding.c_str());
    }*/

    // Colormap and display the disparity image
    float min_disparity = disparity_msg->min_disparity;
    float max_disparity = disparity_msg->max_disparity;
    float multiplier = 255.0f / (max_disparity - min_disparity);

    assert(disparity_msg->image.encoding == enc::TYPE_32FC1);
    const cv::Mat_<float> dmat(disparity_msg->image.height, disparity_msg->image.width,
                               (float*)&disparity_msg->image.data[0], disparity_msg->image.step);
    //disparity_color_.create(disparity_msg->image.height, disparity_msg->image.width);
	//cv::Mat_<char> disparity_gray(disparity_msg->image.height, disparity_msg->image.width);    
    disparity_gray_.create(disparity_msg->image.height, disparity_msg->image.width);
    for (int row = 0; row < disparity_gray_.rows; ++row) {
      const float* d = dmat[row];
      for (int col = 0; col < disparity_gray_.cols; ++col) {
	
        int index = (d[col] - min_disparity) * multiplier + 0.5;
        index = std::min(255, std::max(0, index));
	if(index<0)
	  ROS_INFO("disparity value :%d \n", index);
        // Fill as BGR
        //disparity_color_(row, col)[2] = colormap[3*index + 0];
        //disparity_color_(row, col)[1] = colormap[3*index + 1];
        //disparity_color_(row, col)[0] = colormap[3*index + 2];
	disparity_gray_(row,col) = index;
      }
    }

    // Must release the mutex before calling cv::imshow, or can deadlock against
    // OpenCV's window mutex.
    image_mutex_.unlock();
    /*if (!last_left_image_.empty())
      cv::imshow("left", last_left_image_);
    if (!last_right_image_.empty())
      cv::imshow("right", last_right_image_);*/
    //cv::imshow("disparity", disparity_color_);
	cv::imshow("disparity", disparity_gray_);
  }

  void saveImage(const char* prefix, const cv::Mat& image)
  {
    if (!image.empty()) {
      std::string filename = (filename_format_ % prefix % save_count_).str();
      cv::imwrite(filename, image);
      ROS_INFO("Saved image %s", filename.c_str());
    } else {
      ROS_WARN("Couldn't save %s image, no data!", prefix);
    }
  }
  
  static void mouseCb(int event, int x, int y, int flags, void* param)
  {
    if (event == cv::EVENT_LBUTTONDOWN)
    {
      ROS_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
	  StereoView *sv = (StereoView*)param;
	  boost::lock_guard<boost::mutex> guard(sv->image_mutex_);
	  ROS_INFO("X:%d ,Y:%d ,value:%u", x,y ,sv->disparity_gray_(y,x));
      return;
    }
    if (event != cv::EVENT_RBUTTONDOWN)
      return;
    
    StereoView *sv = (StereoView*)param;
    boost::lock_guard<boost::mutex> guard(sv->image_mutex_);

    //sv->saveImage("left",  sv->last_left_image_);
    //sv->saveImage("right", sv->last_right_image_);
    sv->saveImage("disp",  sv->disparity_gray_);
    sv->save_count_++;
  }

  void checkInputsSynchronized()
  {
    int threshold = 3 * all_received_;
    /*if (left_received_ >= threshold || right_received_ >= threshold || disp_received_ >= threshold) {
      ROS_WARN("[stereo_view] Low number of synchronized left/right/disparity triplets received.\n"
               "Left images received:      %d (topic '%s')\n"
               "Right images received:     %d (topic '%s')\n"
               "Disparity images received: %d (topic '%s')\n"
               "Synchronized triplets: %d\n"
               "Possible issues:\n"
               "\t* stereo_image_proc is not running.\n"
               "\t  Does `rosnode info %s` show any connections?\n"
               "\t* The cameras are not synchronized.\n"
               "\t  Try restarting stereo_view with parameter _approximate_sync:=True\n"
               "\t* The network is too slow. One or more images are dropped from each triplet.\n"
               "\t  Try restarting stereo_view, increasing parameter 'queue_size' (currently %d)",
               left_received_, left_sub_.getTopic().c_str(),
               right_received_, right_sub_.getTopic().c_str(),
               disp_received_, disparity_sub_.getTopic().c_str(),
               all_received_, ros::this_node::getName().c_str(), queue_size_);
    }*/
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_view", ros::init_options::AnonymousName);
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun image_view stereo_view stereo:=narrow_stereo image:=image_color");
  }
  /*if (ros::names::remap("image") == "/image_raw") {
    ROS_WARN("There is a delay between when the camera drivers publish the raw images and "
             "when stereo_image_proc publishes the computed point cloud. stereo_view "
             "may fail to synchronize these topics without a large queue_size.");
  }*/

  std::string transport = argc > 1 ? argv[1] : "raw";
  StereoView view(transport);
  
  ros::spin();
  return 0;
}
