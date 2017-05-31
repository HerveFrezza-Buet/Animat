// This node takes a colored input image
// as well as Hue, Saturation, Value   parameters
// and outputs the grayscale filtered image on the specified color
// or just the pixels with a value and saturation above the speciified minimum
// values if you specifiy in the HSVParmas that you do not care about the hue

// In opencv , Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255].

// Subscribe to /in
// Publish to /filtered

#include <ros/ros.h> 
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <color_filter/ParamConfig.h>
#include <color_filter/HSVParams.h>


struct HSVParams {
  int hue;
  int hue_tol;
  int min_value;
  int min_saturation;
  bool dontcare;
};


void param_callback(HSVParams& params, const color_filter::HSVParamsConstPtr& msg) {
  params.hue = msg->hue;
  params.hue_tol = msg->hue_tol;
  params.min_value = msg->min_value;
  params.min_saturation = msg->min_saturation;
  params.dontcare = msg->dontcare;
}


// Circular symmetric distance
int distance_hue(int hue1, int hue2) {
  return std::min(abs(hue1-hue2), 180 - abs(hue1-hue2));
}

void imageCallback(HSVParams& params,
		   image_transport::Publisher& pub_img, 
		   const sensor_msgs::ImageConstPtr& in_msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(in_msg, in_msg->encoding);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat& src = cv_ptr->image;
    cv::Mat hsv;
    cv::Mat filter(src.size(), CV_8UC1);

    int conversion_mode = 0;
    if(in_msg->encoding == "rgb8") 
      conversion_mode = CV_RGB2HSV;
    else if(in_msg->encoding == "bgr8")
      conversion_mode = CV_BGR2HSV;
    else {
      ROS_ERROR("Unknown conversion with image encoding %s ", in_msg->encoding.c_str());
      return;
    }

    cv::cvtColor(src, hsv, conversion_mode);
    
    for(int i = 0; i < hsv.rows; i++) {
      for(int j = 0; j < hsv.cols; j++) {
	cv::Vec3b hsv_pix=hsv.at<cv::Vec3b>(i,j);
	int H=hsv_pix.val[0]; //hue
	int S=hsv_pix.val[1]; //saturation
	int V=hsv_pix.val[2]; //value
	if(V >= params.min_value && 
	   S >= params.min_saturation) {
	  if(params.dontcare) 
	    filter.at<unsigned char>(i, j) = 255;
	  else {
	  int dhue = distance_hue(H, params.hue);
	    if(dhue >= params.hue_tol) 
	      filter.at<unsigned char>(i, j) = 0;
	    else
	      filter.at<unsigned char>(i, j) = (int)(255 * (params.hue_tol - dhue)/params.hue_tol);
	  }
	}
	else 
	  filter.at<unsigned char>(i, j) = 0;	
      }
    }
  
    sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(in_msg->header, sensor_msgs::image_encodings::MONO8, filter).toImageMsg();
    pub_img.publish(out_img);
}
void on_reconf(color_filter::ParamConfig &config, uint32_t level, HSVParams& param) {
	param.hue = config.hue;
	param.hue_tol = config.hue_tol;
	param.min_value = config.min_value;
	param.min_saturation = config.min_saturation;
	param.dontcare = config.dontcare; 

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_filter");
  ros::NodeHandle nh;

  HSVParams filter_params;
  filter_params.hue = 120;
  filter_params.hue_tol = 20;
  filter_params.min_value = 50;
  filter_params.min_saturation = 50;
  filter_params.dontcare = false;
  
  dynamic_reconfigure::Server<color_filter::ParamConfig> server;
  server.setCallback(boost::bind(&on_reconf, _1, _2, boost::ref(filter_params)));
  
  ros::Subscriber sub_params = nh.subscribe<color_filter::HSVParams>("params", 1, boost::bind(param_callback, boost::ref(filter_params), _1));

  image_transport::ImageTransport it(nh);
  // The publisher of the result
  image_transport::Publisher pub_img = it.advertise("filtered", 1);

  // The subscriber for the original image
  image_transport::Subscriber sub = it.subscribe("in", 1, boost::bind(imageCallback, 
								      boost::ref(filter_params), 
								      boost::ref(pub_img), _1));

  ros::spin();

}
