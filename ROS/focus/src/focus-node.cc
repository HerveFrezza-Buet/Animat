#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <cv.h>
#include <highgui.h>
#include <dynamic_reconfigure/server.h>

#include "neuralfield.hpp"
#include <focus/ParamConfig.h>

void on_reconf(focus::ParamConfig &config, uint32_t level, 
	       focus::NeuralField & nf,
	       double& shift_freeze_duration,
	       double& max_small_shift_radius_2) {
  /*
  if(level == 0) {
    param.reference = config.reference;
    param.tolerance = config.tolerance;
    param.dontcare = config.dontcare;
  }
  */

  switch(level) {
  case 0:
    shift_freeze_duration = config.shift_freeze;
    max_small_shift_radius_2 = config.shift_rad*config.shift_rad;
  case 1:
    nf.updateDTauu(config.dtau_u);
    nf.updateDTauv(config.dtau_u);
    nf.updateBeta(config.beta);
    break;
  case 2:
    nf.updateSg(config.sg);
    nf.updateLatWeights(config.Ap, config.sp, config.Am, config.sm);
    break;
  default:
    break;
  }
}

void on_image(const sensor_msgs::Image::ConstPtr& msg,
	      image_transport::Publisher& nf_out_pub,
	      focus::NeuralField& nf,
	      ros::Publisher& shift_pub,
	      ros::Time& last_shift,
	      double& shift_freeze_duration,
	      double& max_small_shift_radius_2) {
  cv_bridge::CvImageConstPtr bridge_input;
  try {
    bridge_input = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::MONO8);
  }
  catch (cv::Exception& e) {
    std::ostringstream errstr;
    errstr << "cv_bridge exception caught: " << e.what();
    return;
  }
  const cv::Mat& input   = bridge_input->image;
  int nf_width, nf_height;
  nf.getSize(nf_width, nf_height);
  
  cv::Mat nf_input(nf_width, nf_height, CV_8UC1);
  cv::resize(input, nf_input, nf_input.size(), 0, 0);

  // Feed the neural field
  nf.new_input(nf_input);

  // And iterate
  for(unsigned int i = 0 ; i < 30 ; ++i) 
    nf.step();
  
  // Prepare the output
  if(nf_out_pub.getNumSubscribers() > 0) {
    cv::Mat frame;
    nf.fill_output(frame);
    nf_out_pub.publish(cv_bridge::CvImage(msg->header, "mono8", frame).toImageMsg());
  }

  // Compute the center of mass of the output activities
  // mind the center of mass is normalized  by the shape of the neuralfield
  // x : horizontal in [0, 1]
  // y : vertical in [0, 1]
  // (x, y) = (-1, -1) in case no decision is taken, e.g. black input
  double x, y;
  nf.get_com(x, y);
  ros::Time current = ros::Time::now();
  if( x > 0 && y > 0 && (current - last_shift).toSec() > shift_freeze_duration) {
    geometry_msgs::Point msg;
    msg.x = x - 0.5;
    msg.y = y - 0.5;
    msg.z = 0.0;
    shift_pub.publish(msg);
    
    if(msg.x*msg.x + msg.y*msg.y > max_small_shift_radius_2)
      last_shift = current;
  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "focus_node");
  ros::NodeHandle n;
  ros::Time       last_shift = ros::Time::now();

  ////////// Parameters of the neural field
  focus::NeuralField::Parameters nf_params;
  nf_params.width = 30;
  nf_params.height = 30;
  nf_params.dtau_u = 0.27;
  nf_params.dtau_v = 0.27;
  nf_params.beta = 1.0;
  nf_params.noise = 0.0;
  nf_params.sg = 5.0;
  nf_params.Ap = 1.0;
  nf_params.sp = 5.0;
  nf_params.Am = 1.0;
  nf_params.sm = 30.0;
  nf_params.circular_symmetric = false;
  double shift_freeze_duration = .2;
  double max_small_shift_radius_2 = .2*.2;
  focus::NeuralField nf(nf_params);

  

  ////////// Reconfigure the node
  dynamic_reconfigure::Server<focus::ParamConfig> server;
  server.setCallback(boost::bind(&on_reconf, _1, _2,
				 boost::ref(nf),
				 boost::ref(shift_freeze_duration),
				 boost::ref(max_small_shift_radius_2)));

  ////////// Publisher for shifting the focus
  ros::Publisher shift_pub = n.advertise<geometry_msgs::Point>("shift", 1);

  ////////// Subscribers and publishers for the input image, and some debugging outputs (input to the neural field and decision output)
  image_transport::ImageTransport it(n);
  image_transport::Publisher      image_nf_out_pub = it.advertise("out", 1);
  image_transport::Subscriber     image_sub = it.subscribe("in", 1, 
							   boost::bind(on_image, _1,
								       image_nf_out_pub, 
								       boost::ref(nf),
								       boost::ref(shift_pub),
								       boost::ref(last_shift),
								       boost::ref(shift_freeze_duration),
								       boost::ref(max_small_shift_radius_2)));
  ros::spin();
}
