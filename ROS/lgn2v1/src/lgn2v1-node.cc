#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Point.h"
#include <dynamic_reconfigure/server.h>
#include <lgn2v1/ParamConfig.h>


#include "lgn2v1.hpp"

#include <sstream>


class Param {
public:

  double _blur;
  double _r; 
  double _q;  
  double _d;  
  int    _lgnSide;   
  int    _focus_freeze;
  double _alpha;

  double blur()           const {return _blur;}
  double r()              const {return _r;}
  double q()              const {return _q;}
  double d()              const {return _d;}
  int    lgnSide()        const {return _lgnSide;}
  int    focus_freeze()   const {return _focus_freeze;}
  double alpha()          const {return _alpha;}


  Param() 
    : _blur(.005),
      _r(.5), _q(.5), _d(2), 
      _lgnSide(200),
      _focus_freeze(1000),
      _alpha(60) {}
};

typedef lgn2v1::LGN<Param> Lgn;

class Focus {
private:
  ros::Time last;
  int& freeze;
  cv::Point2f focus;

public:
  
  Focus() = delete;
  Focus(const Focus&) = delete;
  Focus& operator=(const Focus&) = delete;
  Focus(int& freeze_duration)
    : last(ros::Time::now()),
      freeze(freeze_duration),
      focus(0,0){}

  cv::Point2f get() {return focus;}
  
  void set(const cv::Point2f& f) {
    focus = f;
    last = ros::Time::now();
  }

  void try_set(const cv::Point2f& f) {
    if((ros::Time::now()-last).toSec() > .001*freeze)
      focus = f;
  }
};


void on_reconf(lgn2v1::ParamConfig &config, uint32_t level, Param& param) {
  param._blur = config.blur;
  param._r = config.r;
  param._q = config.q;
  param._d = config.d;
  param._lgnSide = config.lgnSide;
  param._focus_freeze = config.focus_freeze;
  param._alpha = config.alpha;
}


void on_image(const sensor_msgs::Image::ConstPtr& msg,
	      image_transport::Publisher& pub,
	      image_transport::Publisher& fpub,
	      Lgn& lgn,
	      Focus& focus) {
  if(lgn.param._lgnSide > 0) {
    cv_bridge::CvImageConstPtr bridge_input;
    try {
      bridge_input = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::RGB8);
    }
    catch (cv::Exception& e) {
      std::ostringstream errstr;
      errstr << "cv_bridge exception caught: " << e.what();
      return;
    }
    const cv::Mat& input   = bridge_input->image;

   
    auto foc = focus.get();
    lgn.setFocus(foc.x,foc.y);
    lgn(input);
    pub.publish(cv_bridge::CvImage(msg->header, "rgb8", lgn.lgn).toImageMsg());

    cv::Mat output  = input;
    
    cv::Point2f ifocus((int)((foc.x+.5)*output.cols+.5),
		       (int)(foc.y*output.cols+output.rows/2+.5));
    cv::line(output,cv::Point2i(0, ifocus.y), cv::Point2i(output.cols-1, ifocus.y), cv::Scalar(0,0,0), 3);
    cv::line(output,cv::Point2i(ifocus.x, 0), cv::Point2i(ifocus.x, output.rows-1), cv::Scalar(0,0,0), 3);
    fpub.publish(cv_bridge::CvImage(msg->header, "rgb8", output).toImageMsg());
  }
}

void on_focus(const geometry_msgs::Point::ConstPtr& msg,
	      Focus& focus) {
  focus.set(cv::Point2f(msg->x,msg->y));
}

void on_lgn_focus(const geometry_msgs::Point::ConstPtr& msg,
		  Focus& focus,Lgn& lgn) {
		 
  focus.try_set(lgn.whereIs(cv::Point2f(msg->x,msg->y)));		  
}

void on_lgnconv_focus(const geometry_msgs::Point::ConstPtr& msg, Focus& focus, Lgn& lgn, ros::Publisher& dfocus_pub) {
  geometry_msgs::Point pt_msg;
  auto foc_init = focus.get();
  auto foc_end  = lgn.whereIs(cv::Point2f(msg->x,msg->y));
  pt_msg.x = lgn.param.alpha()*(foc_end.x - foc_init.x);
  pt_msg.y = lgn.param.alpha()*(foc_end.y - foc_init.y);
  dfocus_pub.publish(pt_msg);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "lgn2v1_node");
  ros::NodeHandle n;
  cv::Mat result;
  Param param;
  Lgn lgn(param,result);
  ros::Rate loop_rate(100);
  Focus focus(param._focus_freeze);

  dynamic_reconfigure::Server<lgn2v1::ParamConfig> server;
  server.setCallback(boost::bind(&on_reconf, _1, _2, boost::ref(param)));

  image_transport::ImageTransport it(n);
  image_transport::Publisher      image_pub = it.advertise("lgn", 1);
  image_transport::Publisher      gaze_pub  = it.advertise("gaze", 1);
  image_transport::Subscriber     image_sub = it.subscribe("in", 1, 
							   boost::bind(on_image, _1,
								       image_pub, gaze_pub,
								       boost::ref(lgn), boost::ref(focus)));


  ros::Subscriber                 focus_sub         = n.subscribe<geometry_msgs::Point>("lookat", 1, boost::bind(on_focus,_1,boost::ref(focus)));
  ros::Subscriber                 focus_lgn_sub     = n.subscribe<geometry_msgs::Point>("shift", 1, boost::bind(on_lgn_focus,_1,boost::ref(focus),boost::ref(lgn)));
  ros::Publisher                  focus_pub         = n.advertise<geometry_msgs::Point>("focus", 1);
  ros::Publisher                  dfocus_pub        = n.advertise<geometry_msgs::Point>("dfocus", 1);
  ros::Subscriber                 focus_lgnconv_sub = n.subscribe<geometry_msgs::Point>("convert", 1, boost::bind(on_lgnconv_focus,_1,boost::ref(focus),boost::ref(lgn), boost::ref(dfocus_pub)));

  while (ros::ok()) {
    geometry_msgs::Point pt_msg;
    auto foc = focus.get();
    pt_msg.x = foc.x;
    pt_msg.y = foc.y;
    focus_pub.publish(pt_msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
