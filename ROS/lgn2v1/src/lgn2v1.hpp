
#include <cmath>
#include <opencv/cv.h>

namespace lgn2v1 {

  namespace default_param {
    class LGN {
    public:
      double blur()           const {return   .005;}
      double r()              const {return  .2;}
      double q()              const {return  .4;}
      double d()              const {return   3;}
      int    lgnSide()        const {return 200;}
    };
  }

  struct MapInfo {
    int lgnSide;
    int width;
    int height;
    double w;
    double h;
    double r;
    double q;
    double d;
    bool changed;

    MapInfo() = default;

    MapInfo(int lgnSide, 
	    int width, int height,
	    double w, double h,
	    double r, double q, double d) 
      : lgnSide(lgnSide),
	width(width), height(height),
	w(w), h(h),
	r(r), q(q), d(d),
	changed(false) {}

    MapInfo& operator=(const MapInfo& info) {
      changed = false;
      if(info.lgnSide != lgnSide) {
	lgnSide = info.lgnSide;
	changed = true;
      }
      if(info.width != width) {
	width = info.width;
	changed = true;
      }
      if(info.height != height) {
	height = info.height;
	changed = true;
      }
      if(info.w != w) {
	w = info.w;
	changed = true;
      }
      if(info.h != h) {
	h = info.h;
	changed = true;
      }
      if(info.r != r) {
	r = info.r;
	changed = true;
      }
      if(info.q != q) {
	q = info.q;
	changed = true;
      }
      if(info.d != d) {
	d = info.d;
	changed = true;
      }
    }
  };

  struct BlurInfo {
    double blur;
    bool changed;

    BlurInfo() = default;

    BlurInfo(double blur) 
      : blur(blur),
	changed(false) {}

    BlurInfo& operator=(const BlurInfo& info) {
      changed = false;
      if(info.blur != blur) {
	blur = info.blur;
	changed = true;
      }
    }
  };

  template<typename PARAM>
  class LGN {
    
  private:

    double map_rho(double rad) {
      return r*pow(rad/q,map_info.d);
    }

    void make_mapping() {
      double h_origin = .5*map_info.height/map_info.width;
      cv::Point2i center_source((int)((map_info.w+.5)*map_info.width+.5),
				(int)((map_info.h+h_origin)*map_info.width+.5));// yes, width twice...
      w_map.create(map_info.lgnSide,map_info.lgnSide,cv::DataType<int>::type);
      h_map.create(map_info.lgnSide,map_info.lgnSide,cv::DataType<int>::type);
      
      cv::Point2i center_lgn(map_info.lgnSide/2,map_info.lgnSide/2);

      for(int h=0; h < map_info.lgnSide; ++h)
	for(int w=0; w < map_info.lgnSide; ++w) {
	  cv::Point2i pos = cv::Point2i(w,h);
	  cv::Point2i r   = pos - center_lgn;
	  int radius = r.x*r.x+r.y*r.y;
	  if(radius == 0) {
	    w_map.at<int>(pos) = center_source.x;
	    h_map.at<int>(pos) = center_source.y;
	  }
	  else {
	    double rad = sqrt(radius);
	    double rho = map_rho(rad);
	    cv::Point2f dest(r.x/rad,r.y/rad);
	    w_map.at<int>(pos) = (int)(center_source.x+rho*dest.x+.5);
	    h_map.at<int>(pos) = (int)(center_source.y+rho*dest.y+.5);
	  }
	}
    }
    
    bool in_bounds(const cv::Point2i& p, int width, int height) {
      return p.x >= 0 && p.y >=0 && p.x < width && p.y < height;
    }

    void map(const cv::Mat& src) {
      int W  = lgn.cols;
      int H  = lgn.rows;
      int sW = src.cols;
      int sH = src.rows;

      for(int h=0; h < H; ++h)
	for(int w=0; w < W; ++w) {
	  cv::Point2i pos(w_map.at<int>(w,h),h_map.at<int>(w,h));
	  if(in_bounds(pos,sW,sH)) {
	    cv::Vec3b&       out = lgn.at<cv::Vec3b>(w,h);
	    const cv::Vec3b& in  = src.at<cv::Vec3b>(pos);
	    out = in;
	  }
	}
    }

    void extract_color(const cv::Vec3b& src,
		       double& r, double& g, double& b) {
      r = src[0];
      g = src[1];
      b = src[2];
    }

    void make_rgb() {
      int W  = lgn.rows;
      int H  = lgn.cols;
      red.create  (W, H, cv::DataType<double>::type);
      green.create(W, H, cv::DataType<double>::type);
      blue.create (W, H, cv::DataType<double>::type);
      
      for(int h=0; h < H; ++h)
	for(int w=0; w < W; ++w) {
	  cv::Vec3b& src = lgn.at<cv::Vec3b>(w,h);
	  extract_color(src,red.at<double>(w,h),green.at<double>(w,h),blue.at<double>(w,h));
	}
    }
    
    unsigned char double_to_gray(double d) {
      if(d>255)
	return 255;
      if(d<0)
	return 0;
      return (unsigned char)(d+.5);
    }
    
    void merge_rgb() {
      int W  = lgn.rows;
      int H  = lgn.cols;
      
      for(int h=0; h < H; ++h)
	for(int w=0; w < W; ++w) {
	  cv::Vec3b& out = lgn.at<cv::Vec3b>(w,h);
	  out[0] = double_to_gray(  red.at<double>(w,h));
	  out[1] = double_to_gray(green.at<double>(w,h));
	  out[2] = double_to_gray( blue.at<double>(w,h));
	}
    }
    
    cv::Mat w_map,h_map;
    cv::Mat red,green,blue;
    
    MapInfo map_info;
    BlurInfo blur_info;
    double min_sat, max_sat;
    int blur_side;
    double sigma;
    cv::Mat blurred_source;
    double q,r;

  public:

    const PARAM& param;
    cv::Mat& lgn;

  private:

 
    void check_map_focus(double w, double h) {
      map_info = MapInfo(param.lgnSide(), 
			 map_info.width, map_info.height,
			 w, h,
			 param.r(), param.q(), param.d());
      if(map_info.changed) {
	update_blur();
	update_rq();
      }
    }

    void check_map_input(double width, double height) {
      map_info = MapInfo(param.lgnSide(), 
			 width, height,
			 map_info.w, map_info.h,
			 param.r(), param.q(), param.d());
      if(map_info.changed) {
	update_blur();
	update_rq();
      }
    }

    void check_blur() {
      blur_info = BlurInfo(param.blur());
      if(blur_info.changed)
	update_blur();
    }

    void update_blur() {
      sigma = blur_info.blur*map_info.width;
      blur_side = (int)(4*sigma+.5);
      if(blur_side % 2 == 0) // Must be odd
	blur_side++;
    }

    void update_rq() {
      r = map_info.r*map_info.width;
      q = map_info.q*map_info.lgnSide;
    }


  public:


    cv::Point2i lgnSize() {
      return cv::Point2i(param.lgnSide(),param.lgnSide());
    }

    void setFocus(double w, double h) {
      check_map_focus(w,h);
      if(map_info.changed) make_mapping();
    }


    LGN(const PARAM& p, cv::Mat& output) : param(p), lgn(output) {
    }


    cv::Point2f whereIs(const cv::Point2f& lgn_position) {
      int x = (int)((lgn_position.x+.5)*map_info.lgnSide+.5);
      int y = (int)((lgn_position.y+.5)*map_info.lgnSide+.5);

      if(x<0) x=0;
      else if(x >= map_info.lgnSide) x=map_info.lgnSide-1;
      if(y<0) y=0;
      else if(y >= map_info.lgnSide) y=map_info.lgnSide-1;

      cv::Point2i pos(x,y);
      cv::Point2i res(w_map.at<int>(pos),h_map.at<int>(pos));
    
      return cv::Point2f((res.x-map_info.width/2)/(double)map_info.width,
      (res.y-map_info.height/2)/(double)map_info.width); // yes, denom is .width ! 
    }

    void operator()(const cv::Mat& source) {
      check_blur();
      check_map_input(source.cols,source.rows);
      lgn.create(param.lgnSide(),param.lgnSide(),CV_8UC3);
      lgn = cv::Scalar(0,0,0);
      if(map_info.changed) make_mapping();
      cv::GaussianBlur(source, blurred_source, cv::Size(blur_side, blur_side), sigma, sigma);
      map(blurred_source);
      make_rgb();
      merge_rgb();
    }
  };
}
