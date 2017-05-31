#include "convolution_fftw.h"
#include <opencv2/opencv.hpp> 


using namespace FFTW_Convolution;

namespace focus {

  class NeuralField {
  public:
    struct Parameters {
      // The following must be defined
      int width, height;
      double dtau_u, dtau_v; // Time constants
      double beta; // coefficient for the u(x,t) layer
      double noise; // noise in the v layer
      double sg; // variance of the g(x,y) gaussian
      double Ap, sp, Am, sm; // parameters of the dog lateral weights
      bool circular_symmetric; // is the field circular symmetric ?

      // The following will be computed by init_state
      int size;
      int w_width, w_height, w_size;
    };

    struct State {
      double* input;
      double* u, *fu;
      double* v, *fv;

      double * lat_u_weights;
      double * lat_v_weights;

      Workspace ws_g, ws_lat; // To perform the convolution with the FFT

      double* tmp_buf_fu; // buffer used for the synchronous update
    };

  private:
    Parameters parameters;
    State state;


  private:
    static double transfer_function(double x);
    static double gaussian(double A, double s, double d);
    static double distance(int i, int j, int k, int l, bool is_circular, int w, int h);// width, height, width, height 


    void compute_gweights();
    void compute_latweights();

  public:
    NeuralField(Parameters);
    ~NeuralField();
    
    void getSize(int& width, int& height);

    void updateDTauu(double);
    void updateDTauv(double);
    void updateBeta(double);
    void updateNoise(double);
    void updateSg(double);
    void updateLatWeights(double, double, double, double);
    
    void init();
    void free();
    void reinit();


    void reset();
    void new_input(double* input);
    void step();

    // output center of mass, over f(v)
    void get_com(double& x, double& y);

    // openCV interface
    void new_input(const cv::Mat& mat);
    void fill_output(cv::Mat& mat);
    void fill_input(cv::Mat& mat);
  };
}

