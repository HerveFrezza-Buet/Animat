#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp> 

#include "neuralfield.hpp"

double focus::NeuralField::transfer_function(double x) {
  if(x <= 0.0)
    return 0.0;
  else
    return x;
}

double focus::NeuralField::gaussian(double A, double s, double d) {
  return A * exp(-d*d/(2.0 * s * s));
}

double focus::NeuralField::distance(int i, int j, int k, int l, bool is_circular, int w, int h) {
  if(is_circular)
    {
      double xmin = std::min(w - abs(i - k), abs(i - k));
      double ymin = std::min(h - abs(j - l), abs(j - l));
      return sqrt(xmin*xmin + ymin+ymin);
    }
  else
    {
      int xdiff = i - k;
      int ydiff = j - l;
      return sqrt(xdiff*xdiff + ydiff*ydiff);
    }
}


void focus::NeuralField::compute_gweights() {
  double * tmp = state.lat_u_weights;
  int mid_w = int(parameters.w_width / 2.0);
  int mid_h = int(parameters.w_height / 2.0);

  for(unsigned int i = 0 ; i < parameters.w_height ; ++i){
    for(unsigned int j = 0 ; j < parameters.w_width ; ++j) {
      double d = distance(j, i, mid_w, mid_h, parameters.circular_symmetric, parameters.w_width, parameters.w_height);
      *tmp++ = gaussian(1.0, parameters.sg, d);
    }
  }
}

void focus::NeuralField::compute_latweights() {
  double * tmp = state.lat_v_weights;
  int mid_w = int(parameters.w_width / 2.0);
  int mid_h = int(parameters.w_height / 2.0);

  for(unsigned int i = 0 ; i < parameters.w_height ; ++i){
    for(unsigned int j = 0 ; j < parameters.w_width ; ++j) {
      double d = distance(j, i, mid_w, mid_h, parameters.circular_symmetric, parameters.w_width, parameters.w_height);
      *tmp++ = gaussian(parameters.Ap, parameters.sp, d) - gaussian(parameters.Am, parameters.sm, d);
    }
  }
}


focus::NeuralField::NeuralField(Parameters p): parameters(p) {
  init();
}

focus::NeuralField::~NeuralField() {
  free();
}

void focus::NeuralField::getSize(int& width, int &height) {
  width = parameters.width;
  height = parameters.height;
}

void focus::NeuralField::updateDTauu(double value){
  parameters.dtau_u = value;
}
void focus::NeuralField::updateDTauv(double value){
  parameters.dtau_v = value;
}

void focus::NeuralField::updateBeta(double value){
  parameters.beta = value;
}
void focus::NeuralField::updateNoise(double value){
  parameters.noise = value;
}
void focus::NeuralField::updateSg(double value){
  parameters.sg = value;
  reinit();
}

void focus::NeuralField::updateLatWeights(double Ap, double sp, double Am, double sm){
  if(sp == 0 || sm == 0) {
    std::cerr << "Null values for sp or sm are forbidden; Your changes are not taken into account" << std::endl;
    return;
  }
  parameters.Ap = Ap;
  parameters.Am = Am;
  parameters.sp = sp;
  parameters.sm = sm;
  reinit();
}
    
void focus::NeuralField::init(){
  parameters.size = parameters.width * parameters.height;
  state.input = new double[parameters.size];
  state.u = new double[parameters.size];
  state.fu = new double[parameters.size];
  state.v = new double[parameters.size];
  state.fv = new double[parameters.size];

  state.tmp_buf_fu = new double[parameters.size];

  if(parameters.circular_symmetric) {
    parameters.w_width = parameters.width;
    parameters.w_height = parameters.height;
  }
  else {
    parameters.w_width = 2*parameters.width-1;
    parameters.w_height = 2*parameters.height-1;
  }
  parameters.w_size = parameters.w_width * parameters.w_height;

  state.lat_u_weights = new double[parameters.w_size];
  state.lat_v_weights = new double[parameters.w_size];

  init_workspace(state.ws_g, parameters.circular_symmetric ? CIRCULAR : LINEAR,parameters.height, parameters.width, parameters.w_height, parameters.w_width);
  init_workspace(state.ws_lat, parameters.circular_symmetric ? CIRCULAR : LINEAR,parameters.height, parameters.width, parameters.w_height, parameters.w_width);

  compute_gweights();
  compute_latweights();

  reset();
}
void focus::NeuralField::free(){
  if(state.input != 0) delete[] state.input;
  state.input = 0;

  if(state.u != 0) delete[] state.u;
  state.u = 0;

  if(state.fu != 0) delete[] state.fu;
  state.fu = 0;

  if(state.v != 0) delete[] state.v;
  state.v = 0;

  if(state.fv != 0) delete[] state.fv;
  state.fv = 0;

  if(state.tmp_buf_fu != 0) delete[] state.tmp_buf_fu;
  state.tmp_buf_fu = 0;

  if(state.lat_u_weights != 0) delete[] state.lat_u_weights;
  state.lat_u_weights = 0;

  if(state.lat_v_weights != 0) delete[] state.lat_v_weights;
  state.lat_v_weights = 0;

  FFTW_Convolution::clear_workspace(state.ws_g);
  FFTW_Convolution::clear_workspace(state.ws_lat);
}
void focus::NeuralField::reinit(){
  free();
  init();
}

void focus::NeuralField::reset(){
  memset(state.input, 0, parameters.size * sizeof(double));
  memset(state.u, 0, parameters.size * sizeof(double));
  memset(state.fu, 0, parameters.size * sizeof(double));
  memset(state.v, 0, parameters.size * sizeof(double));
  memset(state.fv, 0, parameters.size * sizeof(double));
}

void focus::NeuralField::new_input(double* input){
  memcpy(state.input, input, parameters.size * sizeof(double));
}

void focus::NeuralField::step() {
  // We need to copy tmp_buf_fu for the synchronous update
  memcpy(state.tmp_buf_fu, state.fu, parameters.size * sizeof(double));

  // We compute the term : sum_y f(v(y)) * i(y)
  double* iptr = state.input;
  double* fvptr = state.fv;
  double sum_fvi = 0.0;
  for(unsigned int i = 0 ; i < parameters.size; ++i, ++fvptr, ++iptr)
    sum_fvi += (*fvptr) * (*iptr);

  // Compute the convolution in the U layer
  convolve(state.ws_g, state.input, state.lat_u_weights);

  // Compute the lateral contribution in the V layer
  convolve(state.ws_lat, state.fv, state.lat_v_weights);

  // Update the U layer
  double* uptr = state.u;
  double* fuptr = state.fu;
  double* tmp_ptr = state.ws_g.dst;

  for(unsigned int i = 0 ; i < parameters.size; ++i, ++uptr, ++tmp_ptr, ++fuptr)
    {
      *uptr = *uptr + parameters.dtau_u * (- *uptr + parameters.beta * (*tmp_ptr-sum_fvi));
      *fuptr = transfer_function(*uptr);
    }

  // Update the V layer
  fvptr = state.fv;
  double* vptr = state.v;
  fuptr = state.tmp_buf_fu;
  tmp_ptr = state.ws_lat.dst;
  for(unsigned int i = 0 ; i < parameters.size ; ++i, ++fuptr, ++vptr, ++fvptr, ++tmp_ptr)
    {
      *vptr = *vptr + parameters.dtau_v * (- *vptr + *fuptr + *tmp_ptr + parameters.noise * (2.0 * rand()/double(RAND_MAX-1) - 1.0));
      *fvptr = transfer_function(*vptr);
    }
}


void focus::NeuralField::new_input(const cv::Mat& input){
  cv::Mat resized;
  cv::resize(input, resized, cv::Size(parameters.width, parameters.height));

  unsigned char* dptr = resized.data;
  double* iptr = state.input;
  for(unsigned int i = 0 ; i < parameters.size ; ++i, ++dptr, ++iptr)
    *iptr = double(*dptr) / 255.;
}

void focus::NeuralField::fill_output(cv::Mat& output) {
  output.create(parameters.width, parameters.height, CV_8UC1);
  unsigned char* dptr = output.data;
  double* fvptr = state.fv;
  for(unsigned int i = 0 ; i < parameters.size ; ++i, ++dptr, ++fvptr) {
    double v = *fvptr;
    v = v > 1.0 ? 1.0 : v;
    *dptr = (unsigned char)(255 * v);
  }
}

void focus::NeuralField::fill_input(cv::Mat& input) {
  input.create(parameters.width, parameters.height, CV_8UC1);
  unsigned char* dptr = input.data;
  double* iptr = state.input;
 
  for(unsigned int i = 0 ; i < parameters.size ; ++i, ++dptr, ++iptr) {
    double v = *iptr;
    v = v > 1.0 ? 1.0 : v;
    *dptr = (unsigned char)(255 * v);
  }
}

void focus::NeuralField::get_com(double& x, double& y) {
  // x = 0 , y = 0 , bottom left
  // x : width ; y : height
  double *fvptr = state.fv;
  double sum = 0.0;
  x = 0.0;
  y = 0.0;
  for(unsigned int i = 0 ; i < parameters.height ; ++i)
    for(unsigned int j = 0 ; j < parameters.width; ++j, ++fvptr) {
      sum += *fvptr;
      y += i * (*fvptr);
      x += j * (*fvptr);
    }

  if(sum <= 1e-5) {
    x = -1;
    y = -1;
  }
  else {
    x /= (sum * parameters.width);
    y /= (sum * parameters.height);
  }
}
