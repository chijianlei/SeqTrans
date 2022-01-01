#include <swri_nav_util/savitzky_golay.h>
#include <rclcpp/rclcpp.hpp>

namespace swri_nav_util
{

//TODO, make these implementations faster. more efficient convolution

bool sgolaySmooth(
    const int frame_size, // odd and between 5-15
    const std::vector<double>& f,
    std::vector<double>& fsmooth) //smoothed f
{
  fsmooth = f; //init

  //get coefficients
  double denom;
  std::vector<double> a;
  if (frame_size == 5) { denom = 35; a = {17, 12, -3}; }
  else if (frame_size == 7) { denom = 21; a = {7, 6, 3, -2}; }
  else if (frame_size == 9) { denom = 231; a = {59, 54, 39, 14, -21}; }
  else if (frame_size == 11) { denom = 429; a = {89, 84, 69, 44, 9, -36}; }
  else if (frame_size == 13) { denom = 143; a = {25, 24, 21, 16, 9, 0, -11}; }
  else if (frame_size == 15) { denom = 1105; a = {167, 162, 147, 122, 87, 42, -13, -78}; } //TODO, add more
  else {
    printf("ERROR: frame_size input = %d invalid. must be odd and between 5-15", frame_size);//error
    return false;
  }

  int m = (frame_size-1)/2;
  int numel = f.size(); //abbreviate
  if (numel < m+1)
  {
    printf("ERROR: f is too small for frame_size (numel = %d < (frame_size-1)/2 = %d)", numel, m);//error
    return false;
  }

  //calc smooth values
  for (int i=0; i < numel; i++)
  {
    double sum = a[0]*f[i];
    for (size_t k=1; k<=m; k++)
    {
      sum +=
          a[k]*indexPadVec(f,i-k,NEAREST);
          a[k]*indexPadVec(f,i+k,NEAREST);
    }
    fsmooth[i] = sum/denom;
  }
  return true;
}

bool sgolayDiff1(
    const int frame_size, //must be odd and >= 3, 3 for central difference
    const double step_size,
    const std::vector<double>& f,
    std::vector<double>& fd) //first derivative of f
{
  fd.resize(f.size()); //init

  //check inputs
  if (frame_size % 2 != 1 || frame_size < 3)
  {
    printf("ERROR: frame_size input = %d invalid. must be odd and >= 3", frame_size);
    return false;
  }

  int m = (frame_size-1)/2;
  int numel = f.size(); //abbreviate
  if (numel < m+1)
  {
    printf("ERROR: f is too small for frame_size (numel = %d < (frame_size-1)/2 = %d)", numel, m);
    return false;
  }

  double coeff = 3.0/(step_size*m*(m+1)*(2.0*m+1));

  //calc smooth derivative
  for (int i=0; i < numel; i++)
  {
    double sum = 0;
    for (int k=1; k<=m; k++)
    {
      double fm = indexPadVec(f,i-k,REFLECT_FLIP);
      double fp = indexPadVec(f,i+k,REFLECT_FLIP);
      sum += k*(fp-fm);
    }
    fd[i] = coeff * sum;
  }
  return true;
}

} //namespace
