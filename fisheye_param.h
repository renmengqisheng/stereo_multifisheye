#ifndef CAM_SYSTEM_CALIB_FISHEYE_PARAMETER_H_
#define CAM_SYSTEM_CALIB_FISHEYE_PARAMETER_H_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64
const int ImageWidth = 1280;
struct ocam_model {
  std::vector<float> pol;    // the polynomial coefficients: pol[0] + x"pol[1]
                             // + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;            // length of polynomial
  std::vector<float> invpol; // the coefficients of the inverse polynomial
  int length_invpol;         // length of inverse polynomial
  double xc;                 // row coordinate of the center
  double yc;                 // column coordinate of the center
  double c;                  // affine parameter
  double d;                  // affine parameter
  double e;                  // affine parameter
  int width;                 // image width
  int height;                // image height
};

class FisheyeParam
{
public:
  FisheyeParam() {}
  void Load(const std::string &calbration_file);
  FisheyeParam(const std::string &calibration_file);
  ~FisheyeParam() {}
  cv::Point2f World2Camera(cv::Point3f X);
  cv::Point3f Camera2World(cv::Point2f pt);
  inline double horner(double x) {
    double res = 0.0;
    int s = ocam_model_.length_invpol;
    for (int i = s - 1; i >= 0; i--)
      res = res * x + ocam_model_.invpol[i];
    return res;
  }

private:
  ocam_model ocam_model_;
};

#endif // VALET_MAPPING_DEMO_CREATE_AVM_FISHEYE_PARAM_H
