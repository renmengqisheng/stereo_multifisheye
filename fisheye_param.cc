#include "fisheye_param.h"

using namespace std;

void FisheyeParam::Load(const string &calibration_file) {
  ifstream input_file;
  string s;
  input_file.open(calibration_file.c_str(), ifstream::in);
  getline(input_file, s); // first line
  getline(input_file, s); // second line
  getline(input_file, s); // third line
  stringstream ss(s);
  string strlength_pol;
  ss >> strlength_pol;

  ocam_model_.length_pol = atoi(strlength_pol.c_str());

  ocam_model_.pol.resize(ocam_model_.length_pol);
  for (int i = 0; i < ocam_model_.length_pol; i++) {
    string coef;
    ss >> coef;
    ocam_model_.pol[i] = atof(coef.c_str());
  }

  getline(input_file, s); // 4th
  getline(input_file, s); // 5th
  getline(input_file, s); // 6th
  getline(input_file, s); // 7th
  stringstream ssinv(s);
  string strlength_invpol;
  ssinv >> strlength_invpol;
  ocam_model_.length_invpol = atoi(strlength_invpol.c_str());
  ocam_model_.invpol.resize(ocam_model_.length_invpol);
  for (int i = 0; i < ocam_model_.length_invpol; i++) {
    string coef;
    ssinv >> coef;
    ocam_model_.invpol[i] = atof(coef.c_str());
  }

  getline(input_file, s); // 8th
  getline(input_file, s); // 9th
  getline(input_file, s); // 10th
  getline(input_file, s); // 11th

  stringstream uv(s);
  string stry;
  string strx;

  uv >> stry;
  uv >> strx;
  ocam_model_.yc = atof(stry.c_str());
  ocam_model_.xc = atof(strx.c_str());

  getline(input_file, s); // 12th
  getline(input_file, s); // 13th
  getline(input_file, s); // 14th
  getline(input_file, s); // 15th

  stringstream affinecoef(s);
  string strc;
  string strd;
  string stre;
  affinecoef >> strc;
  affinecoef >> strd;
  affinecoef >> stre;

  ocam_model_.c = atof(strc.c_str());
  ocam_model_.d = atof(strd.c_str());
  ocam_model_.e = atof(stre.c_str());

  getline(input_file, s); // 16th
  getline(input_file, s); // 17th
  getline(input_file, s); // 18th
  getline(input_file, s); // 19th

  stringstream imagesize(s);
  string strwidth;
  string strheight;
  imagesize >> strheight;
  imagesize >> strwidth;
  ocam_model_.height = atoi(strheight.c_str());
  ocam_model_.width = atoi(strwidth.c_str());
}

FisheyeParam::FisheyeParam(const std::string &calibration_file) {
  Load(calibration_file);
}

cv::Point2f FisheyeParam::World2Camera(cv::Point3f X) {
  ocam_model &ocam = ocam_model_;
  double norm = sqrt(X.x * X.x + X.y * X.y);
  if (norm == 0.0)
    norm = 1e-14;
  const double theta = atan(-X.z / norm);
  const double rho = horner(theta);

  const double uu = X.x / norm * rho;
  const double vv = X.y / norm * rho;
  cv::Point2f m;
  m.x = uu * ocam.c + vv * ocam.d + ocam.xc;
  m.y = uu * ocam.e + vv + ocam.yc;
  return m;
}

cv::Point3f FisheyeParam::Camera2World(cv::Point2f pt) {
  ocam_model &ocam = ocam_model_;
  // double invAff = c - d*e;

  const double u_t = pt.x - ocam.xc;
  const double v_t = pt.y - ocam.yc;
  float invAffine = ocam.c - ocam.d * ocam.e;
  // inverse affine matrix image to sensor plane conversion
  float x = (u_t - ocam.d * v_t) / invAffine;
  float y = (-ocam.e * u_t + ocam.c * v_t) / invAffine;
  const double X2 = x * x;
  const double Y2 = y * y;
  //  float z = -horner((double*)p.data, p_deg, sqrt(X2 + Y2));

  //  double res = 0.0;
  //  int s = ocam.length_invpol;
  //  for (int i = s - 1; i >= 0; i--)
  //    res = res * x + ocam.invpol[i];
  //  return res;

  //  inline double horner(
  //          const double* coeffs, const int& s, const double& x)
  //  {
  //          double res = 0.0;
  //          for (int i = s - 1; i >= 0; i--)
  //                  res = res * x + coeffs[i];
  //          return res;
  //  }
  float value = sqrt(X2 + Y2);
  int s = ocam.length_pol;
  double res = 0.0;
  ;
  for (int i = s - 1; i >= 0; i--) {
    res = res * value + ocam.pol[i];
  }
  float z = -res;

  // normalize vectors spherically
  double norm = sqrt(X2 + Y2 + z * z);
  x /= norm;
  y /= norm;
  z /= norm;
  return cv::Point3f(x, y, z);
}
