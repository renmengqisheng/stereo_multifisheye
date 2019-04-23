/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images

   NOTE, IF YOU WANT TO SPEED UP THE REMAP FUNCTION I STRONGLY RECOMMEND TO INSTALL
   INTELL IPP LIBRARIES ( http://software.intel.com/en-us/intel-ipp/ )
   YOU JUST NEED TO INSTALL IT AND INCLUDE ipp.h IN YOUR PROGRAM

   Copyright (C) 2009 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "fisheye_param.h"
#include <iostream>
#include <string>

const int slider_max = 5000;
int fx, fy, xc, yc;
cv::Mat large;
cv::Mat src_left, src_front, src_right, src_back;
cv::Mat dst_left, dst_front, dst_right, dst_back;
cv::Mat result_left, result_front, result_right, result_back;
FisheyeParam ocam_model_left, ocam_model_front, ocam_model_right, ocam_model_back;

cv::Matx33d Rotation_left, Rotation_front, Rotation_right, Rotation_back;
cv::Vec3d Translation_left, Translation_front, Translation_right, Translation_back;

cv::Matx33d Rotation_left2front, Rotation_front2right, Rotation_right2back, Rotation_back2front;
cv::Vec3d Translation_left2front, Translation_front2right, Translation_right2back, Translation_back2front;

cv::Matx33d Rotation_left_l, Rotation_left_r, Rotation_front_l, Rotation_front_r;
cv::Matx33d Rotation_right_l, Rotation_right_r, Rotation_back_l, Rotation_back_r;

cv::Vec3f euler1, euler2;

cv::Matx33d Rotation_x(float angle)
{
  //angle = angle * CV_PI / 180;
  return cv::Matx33d(1, 0, 0, 0, cos(angle), sin(angle), 0, -sin(angle), cos(angle));
}

cv::Matx33d Rotation_y(float angle)
{
  //angle = angle * CV_PI / 180;
  return cv::Matx33d(cos(angle), 0, -sin(angle), 0, 1, 0, sin(angle), 0, cos(angle));
}

cv::Matx33d Rotation_z(float angle)
{
  //angle = angle * CV_PI / 180;
  return cv::Matx33d(cos(angle), sin(angle), 0, -sin(angle), cos(angle), 0, 0, 0, 1);
}

void translateTwc2Tcw(cv::Matx33d& R, cv::Vec3d& t)
{
  R = R.t();  //由Twc转化为Tcw
  t = -R * t;
}

void readTransformation(std::string filename, cv::Matx33d& R, cv::Vec3d& t)
{
  std::ifstream result_in(filename.data());
  assert(result_in.is_open());

  cv::Matx44d T;
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 4; j++)
    {
        result_in >> T(i,j);
    }
  }

  R = T.get_minor<3,3>(0,0);
  t = cv::Vec3d(T(0,3), T(1,3), T(2,3));
  result_in.close();

  translateTwc2Tcw(R, t);
}

void Load()
{
  /* --------------------------------------------------------------------*/
  /* Read the parameters of the omnidirectional camera from the TXT file */
  /* --------------------------------------------------------------------*/
  ocam_model_left.Load("../intrinsic_parameters/left/calib.txt");
  ocam_model_front.Load("../intrinsic_parameters/front/calib.txt");
  ocam_model_right.Load("../intrinsic_parameters/right/calib.txt");
  ocam_model_back.Load("../intrinsic_parameters/back/calib.txt");

  /* --------------------------------------------------------------------*/
  /* Allocate space                           */
  /* --------------------------------------------------------------------*/
  src_left = cv::imread("../bmp/frame_vc9_1814.bmp"); 
  assert(!src_left.empty());
  src_front = cv::imread("../bmp/frame_vc10_1814.bmp"); 
  assert(!src_front.empty());
  src_right = cv::imread("../bmp/frame_vc11_1814.bmp"); 
  assert(!src_right.empty());
  src_back = cv::imread("../bmp/frame_vc12_1814.bmp"); 
  assert(!src_back.empty());

  readTransformation("../result/left.txt", Rotation_left, Translation_left);  //Rcw, tcw
  readTransformation("../result/front.txt", Rotation_front, Translation_front);
  readTransformation("../result/right.txt", Rotation_right, Translation_right);
  readTransformation("../result/back.txt", Rotation_back, Translation_back);
}

void initImages()
{
  large = cv::Mat (src_left.rows, src_left.cols*2, src_left.type(), cv::Scalar::all(0));

  dst_left = cv::Mat(src_left.rows, src_left.cols, src_left.type(), cv::Scalar::all(0));
  dst_front = cv::Mat(src_front.rows, src_front.cols, src_front.type(), cv::Scalar::all(0));
  dst_right = cv::Mat(src_right.rows, src_right.cols, src_right.type(), cv::Scalar::all(0));
  dst_back = cv::Mat(src_back.rows, src_back.cols, src_back.type(), cv::Scalar::all(0));

  result_left = cv::Mat(src_left.rows, src_left.cols, src_left.type(), cv::Scalar::all(0));
  result_front = cv::Mat(src_front.rows, src_front.cols, src_front.type(), cv::Scalar::all(0));
  result_right = cv::Mat(src_right.rows, src_right.cols, src_right.type(), cv::Scalar::all(0));
  result_back = cv::Mat(src_back.rows, src_back.cols, src_back.type(), cv::Scalar::all(0));
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(const cv::Matx33d &R)
{
    cv::Matx33d Rt = R.t();
    cv::Matx33d shouldBeIdentity = Rt * R;
    cv::Matx33d I = cv::Matx33d::eye();
 
    return  norm(I, shouldBeIdentity) < 1e-5;
}

// Calculates rotation matrix to euler angles(the order is z first, then x, y last).
cv::Vec3f rotationMatrixToEulerAngles(const cv::Matx33d &R)
{
    assert(isRotationMatrix(R));

    //float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0));
    float sy = sqrt(R(0,0) * R(0,0) +  R(0,1) * R(0,1));
    //float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(0,2) * R.at<double>(0,2));
    bool singular = sy < 1e-5; // If
 
    cv::Vec3f result;
    if (!singular)
    {
        //先绕x再绕y最后绕z旋转, don't work
        /*
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        */
        //先绕z再绕y最后绕x旋转, work
        
        result(0) = atan2(R(1,2) , R(2,2));
        result(1) = atan2(-R(0,2), sy);
        result(2) = atan2(R(0,1), R(0,0));
        
        //先绕y再绕z最后绕x旋转, don't work
        /*
        x = atan2(-R.at<double>(2,1) , R.at<double>(1,1));
        y = atan2(-R.at<double>(0,2), R.at<double>(0,0));
        z = atan2(R.at<double>(0,1), sy);
        */
        
    }
    else
    {
        //先绕x再绕y最后绕z旋转, don't work
        /*
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
        */
        //先绕z再绕y最后绕x旋转, work
        
        result(0) = atan2(R(1,0), R(2,0));
        result(1) = atan2(-R(0,2), sy);
        result(2) = 0;
        
        //先绕y再绕z最后绕x旋转, don't work
        /*
        x = atan2(R.at<double>(1,2), R.at<double>(2,2));
        y = 0;
        z = atan2(R.at<double>(0,1), sy);
        */
    }

    return result;
}

void calculateTwoRotation(cv::Matx33d R, cv::Vec3d tvec, cv::Matx33d& R1, cv::Matx33d& R2)
{
  cv::Vec3d rvec = cv::Affine3d(R).rvec();

  // rectification algorithm
  rvec *= -0.5;              // get average rotation

  cv::Matx33d r_r;
  Rodrigues(rvec, r_r);  // rotate cameras to same orientation by averaging
  cv::Vec3d t = r_r * tvec;

  // double nt = norm(t);
  // if(nt > 0.0)
  //   t /= nt;
  // cv::Vec3d uu(0, 0, t(0) > 0 ? 1 : -1);
  // cv::Vec3d ww = t.cross(uu);
  // double nw = norm(ww);
  // if(nw > 0.0)
  //   ww /= nw;
  // cv::Vec3d vv = t.cross(ww);
  // double nv = norm(vv);
  // if(nv > 0.0)
  //   vv /= nv;
  // cv::Matx33d Rrect(t(0), t(1), t(2), ww(0), ww(1), ww(2), vv(0), vv(1), vv(2));
  // R1 = Rrect * r_r.t();
  // R2 = Rrect * r_r;

  cv::Vec3d uu(t[0] > 0 ? 1 : -1, 0, 0);

  // calculate global Z rotation
  cv::Vec3d ww = t.cross(uu);
  double nw = norm(ww);
  if (nw > 0.0)
      ww *= acos(fabs(t[0])/cv::norm(t))/nw;

  cv::Matx33d wr;
  Rodrigues(ww, wr);

  // apply to both views
  R1 = wr * r_r.t();
  R2 = wr * r_r;
}

void calculateTwoCorrectRotation(const cv::Matx33d& R1_old, const cv::Vec3d& t1_old, const cv::Matx33d& R2_old, const cv::Vec3d& t2_old, cv::Matx33d& R_l, cv::Matx33d& R_r)
{
  cv::Vec3f euler1 = rotationMatrixToEulerAngles(R1_old);  //R1, R2为Rcw
  cv::Vec3f euler2 = rotationMatrixToEulerAngles(R2_old);

  cv::Matx33d Rx1 = Rotation_x(euler1(0));
  cv::Matx33d Rx2 = Rotation_x(euler2(0));
  cv::Matx33d Rrect = Rotation_x(CV_PI/2);

  cv::Matx33d R1 = Rrect * Rx1.t() * R1_old;
  cv::Vec3d t1 = Rrect * Rx1.t() * t1_old;
  cv::Matx33d R2 = Rrect * Rx2.t() * R2_old;
  cv::Vec3d t2 = Rrect * Rx2.t() * t2_old;

  cv::Matx33d R = R2 * R1.t();
  cv::Vec3d tvec = t2 - R * t1;

  cv::Vec3d rvec = cv::Affine3d(R).rvec();

  // rectification algorithm
  rvec *= -0.5;              // get average rotation
  cv::Matx33d r_r;
  Rodrigues(rvec, r_r);  // rotate cameras to same orientation by averaging

  cv::Vec3d t = r_r * tvec;
  cv::Vec3d uu(t[0] > 0 ? 1 : -1, 0, 0);

  // calculate global Z rotation
  cv::Vec3d ww = t.cross(uu);
  double nw = norm(ww);
  if (nw > 0.0)
      ww *= acos(fabs(t[0])/cv::norm(t))/nw;

  cv::Matx33d wr;
  Rodrigues(ww, wr);

  // apply to both views
  R_l = wr * r_r.t();
  R_r = wr * r_r;

  R_l = R_l * Rrect * Rx1.t();
  R_r = R_r * Rrect * Rx2.t();
}

void Fisheye2twoPerspective(cv::Mat src, cv::Mat dst, float fx, float fy, float xc, float yc, FisheyeParam& ocam_model)
{
  cv::Size image_size = src.size();
  float theta1 = -45*CV_PI/180.0, theta2 = 45*CV_PI/180.0;

  if(src.empty() || dst.empty())
  {
    std::cout << __FILE__ << ": "<< __LINE__ << " error!" << std::endl;
    return;
  }

  cv::Vec3f center(0, 0, 1);
  cv::Vec3f center1 = Rotation_y(-(theta2-theta1)/2).t() * center;
  cv::Vec3f center2 = Rotation_y((theta2-theta1)/2).t() * center;
  float xc1 = xc + fx * center1(0)/center1(2), yc1 = yc + fy * center1(1)/center1(2); //theta1 < 0 theta2 > 0 找到左右针孔的主点
  float xc2 = xc + fx * center2(0)/center2(2), yc2 = yc + fy * center2(1)/center2(2);
  
  for(int i = 0; i < image_size.height; i++)
    for(int j = 0; j < image_size.width; j++)
    {
      cv::Vec3f world_point;
      if(j <= xc)
      {
        world_point(0) = (j-xc1) / fx;
        world_point(1) = (i-yc1) / fy;
        world_point(2) = 1;
        float theta = -(theta2-theta1)/2;
        world_point = Rotation_y(theta).t() * world_point;  //转回原相机坐标系
      }
      else
      {
        world_point(0) = (j-xc2) / fx;
        world_point(1) = (i-yc2) / fy;
        world_point(2) = 1;
        float theta = (theta2-theta1)/2;
        world_point = Rotation_y(theta).t() * world_point;  //转回原相机坐标系
      }
      cv::Point2f point2d = ocam_model.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
      int u = point2d.x, v = point2d.y;
      if(u >= 0 && u < src.cols && v >= 0 && v < src.rows)
        dst.at<cv::Vec3b>(i,j) = src.at<cv::Vec3b>(v,u);
    }
}

/*
void Fisheye2twoPerspective(cv::Mat src, cv::Mat dst, float fx, float fy, float xc, float yc, FisheyeParam& ocam_model)
{
  cv::Size image_size = src.size();
  float theta1 = -45*CV_PI/180.0, theta2 = 10*CV_PI/180.0;
  for(int i = 0; i < image_size.height; i++)
    for(int j = 0; j < image_size.width; j++)
    {
      cv::Point2f point2d;
      point2d.x = j;
      point2d.y = i;
      cv::Point3f point3d = ocam_model.Camera2World(point2d);
      float x = point3d.x;
      float y = point3d.y;
      float z = point3d.z;
      int u, v;
      if(x <= tan((theta1+theta2)/2)*point3d.z)
      {
        float alpha = -(theta2-theta1)/2;
        point3d.x = -z*sin(theta1) + x*cos(theta1);
        point3d.y = y; 
        point3d.z = z*cos(theta1) + x*sin(theta1);
        //u = point3d.x/point3d.z*fx + xc - fx*tan(theta1);
        u = point3d.x/point3d.z*fx + xc + fx*tan(alpha);
        v = point3d.y/point3d.z*fy + yc;
      }
      else
      {
        float alpha = (theta2-theta1)/2;
        point3d.x = -z*sin(theta2) + x*cos(theta2);
        point3d.y = y;
        point3d.z = z*cos(theta2) + x*sin(theta2);
        //u = point3d.x/point3d.z*fx + xc - fx*tan(theta2);
        u = point3d.x/point3d.z*fx + xc + fx*tan(alpha);
        v = point3d.y/point3d.z*fy + yc;
      }
      
      if(u >= 0 && u < dst.cols && v >= 0 && v < dst.rows)
        dst.at<cv::Vec3b>(v, u) = src.at<cv::Vec3b>(i,j);
    }
}
*/

void Fisheye2Perspective(cv::Mat src, cv::Mat dst, float fx, float fy, cv::Matx33d Rrect, FisheyeParam& ocam_model)
{
  if(src.empty() || dst.empty())
  {
    std::cout << __FILE__ << ": "<< __LINE__ << " error!" << std::endl;
    return;
  }

  cv::Size image_size = dst.size();
  for(int i = 0; i < image_size.height; i++)
    for(int j = 0; j < image_size.width; j++)
    {
      cv::Point3f point3d;
      float x, y, z;

      
      x = (j - xc) / fx;
      y = (i - yc) / fy;
      z = 1;
      cv::Vec3d p(x,y,z);
      p = Rrect.t() * p;
      point3d.x = p[0];
      point3d.y = p[1];
      point3d.z = p[2];
      
      cv::Point2f point2d = ocam_model.World2Camera(point3d);

      int u = point2d.x, v = point2d.y;
      if(u >= 0 && u < src.cols && v >= 0 && v < src.rows)
        dst.at<cv::Vec3b>(i,j) = src.at<cv::Vec3b>(v,u);
    }
}

void twoFisheye2twoPerspective(cv::Mat src1, cv::Mat src2, cv::Mat dst1, cv::Mat dst2, float fx, float fy, 
                                cv::Matx33d Rrect1, cv::Matx33d Rrect2, FisheyeParam& ocam_model1, FisheyeParam& ocam_model2)
{
  if(src1.empty() || dst1.empty() || src2.empty() || dst2.empty())
  {
    std::cout << __FILE__ << ": "<< __LINE__ << " error!" << std::endl;
    return;
  }

  cv::Size image_size = dst1.size();
  
  float theta1 = 45*CV_PI/180, theta2 = -45*CV_PI/180;
  float xc1 = xc - fx * tan(theta1), yc1 = yc;
  float xc2 = xc - fx * tan(theta2), yc2 = yc;
  
  // cv::Matx33d Rymean = Ryright * Ryback.t();
  // cv::Vec3d rvec = cv::Affine3d(Rymean).rvec() * 0.5;
  // cv::Rodrigues(rvec, Rymean);

  for(int i = 0; i < image_size.height; i++)
    for(int j = 0; j < image_size.width; j++)
    {
      cv::Point3f point3d, point3d1, point3d2;
      float x, y, z;

      if(j <= xc )
      {
        x = (j-xc1) / fx;
        y = (i-yc1) / fy;
        z = 1;
        float theta = -theta1;
        point3d.x = z*sin(theta) + x*cos(theta);
        point3d.y = y;
        point3d.z = z*cos(theta) - x*sin(theta);
        cv::Vec3d p(point3d.x,point3d.y,point3d.z);
        //cv::Vec3d p(x,y,z);
        p = Rrect2.inv() * p;
        point3d1.x = point3d.x;
        point3d1.y = point3d.y;
        point3d1.z = point3d.z;
        //point3d1.x = x;
        //point3d1.y = y;
        //point3d1.z = z;
        point3d2.x = p[0];
        point3d2.y = p[1];
        point3d2.z = p[2];
      }
      else
      {
        x = (j-xc2) / fx;
        y = (i-yc2) / fy;
        z = 1;
        float theta = -theta2;
        point3d.x = z*sin(theta) + x*cos(theta);
        point3d.y = y;
        point3d.z = z*cos(theta) - x*sin(theta);
        cv::Vec3d p(point3d.x,point3d.y,point3d.z);
        p = Rrect1.inv() * p;
        point3d1.x = p[0];
        point3d1.y = p[1];
        point3d1.z = p[2];
        point3d2.x = point3d.x;
        point3d2.y = point3d.y;
        point3d2.z = point3d.z;
      }
      cv::Point2f point2d1 = ocam_model1.World2Camera(point3d1);
      cv::Point2f point2d2 = ocam_model2.World2Camera(point3d2);

      int u1 = point2d1.x, v1 = point2d1.y;
      int u2 = point2d2.x, v2 = point2d2.y;
      if(u1 >= 0 && u1 < src1.cols && v1 >= 0 && v1 < src1.rows)
        dst1.at<cv::Vec3b>(i,j) = src1.at<cv::Vec3b>(v1,u1);
      if(u2 >= 0 && u2 < src2.cols && v2 >= 0 && v2 < src2.rows)
        dst2.at<cv::Vec3b>(i,j) = src2.at<cv::Vec3b>(v2,u2);
    }
}


void upraiseCamera(cv::Mat src, cv::Mat dst, cv::Vec3f euler, float fx, float fy, float xc, float yc, FisheyeParam ocam)
{
  int width = src.cols, height = src.rows;
  //float theta1 = -45*CV_PI/180, theta2 = 45*CV_PI/180;
  float theta1 = -0*CV_PI/180.0, theta2 = 0*CV_PI/180.0;

  cv::Matx33d Rz = Rotation_z(euler(2));
  cv::Matx33d Ry = Rotation_y(euler(1));
  cv::Matx33d Rx = Rotation_x(euler(0));

  float theta = CV_PI/2;
  cv::Matx33d Rrectx = Rotation_x(theta);
  cv::Matx33d Rrecty = Rotation_y(theta);
  cv::Matx33d Rrectz = Rotation_z(theta);

  cv::Matx33d Ryleft = Rotation_y(theta1);
  cv::Matx33d Ryright = Rotation_y(theta2);

  float fx1 = fx, fx2 = fx, fy1 = fy, fy2 = fy;
  cv::Vec3f center(0, 0, 1);
  cv::Vec3f center1 = Rotation_y(theta1).t() * center;
  cv::Vec3f center2 = Rotation_y(theta2).t() * center;
  float xc1 = xc + fx * center1(0)/center1(2), yc1 = yc + fy * center1(1)/center1(2);
  float xc2 = xc + fx * center2(0)/center2(2), yc2 = yc + fy * center2(1)/center2(2);

  for(int i = 0; i < height; i++)
    for(int j =0; j < width; j++)
    {
      cv::Vec3f world_point;
      if(j < xc)
      {
        world_point(0) = (j-xc1)/fx1;
        world_point(1) = (i-yc1)/fy1;
        world_point(2) = 1;
        world_point = Ryleft.t() * world_point;
      }
      else
      {
        world_point(0) = (j-xc2)/fx2;
        world_point(1) = (i-yc2)/fy2;
        world_point(2) = 1;
        world_point = Ryright.t() * world_point;
      }

      world_point = Rrectx.t() * world_point;
      world_point = Rx * world_point;

      cv::Point2f pix = ocam.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
      int u = pix.x, v = pix.y;
      if(u >=0 && u < width && v >= 0 && v < height)
        dst.at<cv::Vec3b>(i,j) = src.at<cv::Vec3b>(v,u);
    }
}

void upraiseCamera2(cv::Mat src, cv::Mat dst, cv::Vec3f euler, float fx, float fy, float xc, float yc, FisheyeParam ocam)
{
  int width = src.cols, height = src.rows;
  float theta1 = -45*CV_PI/180, theta2 = 45*CV_PI/180;

  cv::Matx33d Rz = Rotation_z(euler(2));
  cv::Matx33d Ry = Rotation_y(euler(1));
  cv::Matx33d Rx = Rotation_x(euler(0));

  float theta = CV_PI/2;
  cv::Matx33d Rrectx = Rotation_x(theta);

  cv::Vec3f center(0, 0, 1);
  cv::Vec3f center1 = Rotation_y(theta1).t() * center;
  cv::Vec3f center2 = Rotation_y(theta2).t() * center;
  float xc1 = xc + fx * center1(0)/center1(2), yc1 = yc + fy * center1(1)/center1(2);
  float xc2 = xc + fx * center2(0)/center2(2), yc2 = yc + fy * center2(1)/center2(2);

  for(int i = 0; i < height; i++)
    for(int j =0; j < width; j++)
    {
      cv::Point3f point3d = ocam.Camera2World(cv::Point2f(j,i));
      cv::Vec3f world_point(point3d.x, point3d.y, point3d.z);
      world_point = Rx.t() * world_point;
      world_point = Rrectx * world_point;
      int u, v;
      if(j < xc)
      {
        world_point = Rotation_y(theta1) * world_point;
        u = world_point(0)/world_point(2)*fx + xc1;
        v = world_point(1)/world_point(2)*fy + yc1;
      }
      else
      {
        world_point = Rotation_y(theta2) * world_point;
        u = world_point(0)/world_point(2)*fx + xc2;
        v = world_point(1)/world_point(2)*fy + yc2;
      }
      
      if(u >=0 && u < width && v >= 0 && v < height)
        dst.at<cv::Vec3b>(v,u) = src.at<cv::Vec3b>(i,j);
    }
}

void correctCameraWithCorrectRotation(cv::Mat src, cv::Mat dst, cv::Matx33d R, float fx, float fy, float xc, float yc, FisheyeParam ocam)
{
  int width = src.cols, height = src.rows;
  for(int i = 0; i < height; i++)
    for(int j =0; j < width; j++)
    {
      cv::Vec3f world_point;
      world_point(0) = (j-xc)/fx;
      world_point(1) = (i-yc)/fy;
      world_point(2) = 1;
      world_point = R.t() * world_point;
      cv::Point2f pix = ocam.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
      int u = pix.x, v = pix.y;
      if(u >=0 && u < width && v >= 0 && v < height)
        dst.at<cv::Vec3b>(i,j) = src.at<cv::Vec3b>(v,u);
    }
}

void Onchange(int, void*)
{
  initImages();

  //twoFisheye2twoPerspective(src1, src2, dst1, dst2, fx, fy, R_r, R_l, ocam_model1, ocam_model2);
  //Fisheye2Perspective(src1, result1, fx, fy, R_l, ocam_model1);
  //Fisheye2Perspective(src2, result2, fx, fy, R_r, ocam_model2);
  //Fisheye2twoPerspective(src1, result1, fx, fy, xc, yc,ocam_model1);
  //Fisheye2twoPerspective(src2, result2, fx, fy, xc, yc,ocam_model2);
  // upraiseCamera(src_left, result_left, euler1, fx, fy, xc, yc, ocam_model_left);
  // upraiseCamera(src_front, result_front, euler2, fx, fy, xc, yc, ocam_model_front);
  correctCameraWithCorrectRotation(src_left, dst_left, Rotation_left_r, fx, fy, xc, yc, ocam_model_left);
  correctCameraWithCorrectRotation(src_front, dst_front, Rotation_front_l, fx, fy, xc, yc, ocam_model_front);

  cv::imshow( "large image", large);
  // cv::imshow( "upraiseCamera1", result_left );
  // cv::imshow( "upraiseCamera2", result_front );
  // cv::imshow( "two perspective result1", dst_left );
  // cv::imshow( "two perspective result2", dst_front );
}

int main(int argc, char *argv[])
{
  Load();
  initImages();
  fx = 480, fy = 360, xc = dst_left.cols/2.0, yc = dst_left.rows/2.0;

  // euler1 = rotationMatrixToEulerAngles(Rotation_left);  //R为Rcw
  // euler2 = rotationMatrixToEulerAngles(Rotation_front);
  // upraiseCamera(src_left, result_left, euler1, fx, fy, xc, yc, ocam_model_left);
  // upraiseCamera(src_front, result_front, euler2, fx, fy, xc, yc, ocam_model_front);

  //twoFisheye2twoPerspective(src1, src2, dst1, dst2, fx, fy, R_r, R_l, ocam_model1, ocam_model2);

/*
  cv::Matx33d Rx1 = Rotation_x(euler1(0));
  cv::Matx33d Rx2 = Rotation_x(euler2(0));
  cv::Matx33d Rrect = Rotation_x(CV_PI/2);
  R1 = Rrect * Rx1.t() * R1;
  t1 = Rrect * Rx1.t() * t1;
  R2 = Rrect * Rx2.t() * R2;
  t2 = Rrect * Rx2.t() * t2;
  R = R2 * R1.t();
  t = t2 - R * t1;
  calculateTwoRotation(R, t, R_l, R_r);
  //Fisheye2Perspective(dst1, result1, fx, fy, R_l, ocam_model1);
  //Fisheye2Perspective(dst2, result2, fx, fy, R_r, ocam_model2);
  //Fisheye2twoPerspective(src1, result1, fx, fy, xc, yc, ocam_model1);
  //Fisheye2twoPerspective(src2, result2, fx, fy, xc, yc, ocam_model2);
  //calculateTwoRotation(R1, t1, R2, t2, euler1, euler2, R_l, R_r);
  //calculateTwoRotation(R1, t1, R2, t2, R_l, R_r);
  R_l = R_l * Rrect * Rx1.t();
  R_r = R_r * Rrect * Rx2.t();
*/

  calculateTwoCorrectRotation(Rotation_left, Translation_left, Rotation_front, Translation_front, Rotation_left_r, Rotation_front_l);
  correctCameraWithCorrectRotation(src_left, dst_left, Rotation_left_r, fx, fy, xc, yc, ocam_model_left);
  correctCameraWithCorrectRotation(src_front, dst_front, Rotation_front_l, fx, fy, xc, yc, ocam_model_front);
  cv::Mat ROI = large(cv::Rect(0, 0, dst_left.cols, dst_left.rows));
  dst_left.copyTo(ROI);
  ROI = large(cv::Rect(dst_left.cols, 0, dst_front.cols, dst_front.rows));
  dst_front.copyTo(ROI);
  for(int i = 0; i < large.rows; i += 20)
  {
    cv::Scalar s1(255, 0, 0), s2(0, 255, 0), s3(0, 0, 255);
    if(i / 20 % 2)
      cv::line(large, cv::Point(0, i), cv::Point(large.cols, i), s1);
    else
      cv::line(large, cv::Point(0, i), cv::Point(large.cols, i), s2);
      
  }

  cv::namedWindow( "large image", 0 );
  cv::namedWindow( "Original fisheye camera image1", 0 );
  cv::namedWindow( "Original fisheye camera image2", 0 );
  // cv::namedWindow( "upraiseCamera1", 0 );
  // cv::namedWindow( "upraiseCamera2", 0 );
  // cv::namedWindow( "two perspective result1", 0 );
  // cv::namedWindow( "two perspective result2", 0 );
  cv::namedWindow( "toolbox", 0 );

  cv::imshow( "large image", large);
  cv::imshow( "Original fisheye camera image1", src_left );
  cv::imshow( "Original fisheye camera image2", src_front );
  // cv::imshow( "upraiseCamera1", result_left );
  // cv::imshow( "upraiseCamera2", result_front );
  // cv::imshow( "two perspective result1", dst_left );
  // cv::imshow( "two perspective result2", dst_front );

  cv::createTrackbar("fx", "toolbox", &fx, slider_max, Onchange);
  cv::createTrackbar("fy", "toolbox", &fy, slider_max, Onchange);
  cv::createTrackbar("xc", "toolbox", &xc, slider_max, Onchange);
  cv::createTrackbar("yc", "toolbox", &yc, slider_max, Onchange);

  /* --------------------------------------------------------------------*/
  /* Wait until Key 'q' pressed                                         */
  /* --------------------------------------------------------------------*/
  while((char)cv::waitKey(10) != 'q');
 
  /* --------------------------------------------------------------------*/
  /* Save image                                                          */
  /* --------------------------------------------------------------------*/
  // cv::imwrite("upraiseCamera1.jpg", dst_left);
  // printf("\nImage %s saved\n","upraiseCamera1.jpg");
  // cv::imwrite("upraiseCamera2.jpg", dst_front);
  // printf("\nImage %s saved\n","upraiseCamera2.jpg");
  // printf("\nImage %s saved\n","result1.jpg");
  // cv::imwrite("result1.jpg", dst_left);
  // printf("\nImage %s saved\n","result2.jpg");
  // cv::imwrite("result2.jpg", dst_front);
  cv::imwrite("large.jpg", large);
  printf("\nImage %s saved\n","large.jpg");

  //cv::destroyAllWindows();

  return 0;
}
