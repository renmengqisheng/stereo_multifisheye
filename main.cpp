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
cv::Mat src1, dst1, src2, dst2;
cv::Mat mapx1, mapy1;
cv::Mat mapx2, mapy2;
cv::Mat K1, K2;
int fx, fy;
int xc, yc;
FisheyeParam ocam_model1, ocam_model2;
float theta1 = 45*CV_PI/180, theta2 = -45*CV_PI/180;
cv::Matx33d R1, R2, R, R_l, R_r;
cv::Vec3d t1, t2, t;

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
}

void calculateTwoRotation(cv::Matx33d R, cv::Vec3d tvec, cv::Matx33d& R1, cv::Matx33d& R2)
{
  cv::Vec3d rvec = cv::Affine3d(R).rvec();

  
  // rectification algorithm
  rvec *= -0.5;              // get average rotation

  cv::Matx33d r_r;
  Rodrigues(rvec, r_r);  // rotate cameras to same orientation by averaging

/*
  double nt = norm(tvec);
  if(nt > 0.0)
    tvec /= nt;
  cv::Vec3d uu(0, 0, tvec(0) > 0 ? 1 : -1);
  cv::Vec3d ww = tvec.cross(uu);
  double nw = norm(ww);
  if (nw > 0.0)
    ww /= nw;
  R1 = cv::Matx33d(tvec(0), tvec(1), tvec(2), uu(0), uu(1), uu(2), ww(0), ww(1), ww(2));
  R2 = R * R1;
*/
  
  //std::cout << "origin t: " << tvec << std::endl;
  cv::Vec3d t = r_r * tvec;
  //std::cout << "rotate t: " << t << std::endl;
  cv::Vec3d uu(t[0] > 0 ? 1 : -1, 0, 0);

  // calculate global Z rotation
  cv::Vec3d ww = t.cross(uu);
  double nw = norm(ww);
  if (nw > 0.0)
      ww *= acos(fabs(t[0])/cv::norm(t))/nw;

  cv::Matx33d wr;
  Rodrigues(ww, wr);

  // apply to both views
  cv::Matx33d ri1 = wr * r_r.t();
  cv::Mat(ri1, false).convertTo(R1, CV_64F);
  cv::Matx33d ri2 = wr * r_r;
  cv::Mat(ri2, false).convertTo(R2, CV_64F);
  
}

void Fisheye2twoPerspective(cv::Mat src, cv::Mat dst, float fx1, float fy1, float fx2, float fy2, cv::Matx33d R1, cv::Matx33d R2, FisheyeParam& ocam_model)
{
  if(src.empty() || dst.empty())
  {
    std::cout << __FILE__ << ": "<< __LINE__ << " error!" << std::endl;
    return;
  }

  cv::Size image_size = dst.size();
      
  float xc1 = xc - fx1 * tan((theta1-theta2)/2), yc1 = yc;
  float xc2 = xc - fx2 * tan(-(theta1-theta2)/2), yc2 = yc;
  
  for(int i = 0; i < image_size.height; i++)
    for(int j = 0; j < image_size.width; j++)
    {
      cv::Point3f point3d;
      float x, y, z;
      if(j <= xc )
      {
        x = (j-xc1) / fx1;
        y = (i-yc1) / fy1;
        z = 1;
        float theta = -theta1;
        point3d.x = z*sin(theta) + x*cos(theta);
        point3d.y = y;
        point3d.z = z*cos(theta) - x*sin(theta);
      }
      else
      {
        x = (j-xc2) / fx2;
        y = (i-yc2) / fy2;
        z = 1;
        float theta = -theta2;
        point3d.x = z*sin(theta) + x*cos(theta);
        point3d.y = y;
        point3d.z = z*cos(theta) - x*sin(theta);
      }
      cv::Point2f point2d = ocam_model.World2Camera(point3d);
      int u = point2d.x, v = point2d.y;
      if(u >= 0 && u < src.cols && v >= 0 && v < src.rows)
        dst.at<cv::Vec3b>(i,j) = src.at<cv::Vec3b>(v,u);
    }
}

void Fisheye2Perspective(cv::Mat src, cv::Mat dst, float fx, float fy, cv::Matx33d Rrect, FisheyeParam& ocam_model)
{
  if(src.empty() || dst.empty())
  {
    std::cout << __FILE__ << ": "<< __LINE__ << " error!" << std::endl;
    return;
  }

  cv::Size image_size = dst.size();
  
  float theta1 = 45*CV_PI/180, theta2 = -45*CV_PI/180;
  float xc1 = xc - fx * tan(45*CV_PI/180), yc1 = yc;
  float xc2 = xc - fx * tan(-45*CV_PI/180), yc2 = yc;
  
  for(int i = 0; i < image_size.height; i++)
    for(int j = 0; j < image_size.width; j++)
    {
      cv::Point3f point3d;
      float x, y, z;

      
      x = (j - xc) / fx;
      y = (i - yc) / fy;
      z = 1;
      cv::Vec3d p(x,y,z);

      /*
      if(j <= xc)
      {
        x = (j-xc1) / fx;
        y = (i-yc1) / fy;
        z = 1;
        float theta = -theta1;
        point3d.x = z*sin(theta) + x*cos(theta);
        point3d.y = y;
        point3d.z = z*cos(theta) - x*sin(theta);
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
      }
      cv::Vec3d p(point3d.x,point3d.y,point3d.z);
      */
      p = Rrect.inv() * p;
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
        //cv::Vec3d p(x,y,z);
        p = Rrect1.inv() * p;
        point3d1.x = p[0];
        point3d1.y = p[1];
        point3d1.z = p[2];
        point3d2.x = point3d.x;
        point3d2.y = point3d.y;
        point3d2.z = point3d.z;
        //point3d2.x = x;
        //point3d2.y = y;
        //point3d2.z = z;
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

void Onchange(int, void*)
{
  K1 = (cv::Mat_<float>(3, 3) << fx,  0, xc,
                                 0, fy, yc,
                                 0,  0, 1);
  dst1 = cv::Mat(src1.rows, src1.cols, src1.type(), cv::Scalar::all(0));
  dst2 = cv::Mat(src2.rows, src2.cols, src2.type(), cv::Scalar::all(0));
  Fisheye2Perspective(src1, dst1, fx, fy, R_r, ocam_model1);
  Fisheye2Perspective(src2, dst2, fx, fy, R_l, ocam_model2);
  //twoFisheye2twoPerspective(src1, src2, dst1, dst2, fx, fy, R_r, R_l, ocam_model1, ocam_model2);
  cv::imshow( "Undistorted Perspective Image1", dst1 );
  cv::imshow( "Undistorted Perspective Image2", dst2 );
}

int main(int argc, char *argv[])
{
  /* --------------------------------------------------------------------*/
  /* Read the parameters of the omnidirectional camera from the TXT file */
  /* --------------------------------------------------------------------*/
  ocam_model1.Load("../intrinsic_parameters/left/calib.txt");
  ocam_model2.Load("../intrinsic_parameters/front/calib.txt");

  /* --------------------------------------------------------------------*/
  /* Allocate space for the unistorted images                            */
  /* --------------------------------------------------------------------*/
  src1 = cv::imread("../bmp/frame_vc9_1814.bmp");      // source image 1
  assert(!src1.empty());
  src2 = cv::imread("../bmp/frame_vc10_1814.bmp");      // source image 1
  assert(!src2.empty());

  //cv::Matx44d T1, T2;
  readTransformation("../result/left.txt", R1, t1);
  readTransformation("../result/front.txt", R2, t2);
  //mapx1 = cv::Mat(src1.rows, src1.cols, CV_32FC1);
  //mapy1 = cv::Mat(src1.rows, src1.cols, CV_32FC1);

  dst1 = cv::Mat(src1.rows, src1.cols, src1.type(), cv::Scalar::all(0));    // undistorted panoramic image
  dst2 = cv::Mat(src2.rows, src2.cols, src2.type(), cv::Scalar::all(0));    // undistorted panoramic image

  /* --------------------------------------------------------------------  */
  /* Create LooK1-Up-Table for perspective undistortion                     */
  /* SF is K1ind of distance from the undistorted image to the camera       */
  /* (it is not meters, it is justa zoom fator)                            */
  /* Try to change SF to see how it affects the result                     */
  /* The undistortion is done on a  plane perpendicular to the camera axis */
  /* --------------------------------------------------------------------  */
  fx = 480, fy = 360, xc = dst1.cols/2.0, yc = dst1.rows/2.0;
  //K1 = (cv::Mat_<float>(3, 3) << fx,  0, xc,
  //                               0, fy, yc,
  //                               0,  0, 1);

  R = R2 * R1.t();
  t = t2 - R * t1;
  calculateTwoRotation(R, t, R_l, R_r);
  Fisheye2Perspective(src1, dst1, fx, fy, R_r, ocam_model1);
  Fisheye2Perspective(src2, dst2, fx, fy, R_l, ocam_model2);
  //twoFisheye2twoPerspective(src1, src2, dst1, dst2, fx, fy, R_r, R_l, ocam_model1, ocam_model2);

  
  cv::namedWindow( "Original fisheye camera image1", 0 );
  cv::namedWindow( "Undistorted Perspective Image1", 0 );
  cv::namedWindow( "Original fisheye camera image2", 0 );
  cv::namedWindow( "Undistorted Perspective Image2", 0 );
  cv::namedWindow( "toolbox", 0 );

  cv::imshow( "Original fisheye camera image1", src1 );
  cv::imshow( "Undistorted Perspective Image1", dst1 );
  cv::imshow( "Original fisheye camera image2", src2 );
  cv::imshow( "Undistorted Perspective Image2", dst2 );

  cv::createTrackbar("fx", "toolbox", &fx, slider_max, Onchange);
  cv::createTrackbar("fy", "toolbox", &fy, slider_max, Onchange);
  cv::createTrackbar("xc", "toolbox", &xc, slider_max, Onchange);
  cv::createTrackbar("yc", "toolbox", &yc, slider_max, Onchange);

  /* --------------------------------------------------------------------*/
  /* Wait until K1ey 'q' pressed                                         */
  /* --------------------------------------------------------------------*/
  while((char)cv::waitKey(10) != 'q');
 
  /* --------------------------------------------------------------------*/
  /* Save image                                                          */
  /* --------------------------------------------------------------------*/
  cv::imwrite("undistorted_perspective1.jpg", dst1);
  printf("\nImage %s saved\n","undistorted_perspective1.jpg");
  cv::imwrite("undistorted_perspective2.jpg", dst2);
  printf("\nImage %s saved\n","undistorted_perspective2.jpg");

  cv::destroyAllWindows();

  return 0;
}
