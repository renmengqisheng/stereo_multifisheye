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
#include "Frame.h"
#include "ORBmatcher.h"
#include "multi_fisheye_param.h"

#include <iostream>
#include <string>

const int slider_max = 5000;
double fx, fy, xc, yc;
cv::Mat large, large1, large2, large3, large4;
cv::Mat src_left, src_front, src_right, src_back;
cv::Mat dst_left, dst_front, dst_right, dst_back;
cv::Mat result_left, result_front, result_right, result_back;

cv::Mat topview_full;

cv::Mat mapx_left, mapx_front, mapx_right, mapx_back;
cv::Mat mapy_left, mapy_front, mapy_right, mapy_back;

FisheyeParam ocam_model_left, ocam_model_front, ocam_model_right, ocam_model_back;

cv::Matx33d Rotation_left, Rotation_front, Rotation_right, Rotation_back;
cv::Vec3d Translation_left, Translation_front, Translation_right, Translation_back;

cv::Matx33d Rotation_left_ideal, Rotation_front_ideal, Rotation_right_ideal, Rotation_back_ideal;
cv::Vec3d Translation_left_ideal, Translation_front_ideal, Translation_right_ideal, Translation_back_ideal;

cv::Matx33d Rotation_left_l, Rotation_left_r, Rotation_front_l, Rotation_front_r;
cv::Matx33d Rotation_right_l, Rotation_right_r, Rotation_back_l, Rotation_back_r;

cv::Matx33d Rrect;

std::vector<cv::DMatch> matches;

const int ROWS = 800;
const int COLS = 800;

multi_fisheye_param multi;

//坐标轴旋转
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

  // translateTwc2Tcw(R, t);
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

  // cv::cvtColor(src_left, dst_left, CV_BGR2GRAY);
  // cv::cvtColor(src_front, dst_front, CV_BGR2GRAY);
  // cv::cvtColor(src_right, dst_right, CV_BGR2GRAY);
  // cv::cvtColor(src_back, dst_back, CV_BGR2GRAY);

  readTransformation("../result/left.txt", Rotation_left, Translation_left);  //Rcw, tcw
  readTransformation("../result/front.txt", Rotation_front, Translation_front);
  readTransformation("../result/right.txt", Rotation_right, Translation_right);
  readTransformation("../result/back.txt", Rotation_back, Translation_back);
}

void initImages()
{
  // topview_full = cv::Mat(src_left.rows, src_left.cols, src_left.type(), cv::Scalar::all(0));

  // large1 = cv::Mat(src_left.rows, src_left.cols*4, src_left.type(), cv::Scalar::all(0));
  // large2 = cv::Mat(src_left.rows, src_left.cols*4, src_left.type(), cv::Scalar::all(0));
  // large3 = cv::Mat(src_left.rows, src_left.cols*4, src_left.type(), cv::Scalar::all(0));
  // large4 = cv::Mat(src_left.rows, src_left.cols*4, src_left.type(), cv::Scalar::all(0));

  // dst_left = cv::Mat(src_left.rows, src_left.cols, src_left.type(), cv::Scalar::all(0));
  // dst_front = cv::Mat(src_front.rows, src_front.cols, src_front.type(), cv::Scalar::all(0));
  // dst_right = cv::Mat(src_right.rows, src_right.cols, src_right.type(), cv::Scalar::all(0));
  // dst_back = cv::Mat(src_back.rows, src_back.cols, src_back.type(), cv::Scalar::all(0));

  // result_left = cv::Mat(src_left.rows, src_left.cols, src_left.type(), cv::Scalar::all(0));
  // result_front = cv::Mat(src_front.rows, src_front.cols, src_front.type(), cv::Scalar::all(0));
  // result_right = cv::Mat(src_right.rows, src_right.cols, src_right.type(), cv::Scalar::all(0));
  // result_back = cv::Mat(src_back.rows, src_back.cols, src_back.type(), cv::Scalar::all(0));

  // mapx_left = cv::Mat(src_left.rows, src_left.cols, CV_32FC1, cv::Scalar::all(0));
  // mapx_front = cv::Mat(src_front.rows, src_front.cols, CV_32FC1, cv::Scalar::all(0));
  // mapx_right = cv::Mat(src_right.rows, src_right.cols, CV_32FC1, cv::Scalar::all(0));
  // mapx_back = cv::Mat(src_back.rows, src_back.cols, CV_32FC1, cv::Scalar::all(0));

  // mapy_left = cv::Mat(src_left.rows, src_left.cols, CV_32FC1, cv::Scalar::all(0));
  // mapy_front = cv::Mat(src_front.rows, src_front.cols, CV_32FC1, cv::Scalar::all(0));
  // mapy_right = cv::Mat(src_right.rows, src_right.cols, CV_32FC1, cv::Scalar::all(0));
  // mapy_back = cv::Mat(src_back.rows, src_back.cols, CV_32FC1, cv::Scalar::all(0));


  large = cv::Mat(ROWS, COLS*8, src_left.type(), cv::Scalar::all(0));

  large1 = cv::Mat(ROWS, COLS*4, src_left.type(), cv::Scalar::all(0));
  large2 = cv::Mat(ROWS, COLS*4, src_left.type(), cv::Scalar::all(0));
  large3 = cv::Mat(ROWS, COLS*4, src_left.type(), cv::Scalar::all(0));
  large4 = cv::Mat(ROWS, COLS*4, src_left.type(), cv::Scalar::all(0));

  mapx_left = cv::Mat(ROWS, COLS*2, CV_32FC1, cv::Scalar::all(0));
  mapx_front = cv::Mat(ROWS, COLS*2, CV_32FC1, cv::Scalar::all(0));
  mapx_right = cv::Mat(ROWS, COLS*2, CV_32FC1, cv::Scalar::all(0));
  mapx_back = cv::Mat(ROWS, COLS*2, CV_32FC1, cv::Scalar::all(0));

  mapy_left = cv::Mat(ROWS, COLS*2, CV_32FC1, cv::Scalar::all(0));
  mapy_front = cv::Mat(ROWS, COLS*2, CV_32FC1, cv::Scalar::all(0));
  mapy_right = cv::Mat(ROWS, COLS*2, CV_32FC1, cv::Scalar::all(0));
  mapy_back = cv::Mat(ROWS, COLS*2, CV_32FC1, cv::Scalar::all(0));


}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(const cv::Matx33d &R)
{
    cv::Matx33d Rt = R.t();
    cv::Matx33d shouldBeIdentity = Rt * R;
    cv::Matx33d I = cv::Matx33d::eye();
 
    return  norm(I, shouldBeIdentity) < 1e-5;
}

//R1, R2为Rcw
void calculateTwoCorrectRotation2(const cv::Matx33d& R1, const cv::Vec3d& t1, const cv::Matx33d& R2, const cv::Vec3d& t2, cv::Matx33d& R_l, cv::Matx33d& R_r, double& baseline)
{
  cv::Matx33d R = R2 * R1.t();  //Rcw,tcw
  cv::Vec3d tvec = R2 * (t1 - t2);

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
      ww *= acos(fabs(t[0])/cv::norm(t))/nw;  //求向量t和uu的夹角Θ，绕向量ww旋转Θ

  cv::Matx33d wr;
  Rodrigues(ww, wr);

  // apply to both views
  R_l = wr * r_r.t();
  R_r = wr * r_r;
  cv::Vec3d tnew = R_r * tvec;
  baseline = fabs(tnew[0]);
}


void correctCamera_LUT2(cv::Mat mapx, cv::Mat mapy, cv::Matx33d R_l, cv::Matx33d R_r, cv::Matx33d R_ideal, double fx, double fy, double xc, double yc, FisheyeParam& ocam, double &xc1, double &xc2)
{
  int width = mapx.cols, height = mapx.rows;
  cv::Vec3d center(0, 0, 1);

  cv::Vec3d center1 = R_l.t() * center, center2 = R_r.t() * center;
  xc1 = xc + fx * center1(0)/center1(2);
  xc2 = xc + fx * center2(0)/center2(2);
  float yc1 = yc + fy * center1(1)/center1(2);
  float yc2 = yc + fy * center2(1)/center2(2);

  for(int i = 0; i < height; i++)
    for(int j =0; j < width; j++)
    {
      cv::Vec3d world_point;
      if(j < xc)
      {
        world_point(0) = (j-xc1)/fx;
        world_point(1) = (i-yc1)/fy;
        world_point(2) = 1;
        world_point = R_l.t() * world_point;
      }
      else
      {
        world_point(0) = (j-xc2)/fx;
        world_point(1) = (i-yc2)/fy;
        world_point(2) = 1;
        world_point = R_r.t() * world_point;
      }
      
      world_point = R_ideal.t() * world_point;

      cv::Point2f point2d = ocam.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
      mapx.at<float>(i,j) = point2d.x;
      mapy.at<float>(i,j) = point2d.y;
    }
}


void mergeImagesandDrawLines(cv::Mat img1, cv::Mat img2, cv::Mat img3, cv::Mat img4, cv::Mat& dst)
{
  cv::Mat ROI = dst(cv::Rect(0, 0, img1.cols, img2.rows));
  img1.copyTo(ROI);
  ROI = dst(cv::Rect(img1.cols, 0, img2.cols, img2.rows));
  img2.copyTo(ROI);
  ROI = dst(cv::Rect(img1.cols+img2.cols, 0, img3.cols, img3.rows));
  img3.copyTo(ROI);
  ROI = dst(cv::Rect(img1.cols+img2.cols+img3.cols, 0, img4.cols, img4.rows));
  img4.copyTo(ROI);
  int count = 0;
  for(int i = 0; i < dst.rows; i += 20)
  {
    cv::Scalar s1(255, 0, 0), s2(0, 255, 0), s3(0, 0, 255);
    if(count == 0)
    {
      cv::line(dst, cv::Point(0, i), cv::Point(dst.cols, i), s1);
      ++count;
    }
    else if(count == 1)
    {
      cv::line(dst, cv::Point(0, i), cv::Point(dst.cols, i), s2);
      ++count;
    }
    else
    {
      cv::line(dst, cv::Point(0, i), cv::Point(dst.cols, i), s3);
      count = 0;
    }
  }
}

float ZNCC(cv::Mat img1, cv::Mat img2, cv::Point2i point1, cv::Point2i point2, int size)
{
	if (img1.empty() || img1.channels() != 1 || img2.empty() || img2.channels() != 1)
	{
		std::cout << "Image error in ZNCC!" << std::endl;
		return 0;
	}

	float diff = 0;
	float l_avg = 0, r_avg = 0;
	int size2 = (2 * size + 1)*(2 * size + 1);
	if (point1.x - size >= 0 && point1.x + size < img1.cols && point1.y - size >= 0 && point1.y + size < img1.rows 
      && point2.x - size >= 0 && point2.x + size < img2.cols && point2.y - size >= 0 && point2.y + size < img2.rows)
	{
		for (int i = -size; i <= size; i++)
		{
			for (int j = -size; j <= size; j++)
			{
				{
					float v1 = img1.ptr<uchar>(point1.y + i)[point1.x + j];
					float v2 = img2.ptr<uchar>(point2.y + i)[point2.x + j];
					l_avg = l_avg + v1 / size2;
					r_avg = r_avg + v2 / size2;
				}
			}
		}
 
		float lr = 0, ll = 0, rr = 0;//这些是自相关和互相关
		for (int i = -size; i <= size; i++)
		{
			for (int j = -size; j <= size; j++)
			{
				float v1 = img1.ptr<uchar>(point1.y + i)[point1.x + j];
				float v2 = img2.ptr<uchar>(point2.y + i)[point2.x + j];
				lr += (v1 - l_avg)*(v2 - r_avg);
				ll += (v1 - l_avg)*(v1 - l_avg);
				rr += (v2 - r_avg)*(v2 - r_avg);
			}
		}
 
		diff = fabs(lr / sqrt(ll*rr));
	}
  
	return diff;
}

void calculateMatches(cv::Mat img1, cv::Mat img2, std::vector<cv::DMatch>& matches, cv::Mat& large)
{
  std::vector<cv::KeyPoint> kps1, kps2;
  //cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
  ORB_SLAM2::ORBextractor orb(2000, 1.2f, 8, 31, 8);

  // cv::Rect roi1 = cv::Rect(img1.cols/2, 0, img1.cols/2, img1.rows);
  // cv::Rect roi2 = cv::Rect(0, 0, img2.cols/2, img2.rows);
  // cv::Mat mask1 = cv::Mat::zeros(img1.size(), CV_8UC1);
  // mask1(roi1).setTo(255);
  // cv::Mat mask2 = cv::Mat::zeros(img2.size(), CV_8UC1);
  // mask2(roi2).setTo(255);

  cv::Mat dst1, dst2;
  cv::Mat desp1, desp2;
  // img1.copyTo(dst1, mask1);
  // img2.copyTo(dst2, mask2);
  img1.copyTo(dst1);
  img2.copyTo(dst2);

  //imshow("test1", dst1);
  //imshow("test2", dst2);

/*
  orb->detect(dst1, kps1);
  orb->detect(dst2, kps2);
  
  orb->compute(dst1, kps1, desp1);
  orb->compute(dst2, kps2, desp2);
*/
  cv::cvtColor(dst1, dst1, CV_BGR2GRAY);
  cv::cvtColor(dst2, dst2, CV_BGR2GRAY);
  orb(dst1, cv::Mat(), kps1, desp1);
  orb(dst2, cv::Mat(), kps2, desp2);

  //std::cout << desp1 << std::endl;
  //desp1.convertTo(desp1, CV_32F);
  //desp2.convertTo(desp2, CV_32F);

  //cv::FlannBasedMatcher matcher;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  //cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
  matcher.match(desp1, desp2, matches);
  //cv::drawMatches(img1, kps1, img2, kps2, matches, large);

  //cv::Mat dst1, dst2;
  // cv::cvtColor(img1, dst1, CV_BGR2GRAY);
  // cv::cvtColor(img2, dst2, CV_BGR2GRAY);

  std::vector<cv::DMatch> goodMatches;
  for (int i = 0; i < matches.size(); i++)
  {
      cv::Point2f pt1 = kps1[matches[i].queryIdx].pt;
      cv::Point2f pt2 = kps2[matches[i].trainIdx].pt;
      float zncc = ZNCC(dst1, dst2, pt1, pt2, 15);
      //std::cout << zncc << std::endl;
      if (fabs(pt1.y-pt2.y) < 5 && pt1.x > pt2.x && zncc > 0.8)
      {
          goodMatches.push_back(matches[i]);
      }
  }

  if(goodMatches.empty())
  {
    std::cout << "goodMatches is empty. " << std::endl;
    return;
  }

  std::cout << "goodMatches size: " << goodMatches.size() << std::endl;
  cv::drawMatches(img1, kps1, img2, kps2, goodMatches, large);

}


void ComputeStereoMatches(ORB_SLAM2::Frame& frame1, ORB_SLAM2::Frame& frame2, const float& xc1R, const float& xc2L)
{
    // std::cout << std::endl;
    // std::cout << "Nl: " << frame1.mvKeysUn.size() << ", Nr: " << frame2.mvKeysUn.size() << std::endl;

    clock_t start = clock();


    const int thOrbDist = (ORB_SLAM2::ORBmatcher::TH_HIGH+ORB_SLAM2::ORBmatcher::TH_LOW)/2;

    const int nRows = frame1.mpORBextractorLeft->mvImagePyramid[0].rows;
    const int nCols = frame2.mpORBextractorLeft->mvImagePyramid[0].cols;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = frame2.mvKeysUn.size();

    // clock_t start = clock();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = frame2.mvKeysUn[iR];

        //新增加，只有右目的左半区域进行双目匹配，保证正深度
        if(kp.pt.x >= nCols/2.0)
            continue;

        const float &kpY = kp.pt.y;
        const float r = 2.0f*frame2.mvScaleFactors[frame2.mvKeysUn[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // clock_t end = clock();
    // std::cout << "table cost: " << (double)(end-start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;

    clock_t zncc_sum = 0, slide_sum = 0, candidate_sum = 0;


    // Set limits for search
    const float minZ = frame1.mb;
    // const float minZ = 0.001;
    // std::cout << "minZ: " << minZ << std::endl;
    const float minD = 0;
    // const float maxD = nCols/2.0;
    const float maxD = frame1.mbf/minZ;
    // std::cout << "maxD: " << maxD << std::endl;

    // std::cout << "minZ: " << minZ << ", mbf: " << frame1.mbf 
    //             << ", maxD: " << maxD << std::endl;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(frame1.N);
    frame1.mvMatch12 = vector<int>(frame1.N, -1);

    for(int iL=0; iL<frame1.N; iL++)
    {
        const cv::KeyPoint &kpL = frame1.mvKeysUn[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        //新增加，只有左目的右半区域进行双目匹配
        if(uL <= nCols/2.0)
            continue;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORB_SLAM2::ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = frame1.mDescriptors.row(iL);



        // clock_t candidate_start = clock();

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = frame2.mvKeysUn[iR];

            //修改，根据关键点尺度差异进行筛选
            if(fabs(levelL - kpR.octave) > 3)
                continue;

            //新增加，根据关键点方向差异进行筛选
            if(fabs(kpL.angle - kpR.angle) > 15)
                continue;

            const float &uR = kpR.pt.x;


            if(uR-xc2L+xc1R>=minU && uR-xc2L+xc1R<=maxU)
            {
                const cv::Mat &dR = frame2.mDescriptors.row(iR);
                const int dist = ORB_SLAM2::ORBmatcher::DescriptorDistance(dL,dR);
                
                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // clock_t candidate_end = clock();
        // candidate_sum += candidate_end - candidate_start;
        // std::cout << "candidate cost: " << (double)(candidate_end-candidate_start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;



        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {

            //新增加，通过关键点尺度差异和方向差异筛选
            const cv::KeyPoint &kpR = frame2.mvKeysUn[bestIdxR];

            // // 新增加，根据关键点方向差异进行筛选
            // if(fabs(kpL.angle - kpR.angle) > 15)
            //     continue;
            // // 新增加，根据关键点尺度差异进行筛选
            // if(fabs(kpL.octave - kpR.octave) > 3)
            //     continue;

            // coordinates in image pyramid at keypoint scale
            const float uR0 = frame2.mvKeysUn[bestIdxR].pt.x;
            const float scaleFactor = frame1.mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);


            //新增加，通过ZNCC进行筛选
            const int size = 5;
            const float scaleFactorR = frame2.mvInvScaleFactors[kpR.octave];
            const float scaleduR = round(kpR.pt.x*scaleFactorR);
            const float scaledvR = round(kpR.pt.y*scaleFactorR);
            const cv::Mat &imgL = frame1.mpORBextractorLeft->mvImagePyramid[kpL.octave];//.rowRange(scaledvL-12*w,scaledvL+12*w+1).colRange(scaleduL-12*w,scaleduL+12*w+1);
            const cv::Mat &imgR = frame2.mpORBextractorLeft->mvImagePyramid[kpR.octave];//.rowRange(scaledvR-12*w,scaledvR+12*w+1).colRange(scaleduR-12*w,scaleduR+12*w+1);
            
            
            // clock_t start = clock();
            
            float zncc = ZNCC(imgL, imgR, cv::Point2i(scaleduL,scaledvL), cv::Point2i(scaleduR,scaledvR), size);
            
            
            // clock_t end = clock();
            // zncc_sum += end - start;
            // std::cout << "zncc cost: " << (double)(end-start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;
            // std::cout << "zncc: " << zncc << std::endl;
            
            if(zncc <= 0.85)
                continue;



            // start = clock();

            // sliding window search
            const int w = 5;
            cv::Mat IL = frame1.mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= frame2.mpORBextractorLeft->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = frame2.mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));


            // end = clock();
            // slide_sum += end - start;
            // std::cout << "slide cost: " << (double)(end-start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;


            if(deltaR<-1 || deltaR>1)
                continue;
                
            // Re-scaled coordinate
            float bestuR = frame1.mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);
            // float disparity = (uL-bestuR);
            float disparity = (uL-xc1R) - (bestuR-xc2L);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                frame1.mvuRight[iL] = bestuR;
                frame1.mvDepth[iL]=frame1.mbf/disparity;
                frame2.mvDepth[bestIdxR]=frame1.mvDepth[iL];
                frame1.mvMatch12[iL] = bestIdxR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    // std::cout << "zncc sum is: " << (double)zncc_sum / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;
    // std::cout << "slide sum is: " << (double)slide_sum / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;
    // std::cout << "candidate sum is: " << (double)candidate_sum / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;

    // clock_t sort_start = clock();

    sort(vDistIdx.begin(),vDistIdx.end());

    // clock_t sort_end = clock();
    // std::cout << "sort cost: " << (double)(sort_end-sort_start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;

    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;
    // const float thDist = median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            frame1.mvuRight[vDistIdx[i].second]=-1;
            frame1.mvDepth[vDistIdx[i].second]=-1;
            frame2.mvDepth[frame1.mvMatch12[vDistIdx[i].second]]=-1;
            frame1.mvMatch12[vDistIdx[i].second] = -1;
        }
    }

    // //测试代码
    // const cv::Mat &img1 = frame1.mpORBextractorLeft->mvImagePyramid[0];
    // const cv::Mat &img2 = frame2.mpORBextractorLeft->mvImagePyramid[0];
    // for(int i = 0; i < frame1.N; i++)
    // {
    //     if(frame1.mvMatch12[i] > 0)
    //     {

    //         const cv::KeyPoint &kpL = frame1.mvKeysUn[i];
    //         const cv::KeyPoint &kpR = frame2.mvKeysUn[frame1.mvMatch12[i]];
    //         const int size = 4;
    //         float xL = kpL.pt.x, yL = kpL.pt.y, xR = kpR.pt.x, yR = kpR.pt.y;
            
    //         cv::Mat dst1 = img1.rowRange(yL-10*size,yL+10*size+1).colRange(xL-10*size,xL+10*size+1).clone();
    //         cv::line(dst1, cv::Point(10*size,0), cv::Point(10*size, 20*size), cv::Scalar(0,0,255));
    //         cv::line(dst1, cv::Point(0,10*size), cv::Point(20*size, 10*size), cv::Scalar(0,0,255));
    //         cv::Mat dst2 = img2.rowRange(yR-10*size,yR+10*size+1).colRange(xR-10*size,xR+10*size+1).clone();
    //         cv::line(dst2, cv::Point(10*size,0), cv::Point(10*size, 20*size), cv::Scalar(0,0,255));
    //         cv::line(dst2, cv::Point(0,10*size), cv::Point(20*size, 10*size), cv::Scalar(0,0,255));
            
    //         cv::imshow("img1", img1);
    //         cv::imshow("img2", img2);
    //         cv::imshow("look1", dst1);
    //         cv::imshow("look2", dst2);
    //         cv::waitKey(0);
    //     }
    // }
}


void topview(cv::Mat& output, cv::Mat left, cv::Mat front, cv::Mat right, cv::Mat back, 
             cv::Matx33d R_l, cv::Vec3f t_l, cv::Matx33d R_f, cv::Vec3f t_f, cv::Matx33d R_r, cv::Vec3f t_r, cv::Matx33d R_b, cv::Vec3f t_b, 
             FisheyeParam& ocam_left, FisheyeParam& ocam_front, FisheyeParam& ocam_right, FisheyeParam& ocam_back)
{
  int width = output.cols;
  int height = output.rows;
  float k = (float)height / width;
  float scale = 38;
  float fx = scale, fy = scale*k;
  float theta = 36 * CV_PI / 180;
  cv::Point2f point2d;
  for(int i = 0; i < height; i++)
    for(int j = 0; j < width; j++)
    {
      float x = j - width/2, y = i - height/2;
      float tan_theta = tan(theta);
      cv::Vec3f world_point(x / fx, y / fy, 0);
      if(x < y * tan_theta && x < -y * tan_theta)
      {
        world_point = R_l * world_point;
        world_point = world_point + t_l;
        point2d = ocam_left.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
        int u = point2d.x, v = point2d.y;
        if(u >= 0 && u < left.cols && v >= 0 && v < left.rows)
          output.at<cv::Vec3b>(i,j) = left.at<cv::Vec3b>(v, u);
      }
      else if(x > y * tan_theta && x < -y * tan_theta)
      {
        world_point = R_f * world_point;
        world_point = world_point + t_f;
        point2d = ocam_front.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
        int u = point2d.x, v = point2d.y;
        if(u >= 0 && u < front.cols && v >= 0 && v < front.rows)
          output.at<cv::Vec3b>(i,j) = front.at<cv::Vec3b>(v, u);
      }
      else if(x > -y * tan_theta && x > y * tan_theta)
      {
        world_point = R_r * world_point;
        world_point = world_point + t_r;
        point2d = ocam_right.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
        int u = point2d.x, v = point2d.y;
        if(u >= 0 && u < right.cols && v >= 0 && v < right.rows)
          output.at<cv::Vec3b>(i,j) = right.at<cv::Vec3b>(v, u);
      }
      else
      {
        world_point = R_b * world_point;
        world_point = world_point + t_b;
        point2d = ocam_back.World2Camera(cv::Point3f(world_point(0), world_point(1), world_point(2)));
        int u = point2d.x, v = point2d.y;
        if(u >= 0 && u < back.cols && v >= 0 && v < back.rows)
          output.at<cv::Vec3b>(i,j) = back.at<cv::Vec3b>(v, u);
      }
    }
}


void computeIntegral(cv::Mat src, vector<vector<unsigned long long>>& res)
{

}


void write(std::string fileName, cv::Mat mapx, cv::Mat mapy)
{
  if(fileName.empty())
  {
    // std::cout << "文件名为空" << std::endl;
    // return;
    throw std::runtime_error("文件名为空");
  }

  if(mapx.empty() || mapy.empty())
  {
    std::cout << "Mat为空" << std::endl;
    return;
  }

  ofstream out(fileName, ios::binary);

  int rows = mapx.rows, cols = mapx.cols, type = mapx.type();

  out.write((char*)&rows, sizeof(int));
  out.write((char*)&cols, sizeof(int));
  out.write((char*)&type, sizeof(int));
  out.write((char*)mapx.data, mapx.rows*mapx.cols*mapx.elemSize());

  rows = mapy.rows;
  cols = mapy.cols;
  type = mapy.type();
  out.write((char*)&rows, sizeof(int));
  out.write((char*)&cols, sizeof(int));
  out.write((char*)&type, sizeof(int));
  out.write((char*)mapy.data, mapy.rows*mapy.cols*mapy.elemSize());

  out.flush();
  out.close();
}


void write(std::string fileName, cv::Mat left_mapx, cv::Mat left_mapy, cv::Mat front_mapx, cv::Mat front_mapy, cv::Mat right_mapx, cv::Mat right_mapy, cv::Mat back_mapx, cv::Mat back_mapy, 
            double fx, double fy, double cx, double cy, double ThDepth, double baseline_left2front, double baseline_front2right, double baseline_right2back, double baseline_back2left, 
            double left_xcL, double left_xcR, double front_xcL, double front_xcR, double right_xcL, double right_xcR, double back_xcL, double back_xcR)
{
  if(fileName.empty())
  {
    // std::cout << "文件名为空" << std::endl;
    // return;
    throw std::runtime_error("文件名为空");
  }

  if(left_mapx.empty() || left_mapy.empty())
  {
    // std::cout << "Mat为空" << std::endl;
    // return;
    throw std::runtime_error("左目map为空");
  }

  if(front_mapx.empty() || front_mapy.empty())
  {
    // std::cout << "Mat为空" << std::endl;
    // return;
    throw std::runtime_error("前目map为空");
  }

  if(right_mapx.empty() || right_mapy.empty())
  {
    // std::cout << "Mat为空" << std::endl;
    // return;
    throw std::runtime_error("右目map为空");
  }

  if(back_mapx.empty() || back_mapy.empty())
  {
    // std::cout << "Mat为空" << std::endl;
    // return;
    throw std::runtime_error("后目map为空");
  }

  ofstream out(fileName, ios::binary);

  // 写入左目mapx和mapy
  int rows = left_mapx.rows, cols = left_mapx.cols, type = left_mapx.type();
  out.write((char*)&rows, sizeof(int));
  out.write((char*)&cols, sizeof(int));
  out.write((char*)&type, sizeof(int));
  out.write((char*)left_mapx.data, left_mapx.rows*left_mapx.cols*left_mapx.elemSize());

  rows = left_mapy.rows;
  cols = left_mapy.cols;
  type = left_mapy.type();
  out.write((char*)&rows, sizeof(int));
  out.write((char*)&cols, sizeof(int));
  out.write((char*)&type, sizeof(int));
  out.write((char*)left_mapy.data, left_mapy.rows*left_mapy.cols*left_mapy.elemSize());

  // 写入前目mapx和mapy
  rows = front_mapx.rows, cols = front_mapx.cols, type = front_mapx.type();
  out.write((char*)&rows, sizeof(int));
  out.write((char*)&cols, sizeof(int));
  out.write((char*)&type, sizeof(int));
  out.write((char*)front_mapx.data, front_mapx.rows*front_mapx.cols*front_mapx.elemSize());

  rows = front_mapy.rows;
  cols = front_mapy.cols;
  type = front_mapy.type();
  out.write((char*)&rows, sizeof(int));
  out.write((char*)&cols, sizeof(int));
  out.write((char*)&type, sizeof(int));
  out.write((char*)front_mapy.data, front_mapy.rows*front_mapy.cols*front_mapy.elemSize());


  // 写入右目mapx和mapy
  rows = right_mapx.rows, cols = right_mapx.cols, type = right_mapx.type();
  out.write((char*)&rows, sizeof(int));
  out.write((char*)&cols, sizeof(int));
  out.write((char*)&type, sizeof(int));
  out.write((char*)right_mapx.data, right_mapx.rows*right_mapx.cols*right_mapx.elemSize());

  rows = right_mapy.rows;
  cols = right_mapy.cols;
  type = right_mapy.type();
  out.write((char*)&rows, sizeof(int));
  out.write((char*)&cols, sizeof(int));
  out.write((char*)&type, sizeof(int));
  out.write((char*)right_mapy.data, right_mapy.rows*right_mapy.cols*right_mapy.elemSize());

  // 写入后目mapx和mapy
  rows = back_mapx.rows, cols = back_mapx.cols, type = back_mapx.type();
  out.write((char*)&rows, sizeof(int));
  out.write((char*)&cols, sizeof(int));
  out.write((char*)&type, sizeof(int));
  out.write((char*)back_mapx.data, back_mapx.rows*back_mapx.cols*back_mapx.elemSize());

  rows = back_mapy.rows;
  cols = back_mapy.cols;
  type = back_mapy.type();
  out.write((char*)&rows, sizeof(int));
  out.write((char*)&cols, sizeof(int));
  out.write((char*)&type, sizeof(int));
  out.write((char*)back_mapy.data, back_mapy.rows*back_mapy.cols*back_mapy.elemSize());

  // 写入fx，fy，cx，cy，ThDepth
  out.write((char*)&fx, sizeof(double));
  out.write((char*)&fy, sizeof(double));
  out.write((char*)&cx, sizeof(double));
  out.write((char*)&cy, sizeof(double));
  out.write((char*)&ThDepth, sizeof(double));

  // 写入baseline_left2front、baseline_front2right、baseline_right2back、baseline_back2left
  out.write((char*)&baseline_left2front, sizeof(double));
  out.write((char*)&baseline_front2right, sizeof(double));
  out.write((char*)&baseline_right2back, sizeof(double));
  out.write((char*)&baseline_back2left, sizeof(double));

  // 写入left_xcL、left_xcR、front_xcL、front_xcR、right_xcL、right_xcR、back_xcL、back_xcR
  out.write((char*)&left_xcL, sizeof(double));
  out.write((char*)&left_xcR, sizeof(double));
  out.write((char*)&front_xcL, sizeof(double));
  out.write((char*)&front_xcR, sizeof(double));
  out.write((char*)&right_xcL, sizeof(double));
  out.write((char*)&right_xcR, sizeof(double));
  out.write((char*)&back_xcL, sizeof(double));
  out.write((char*)&back_xcR, sizeof(double));

  out.flush();
  out.close();
}


void read(std::string fileName, cv::Mat &mapx, cv::Mat &mapy)
{
  if(fileName.empty())
  {
    std::cout << "文件名为空" << std::endl;
    return;
  }

  ifstream in(fileName, ios::binary);

  int rows, cols, type;
  in.read((char*)&rows, sizeof(int));
  in.read((char*)&cols, sizeof(int));
  in.read((char*)&type, sizeof(int));
  mapx = cv::Mat(rows, cols, type);
  in.read((char*)mapx.data, mapx.rows*mapx.cols*mapx.elemSize());

  in.read((char*)&rows, sizeof(int));
  in.read((char*)&cols, sizeof(int));
  in.read((char*)&type, sizeof(int));
  mapy = cv::Mat(rows, cols, type);
  in.read((char*)mapy.data, mapy.rows*mapy.cols*mapy.elemSize());

  in.close();
}


void read(std::string fileName, cv::Mat &left_mapx, cv::Mat &left_mapy, cv::Mat &front_mapx, cv::Mat &front_mapy, cv::Mat &right_mapx, cv::Mat &right_mapy, cv::Mat &back_mapx, cv::Mat &back_mapy, 
            double &fx, double &fy, double &cx, double &cy, double &ThDepth, double &baseline_left2front, double &baseline_front2right, double &baseline_right2back, double &baseline_back2left, 
            double &left_xcL, double &left_xcR, double &front_xcL, double &front_xcR, double &right_xcL, double &right_xcR, double &back_xcL, double &back_xcR)
{
  if(fileName.empty())
  {
    // std::cout << "文件名为空" << std::endl;
    // return;
    throw std::runtime_error("文件名为空");
  }

  ifstream in(fileName, ios::binary);

  int rows, cols, type;

  // 读取左目mapx和mapy
  in.read((char*)&rows, sizeof(int));
  in.read((char*)&cols, sizeof(int));
  in.read((char*)&type, sizeof(int));
  left_mapx = cv::Mat(rows, cols, type);
  in.read((char*)left_mapx.data, left_mapx.rows*left_mapx.cols*left_mapx.elemSize());

  in.read((char*)&rows, sizeof(int));
  in.read((char*)&cols, sizeof(int));
  in.read((char*)&type, sizeof(int));
  left_mapy = cv::Mat(rows, cols, type);
  in.read((char*)left_mapy.data, left_mapy.rows*left_mapy.cols*left_mapy.elemSize());

  // 读取前目mapx和mapy
  in.read((char*)&rows, sizeof(int));
  in.read((char*)&cols, sizeof(int));
  in.read((char*)&type, sizeof(int));
  front_mapx = cv::Mat(rows, cols, type);
  in.read((char*)front_mapx.data, front_mapx.rows*front_mapx.cols*front_mapx.elemSize());

  in.read((char*)&rows, sizeof(int));
  in.read((char*)&cols, sizeof(int));
  in.read((char*)&type, sizeof(int));
  front_mapy = cv::Mat(rows, cols, type);
  in.read((char*)front_mapy.data, front_mapy.rows*front_mapy.cols*front_mapy.elemSize());

  // 读取右目mapx和mapy
  in.read((char*)&rows, sizeof(int));
  in.read((char*)&cols, sizeof(int));
  in.read((char*)&type, sizeof(int));
  right_mapx = cv::Mat(rows, cols, type);
  in.read((char*)right_mapx.data, right_mapx.rows*right_mapx.cols*right_mapx.elemSize());

  in.read((char*)&rows, sizeof(int));
  in.read((char*)&cols, sizeof(int));
  in.read((char*)&type, sizeof(int));
  right_mapy = cv::Mat(rows, cols, type);
  in.read((char*)right_mapy.data, right_mapy.rows*right_mapy.cols*right_mapy.elemSize());

  // 读取后目mapx和mapy
  in.read((char*)&rows, sizeof(int));
  in.read((char*)&cols, sizeof(int));
  in.read((char*)&type, sizeof(int));
  back_mapx = cv::Mat(rows, cols, type);
  in.read((char*)back_mapx.data, back_mapx.rows*back_mapx.cols*back_mapx.elemSize());

  in.read((char*)&rows, sizeof(int));
  in.read((char*)&cols, sizeof(int));
  in.read((char*)&type, sizeof(int));
  back_mapy = cv::Mat(rows, cols, type);
  in.read((char*)back_mapy.data, back_mapy.rows*back_mapy.cols*back_mapy.elemSize());

  // 写入fx，fy，cx，cy，ThDepth
  in.read((char*)&fx, sizeof(double));
  in.read((char*)&fy, sizeof(double));
  in.read((char*)&cx, sizeof(double));
  in.read((char*)&cy, sizeof(double));
  in.read((char*)&ThDepth, sizeof(double));

  // 写入baseline_left2front、baseline_front2right、baseline_right2back、baseline_back2left
  in.read((char*)&baseline_left2front, sizeof(double));
  in.read((char*)&baseline_front2right, sizeof(double));
  in.read((char*)&baseline_right2back, sizeof(double));
  in.read((char*)&baseline_back2left, sizeof(double));

  // 写入left_xcL、left_xcR、front_xcL、front_xcR、right_xcL、right_xcR、back_xcL、back_xcR
  in.read((char*)&left_xcL, sizeof(double));
  in.read((char*)&left_xcR, sizeof(double));
  in.read((char*)&front_xcL, sizeof(double));
  in.read((char*)&front_xcR, sizeof(double));
  in.read((char*)&right_xcL, sizeof(double));
  in.read((char*)&right_xcR, sizeof(double));
  in.read((char*)&back_xcL, sizeof(double));
  in.read((char*)&back_xcR, sizeof(double));

  in.close();
}



int main(int argc, char *argv[])
{
  double baseline_left2front, baseline_front2right, baseline_right2back, baseline_back2left;
  double left_xcL, left_xcR, front_xcL, front_xcR, right_xcL, right_xcR, back_xcL, back_xcR;
  double ThDepth;

  Load();
  initImages();

  bool fromFile = true;
  // xc = COLS, yc = ROWS/2.0;

  if(!fromFile)
  {
    // const float FOVx = 160;
    // const float FOVy = 120;
    const float FOVx = 90;
    const float FOVy = 90;

    ThDepth = 40;
    xc = COLS, yc = ROWS/2.0;
    fx = xc/2.0 / tan(FOVx/2 * CV_PI / 180);
    fy = yc / tan(FOVy/2 * CV_PI / 180);

    // Rcw
    cv::Matx33d Rx = Rotation_x(CV_PI/2);
    Rotation_left_ideal = Rx * Rotation_z(-CV_PI/2);
    Rotation_front_ideal = Rx;
    Rotation_right_ideal = Rx * Rotation_z(CV_PI/2);
    Rotation_back_ideal = Rx * Rotation_z(CV_PI);
    
    // twc
    // const int HIGH = -1;
    // Translation_left_ideal = cv::Vec3d(-1, 0, HIGH);
    // Translation_front_ideal = cv::Vec3d(0, -2, HIGH);
    // Translation_right_ideal = cv::Vec3d(1, 0, HIGH);
    // Translation_back_ideal = cv::Vec3d(0, 2, HIGH);
    // Translation_left_ideal = cv::Vec3d(Translation_left(0), Translation_left(1), HIGH);
    // Translation_front_ideal = cv::Vec3d(Translation_front(0), Translation_front(1), HIGH);
    // Translation_right_ideal = cv::Vec3d(Translation_right(0), Translation_right(1), HIGH);
    // Translation_back_ideal = cv::Vec3d(Translation_back(0), Translation_back(1), HIGH);

    Translation_left_ideal = Translation_left;
    Translation_front_ideal = Translation_front;
    Translation_right_ideal = Translation_right;
    Translation_back_ideal = Translation_back;

    calculateTwoCorrectRotation2(Rotation_left_ideal, Translation_left_ideal, Rotation_front_ideal, Translation_front_ideal, Rotation_left_r, Rotation_front_l, baseline_left2front);
    calculateTwoCorrectRotation2(Rotation_front_ideal, Translation_front_ideal, Rotation_right_ideal, Translation_right_ideal, Rotation_front_r, Rotation_right_l, baseline_front2right);
    calculateTwoCorrectRotation2(Rotation_right_ideal, Translation_right_ideal, Rotation_back_ideal, Translation_back_ideal, Rotation_right_r, Rotation_back_l, baseline_right2back);
    calculateTwoCorrectRotation2(Rotation_back_ideal, Translation_back_ideal, Rotation_left_ideal, Translation_left_ideal, Rotation_back_r, Rotation_left_l, baseline_back2left);

    // std::cout << "baseline_left2front: " << baseline_left2front << std::endl;
    // std::cout << "baseline_front2right: " << baseline_front2right << std::endl;
    // std::cout << "baseline_right2back: " << baseline_right2back << std::endl;
    // std::cout << "baseline_back2left: " << baseline_back2left << std::endl;

    correctCamera_LUT2(mapx_left, mapy_left, Rotation_left_l, Rotation_left_r, Rotation_left_ideal * Rotation_left, fx, fy, xc, yc, ocam_model_left, left_xcL, left_xcR);
    correctCamera_LUT2(mapx_front, mapy_front, Rotation_front_l, Rotation_front_r, Rotation_front_ideal * Rotation_front, fx, fy, xc, yc, ocam_model_front, front_xcL, front_xcR);
    correctCamera_LUT2(mapx_right, mapy_right, Rotation_right_l, Rotation_right_r, Rotation_right_ideal * Rotation_right, fx, fy, xc, yc, ocam_model_right, right_xcL, right_xcR);
    correctCamera_LUT2(mapx_back, mapy_back, Rotation_back_l, Rotation_back_r, Rotation_back_ideal * Rotation_back, fx, fy, xc, yc, ocam_model_back, back_xcL, back_xcR);

    // topview(topview_full, src_left, src_front, src_right, src_back, Rotation_left, Translation_left, Rotation_front, Translation_front, Rotation_right, Translation_right, Rotation_back, Translation_back, ocam_model_left, ocam_model_front, ocam_model_right, ocam_model_back);

    // cv::FileStorage fs("parameters.yaml", cv::FileStorage::WRITE);

    // fs << "fx" << fx;
    // fs << "fy" << fy;
    // fs << "xc" << xc;
    // fs << "yc" << yc;
    // fs << "ThDepth" << ThDepth;
    // fs << "baseline_left2front" << baseline_left2front;
    // fs << "baseline_front2right" << baseline_front2right;
    // fs << "baseline_right2back" << baseline_right2back;
    // fs << "baseline_back2left" << baseline_back2left;
    // fs << "left_xcL" << left_xcL;
    // fs << "left_xcR" << left_xcR;
    // fs << "front_xcL" << front_xcL;
    // fs << "front_xcR" << front_xcR;
    // fs << "right_xcL" << right_xcL;
    // fs << "right_xcR" << right_xcR;
    // fs << "back_xcL" << back_xcL;
    // fs << "back_xcR" << back_xcR;

    // fs.release();

    // write("map_left.mb", mapx_left, mapy_left);
    // write("map_front.mb", mapx_front, mapy_front);
    // write("map_right.mb", mapx_right, mapy_right);
    // write("map_back.mb", mapx_back, mapy_back);
    write("multi_fisheye.b", mapx_left, mapy_left, mapx_front, mapy_front, mapx_right, mapy_right, mapx_back, mapy_back, fx, fy, xc, yc, ThDepth, baseline_left2front, baseline_front2right, baseline_right2back, baseline_back2left, left_xcL, left_xcR, front_xcL, front_xcR, right_xcL, right_xcR, back_xcL, back_xcR);
  }
  else
  {
    // cv::FileStorage fs("parameters.yaml", cv::FileStorage::READ);
    
    // fs["fx"] >> fx;
    // fs["fy"] >> fy;
    // fs["ThDepth"] >> ThDepth;
    // fs["xc"] >> xc;
    // fs["yc"] >> yc;
    // fs["baseline_left2front"] >> baseline_left2front;
    // fs["baseline_front2right"] >> baseline_front2right;
    // fs["baseline_right2back"] >> baseline_right2back;
    // fs["baseline_back2left"] >> baseline_back2left;
    // fs["left_xcL"] >> left_xcL;
    // fs["left_xcR"] >> left_xcR;
    // fs["front_xcL"] >> front_xcL;
    // fs["front_xcR"] >> front_xcR;
    // fs["right_xcL"] >> right_xcL;
    // fs["right_xcR"] >> right_xcR;
    // fs["back_xcL"] >> back_xcL;
    // fs["back_xcR"] >> back_xcR;

    // fs.release();

    // read("map_left.mb", mapx_left, mapy_left);
    // read("map_front.mb", mapx_front, mapy_front);
    // read("map_right.mb", mapx_right, mapy_right);
    // read("map_back.mb", mapx_back, mapy_back);
    // read("multi_fisheye.b", mapx_left, mapy_left, mapx_front, mapy_front, mapx_right, mapy_right, mapx_back, mapy_back, fx, fy, xc, yc, ThDepth, baseline_left2front, baseline_front2right, baseline_right2back, baseline_back2left, left_xcL, left_xcR, front_xcL, front_xcR, right_xcL, right_xcR, back_xcL, back_xcR);

    multi.read("multi_fisheye.b");
    fx = multi.fx;
    fy = multi.fy;
    xc = multi.cx;
    yc = multi.cy;
    ThDepth = multi.ThDepth;

    baseline_left2front = multi.left_baseline;
    baseline_front2right = multi.front_baseline;
    baseline_right2back = multi.right_baseline;
    baseline_back2left = multi.back_baseline;

    // left_xcL = multi.left_cxL;
    // left_xcR = multi.left_cxR;
    // front_xcL = multi.front_cxL;
    // front_xcR = multi.front_cxR;
    // right_xcL = multi.right_cxL;
    // right_xcR = multi.right_cxR;
    // back_xcL = multi.back_cxL;
    // back_xcR = multi.back_cxR;

    mapx_left = multi.left_mapx;
    mapy_left = multi.left_mapy;

    mapx_front = multi.front_mapx;
    mapy_front = multi.front_mapy;

    mapx_right = multi.right_mapx;
    mapy_right = multi.right_mapy;

    mapx_back = multi.back_mapx;
    mapy_back = multi.back_mapy;

  }


  std::cout << "xc: " << xc << ", yc: " << yc << std::endl;
  std::cout << "fx: " << fx << ", fy: " << fy << std::endl;
  std::cout << "水平方向FOV: " << 2 * atan2(xc/2.0, fx) / CV_PI * 180 << std::endl;
  std::cout << "垂直方向FOV: " << 2 * atan2(yc, fy) / CV_PI * 180 << std::endl;


  cv::remap(src_left, dst_left, mapx_left, mapy_left, CV_INTER_LINEAR);
  cv::remap(src_front, dst_front, mapx_front, mapy_front, CV_INTER_LINEAR);
  cv::remap(src_right, dst_right, mapx_right, mapy_right, CV_INTER_LINEAR);
  cv::remap(src_back, dst_back, mapx_back, mapy_back, CV_INTER_LINEAR);

  src_left = dst_left.clone();
  src_front = dst_front.clone();
  src_right = dst_right.clone();
  src_back = dst_back.clone();

  // clock_t start = clock();

  cv::cvtColor(dst_left, dst_left, CV_BGR2GRAY);
  cv::cvtColor(dst_front, dst_front, CV_BGR2GRAY);
  cv::cvtColor(dst_right, dst_right, CV_BGR2GRAY);
  cv::cvtColor(dst_back, dst_back, CV_BGR2GRAY);

  // clock_t end = clock();
  // std::cout << "cost time: " << (double)(end-start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;


  clock_t start = clock();


  //calculateMatches(dst_left, dst_front, matches, large);
  //calculateMatches(dst_front, dst_right, matches, large);
  //calculateMatches(dst_right, dst_back, matches, large);
  const int POINTS = 2000;
  ORB_SLAM2::ORBextractor* orb1 = new ORB_SLAM2::ORBextractor(POINTS, 1.2f, 8, 31, 8);
  ORB_SLAM2::ORBextractor* orb2 = new ORB_SLAM2::ORBextractor(POINTS, 1.2f, 8, 31, 8);
  ORB_SLAM2::ORBextractor* orb3 = new ORB_SLAM2::ORBextractor(POINTS, 1.2f, 8, 31, 8);
  ORB_SLAM2::ORBextractor* orb4 = new ORB_SLAM2::ORBextractor(POINTS, 1.2f, 8, 31, 8);

  ORB_SLAM2::ORBVocabulary* voc = new ORB_SLAM2::ORBVocabulary();
  cv::Mat K = (cv::Mat_<float>(3,3) << fx, 0, xc, 0, fy, yc, 0, 0, 1);
  cv::Mat distCoef = cv::Mat::zeros(4,1,CV_32F);
  double ThDepth_left = baseline_left2front * ThDepth;
  double ThDepth_front = baseline_front2right * ThDepth;
  double ThDepth_right = baseline_right2back * ThDepth;
  double ThDepth_back = baseline_back2left * ThDepth;

  ORB_SLAM2::Frame frame_left = ORB_SLAM2::Frame(dst_left, 0.0, orb1, voc, K, distCoef, baseline_left2front * fx, ThDepth_left);
  ORB_SLAM2::Frame frame_front = ORB_SLAM2::Frame(dst_front, 0.0, orb2, voc, K, distCoef, baseline_front2right * fx, ThDepth_front);
  ORB_SLAM2::Frame frame_right = ORB_SLAM2::Frame(dst_right, 0.0, orb3, voc, K, distCoef, baseline_right2back * fx, ThDepth_right);
  ORB_SLAM2::Frame frame_back = ORB_SLAM2::Frame(dst_back, 0.0, orb4, voc, K, distCoef, baseline_back2left * fx, ThDepth_back);

/*
  // clock_t end = clock();
  // std::cout << "cost time: " << (double)(end-start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;

  // clock_t start = clock();

  // ORB_SLAM2::Frame::ComputeStereoMatches(frame_left, frame_front, left_xcR, front_xcL);
  ComputeStereoMatches(frame_left, frame_front, left_xcR, front_xcL);

  // clock_t end = clock();
  // std::cout << "ComputeStereoMatches time: " << (double)(end-start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;
  
  // start = end;
  // ORB_SLAM2::Frame::ComputeStereoMatches(frame_front, frame_right, front_xcR, right_xcL);
  ComputeStereoMatches(frame_front, frame_right, front_xcR, right_xcL);
  // end = clock();
  // std::cout << "ComputeStereoMatches time: " << (double)(end-start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;
  
  // start = end;
  // ORB_SLAM2::Frame::ComputeStereoMatches(frame_right, frame_back, right_xcR, back_xcL);
  ComputeStereoMatches(frame_right, frame_back, right_xcR, back_xcL);
  // end = clock();
  // std::cout << "ComputeStereoMatches time: " << (double)(end-start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;
  
  // start = end;
  // ORB_SLAM2::Frame::ComputeStereoMatches(frame_back, frame_left, back_xcR, left_xcL);
  ComputeStereoMatches(frame_back, frame_left, back_xcR, left_xcL);
  // end = clock();
  // clock_t end = clock();
  // std::cout << "ComputeStereoMatches time: " << (double)(end-start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;
*/

  multi.ComputeStereoMatches(frame_left, frame_front, frame_right, frame_back);


  clock_t end = clock();
  std::cout << "total time: " << (double)(end-start) / CLOCKS_PER_SEC * 1000 << " ms." << std::endl;


  // 以下代码用于调试显示
  vector<cv::DMatch> match_left2front, match_front2right, match_right2back, match_back2left;
  
  int N = frame_left.N;
  match_left2front.reserve(N);
  match_front2right.reserve(N);
  match_right2back.reserve(N);
  match_back2left.reserve(N);

  for(int i = 0; i < N; i++)
  {
    if(frame_left.mvMatch12[i] < 0)
      continue;
    
    // std::cout << "Depth: " << frame_left.mvDepth[i] << std::endl;
    match_left2front.push_back(cv::DMatch(i, frame_left.mvMatch12[i], FLT_MAX));
  }
  // std::cout << std::endl;

  N = frame_front.N;
  for(int i = 0; i < N; i++)
  {
    if(frame_front.mvMatch12[i] < 0)
      continue;
    
    // std::cout << "Depth: " << frame_front.mvDepth[i] << std::endl;
    match_front2right.push_back(cv::DMatch(i, frame_front.mvMatch12[i], FLT_MAX));
  }
  // std::cout << std::endl;

  N = frame_right.N;
  for(int i = 0; i < N; i++)
  {
    if(frame_right.mvMatch12[i] < 0)
      continue;
    
    // std::cout << "Depth: " << frame_right.mvDepth[i] << std::endl;
    match_right2back.push_back(cv::DMatch(i, frame_right.mvMatch12[i], FLT_MAX));
  }
  // std::cout << std::endl;

  N = frame_back.N;
  for(int i = 0; i < N; i++)
  {
    if(frame_back.mvMatch12[i] < 0)
      continue;
    
    // std::cout << "Depth: " << frame_back.mvDepth[i] << std::endl;
    match_back2left.push_back(cv::DMatch(i, frame_back.mvMatch12[i], FLT_MAX));
  }
  // std::cout << std::endl;

  std::cout << "match_left2front: " << match_left2front.size() << std::endl;
  cv::drawMatches(src_left, frame_left.mvKeysUn, src_front, frame_front.mvKeysUn, match_left2front, large1);
  
  std::cout << "match_front2right: " << match_front2right.size() << std::endl;
  cv::drawMatches(src_front, frame_front.mvKeysUn, src_right, frame_right.mvKeysUn, match_front2right, large2);
  
  std::cout << "match_right2back: " << match_right2back.size() << std::endl;
  cv::drawMatches(src_right, frame_right.mvKeysUn, src_back, frame_back.mvKeysUn, match_right2back, large3);

  std::cout << "match_back2left: " << match_back2left.size() << std::endl;
  cv::drawMatches(src_back, frame_back.mvKeysUn, src_left, frame_left.mvKeysUn, match_back2left, large4);

  mergeImagesandDrawLines(src_left, src_front, src_right, src_back, large);

  delete orb1;
  delete orb2;
  delete orb3;
  delete orb4;
  delete voc;

  // topview(topview_full, src_left, src_front, src_right, src_back, Rotation_left, Translation_left, Rotation_front, Translation_front, Rotation_right, Translation_right, Rotation_back, Translation_back, ocam_model_left, ocam_model_front, ocam_model_right, ocam_model_back);

  cv::namedWindow( "topview", 0 );

  cv::namedWindow( "large image1", 0 );
  cv::namedWindow( "large image2", 0 );
  cv::namedWindow( "large image3", 0 );
  cv::namedWindow( "large image4", 0 );

  // cv::namedWindow( "toolbox", 0 );

  // cv::imshow( "topview", topview_full);
  cv::imshow( "topview", large);

  cv::imshow( "large image1", large1);
  cv::imshow( "large image2", large2);
  cv::imshow( "large image3", large3);
  cv::imshow( "large image4", large4);


  // cv::createTrackbar("fx", "toolbox", &fx, slider_max, Onchange);
  // cv::createTrackbar("fy", "toolbox", &fy, slider_max, Onchange);
  // cv::createTrackbar("xc", "toolbox", &xc, slider_max, Onchange);
  // cv::createTrackbar("yc", "toolbox", &yc, slider_max, Onchange);

  /* --------------------------------------------------------------------*/
  /* Wait until Key 'q' pressed                                         */
  /* --------------------------------------------------------------------*/
  while((char)cv::waitKey(10) != 'q');
 
  /* --------------------------------------------------------------------*/
  /* Save image                                                          */
  /* --------------------------------------------------------------------*/
  cv::imwrite("large.jpg", large);
  cv::imwrite("large1.jpg", large1);
  cv::imwrite("large2.jpg", large2);
  cv::imwrite("large3.jpg", large3);
  cv::imwrite("large4.jpg", large4);

  printf("\nImages %s saved\n","large.jpg");

  return 0;
}
