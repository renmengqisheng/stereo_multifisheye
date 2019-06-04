/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*
*   Editor      : Tao Li
*   File name   : multi_fisheye_param.cpp
*   Author      : Mofan
*   Created date: 2019-05-29 20:14:15
*   Description :
*
*===============================================================*/

#include "multi_fisheye_param.h"

multi_fisheye_param::multi_fisheye_param()
{}

multi_fisheye_param::multi_fisheye_param(std::string fileName)
{
  read(fileName);
}

void multi_fisheye_param::write(std::string fileName)
{
  if(fileName.empty())
  {
    throw std::runtime_error("文件名为空");
  }

  if(left_mapx.empty() || left_mapy.empty())
  {
    throw std::runtime_error("左目map为空");
  }

  if(front_mapx.empty() || front_mapy.empty())
  {
    throw std::runtime_error("前目map为空");
  }

  if(right_mapx.empty() || right_mapy.empty())
  {
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
  out.write((char*)&left_baseline, sizeof(double));
  out.write((char*)&front_baseline, sizeof(double));
  out.write((char*)&right_baseline, sizeof(double));
  out.write((char*)&back_baseline, sizeof(double));

  // 写入left_xcL、left_xcR、front_xcL、front_xcR、right_xcL、right_xcR、back_xcL、back_xcR
  out.write((char*)&left_cxL, sizeof(double));
  out.write((char*)&left_cxR, sizeof(double));
  out.write((char*)&front_cxL, sizeof(double));
  out.write((char*)&front_cxR, sizeof(double));
  out.write((char*)&right_cxL, sizeof(double));
  out.write((char*)&right_cxR, sizeof(double));
  out.write((char*)&back_cxL, sizeof(double));
  out.write((char*)&back_cxR, sizeof(double));

  out.flush();
  out.close();
}

void multi_fisheye_param::read(std::string fileName)
{
  if(fileName.empty())
  {
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

  // 读取fx，fy，cx，cy，ThDepth
  in.read((char*)&fx, sizeof(double));
  in.read((char*)&fy, sizeof(double));
  in.read((char*)&cx, sizeof(double));
  in.read((char*)&cy, sizeof(double));
  in.read((char*)&ThDepth, sizeof(double));

  // 读取baseline_left2front、baseline_front2right、baseline_right2back、baseline_back2left
  in.read((char*)&left_baseline, sizeof(double));
  in.read((char*)&front_baseline, sizeof(double));
  in.read((char*)&right_baseline, sizeof(double));
  in.read((char*)&back_baseline, sizeof(double));

  // 读取left_xcL、left_xcR、front_xcL、front_xcR、right_xcL、right_xcR、back_xcL、back_xcR
  in.read((char*)&left_cxL, sizeof(double));
  in.read((char*)&left_cxR, sizeof(double));
  in.read((char*)&front_cxL, sizeof(double));
  in.read((char*)&front_cxR, sizeof(double));
  in.read((char*)&right_cxL, sizeof(double));
  in.read((char*)&right_cxR, sizeof(double));
  in.read((char*)&back_cxL, sizeof(double));
  in.read((char*)&back_cxR, sizeof(double));

  in.close();

  left_ThDepth = left_baseline * ThDepth;
  front_ThDepth = front_baseline * ThDepth;
  right_ThDepth = right_baseline * ThDepth;
  back_ThDepth = back_baseline * ThDepth;
  left_bf = left_baseline * fx;
  front_bf = front_baseline * fx;
  right_bf = right_baseline * fx;
  back_bf = back_baseline * fx;
  K = (cv::Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  distCoef = cv::Mat::zeros(4,1,CV_32F);
}

float multi_fisheye_param::ZNCC(cv::Mat img1, cv::Mat img2, cv::Point2i point1, cv::Point2i point2, int size)
{
	if (img1.empty() || img1.channels() != 1 || img2.empty() || img2.channels() != 1)
	{
		std::cout << "Image error in ZNCC!" << std::endl;
		return -1;
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

		float lr = 0, ll = 0, rr = 0; //这些是自相关和互相关
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


void multi_fisheye_param::ComputeStereoMatches(ORB_SLAM2::Frame& frame1, ORB_SLAM2::Frame& frame2, enum stereo_pair flag)
{
    int cx1R, cx2L;
    if(flag == left_to_front)
    {
        cx1R = left_cxR;
        cx2L = front_cxL;
    }
    else if(flag == front_to_right)
    {
        cx1R = front_cxR;
        cx2L = right_cxL;
    }
    else if(flag == right_to_back)
    {
        cx1R = right_cxR;
        cx2L = back_cxL;
    }
    else
    {
        cx1R = back_cxR;
        cx2L = left_cxL;
    }

    const int thOrbDist = (ORB_SLAM2::ORBmatcher::TH_HIGH+ORB_SLAM2::ORBmatcher::TH_LOW)/2;

    const int nRows = frame1.mpORBextractorLeft->mvImagePyramid[0].rows;
    const int nCols = frame2.mpORBextractorLeft->mvImagePyramid[0].cols;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = frame2.mvKeysUn.size();

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

    // Set limits for search
    const float minZ = frame1.mb;
    const float minD = 0;
    const float maxD = frame1.mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(frame1.N);
    std::vector<int> vMatch12(frame1.N, -1);

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


            if(uR-cx2L+cx1R>=minU && uR-cx2L+cx1R<=maxU)
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

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {

            //新增加，通过关键点尺度差异和方向差异筛选
            const cv::KeyPoint &kpR = frame2.mvKeysUn[bestIdxR];

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
            const cv::Mat &imgL = frame1.mpORBextractorLeft->mvImagePyramid[kpL.octave];
            const cv::Mat &imgR = frame2.mpORBextractorLeft->mvImagePyramid[kpR.octave];

            float zncc = ZNCC(imgL, imgR, cv::Point2i(scaleduL,scaledvL), cv::Point2i(scaleduR,scaledvR), size);         
            if(zncc <= 0.85)
                continue;

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
            if(deltaR<-1 || deltaR>1)
                continue;
                
            // Re-scaled coordinate
            float bestuR = frame1.mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);
            float disparity = (uL-cx1R) - (bestuR-cx2L);

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
                vMatch12[iL] = bestIdxR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());

    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;
    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            frame1.mvuRight[vDistIdx[i].second]=-1;
            frame1.mvDepth[vDistIdx[i].second]=-1;
            frame2.mvDepth[vMatch12[vDistIdx[i].second]]=-1;
            vMatch12[vDistIdx[i].second] = -1;
        }
    }
    frame1.mvMatch12 = vMatch12;  //用于显示，正式使用时应删除
}

void multi_fisheye_param::ComputeStereoMatches(ORB_SLAM2::Frame& left_frame, ORB_SLAM2::Frame& front_frame, ORB_SLAM2::Frame& right_frame, ORB_SLAM2::Frame& back_frame)
{
  ComputeStereoMatches(left_frame, front_frame, left_to_front);
  ComputeStereoMatches(front_frame, right_frame, front_to_right);
  ComputeStereoMatches(right_frame, back_frame, right_to_back);
  ComputeStereoMatches(back_frame, left_frame, back_to_left);
}





