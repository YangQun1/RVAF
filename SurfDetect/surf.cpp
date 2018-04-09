/* SurfDetect
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1. The program use surf to detect feature points
// and generate feature descriptor. You can also directly sample image from video
// or camera, a webcamera is necessary if you run the program on camera mode.
// The program can mosaic two image and draw the match point, you can see usage
// and input the parameters by command line. The surf detect part referenced on
// OpenSurt(https://github.com/amarburg/opencv-ffi-ext/tree/master/ext/opensurf)
// project, and the core of code write by C. The test shows the program is more
// efficiency than OpenSurf. Enjoy it!
// Created at: 26 Aug. 2016, all rights reserved.*/


#include "common.h"
#include "integral.h"
#include "fasthessian.h"
#include "descriptor.h"
#include "mosaic.h"
#include "stereo.h"
#include "surfdisp.h"
#include <glog\logging.h>

#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;

namespace pc{


AlgParam algparam;
SurfParam suparam;

/*
同时计算surf关键点和描述子
*/
static void surfDetect(uimg& img, vector<Ipoint>& ipts){
	fimg ii_img;
	ii_img.data = (float*)calloc(img.cols * img.rows * img.chns, sizeof(float));
	compute_integral(img, ii_img);

	vector<ResponseLayer> responseMap = buildResponseMap(ii_img);
	static const int filter_map[OCTAVES][INTERVALS]
		= { { 0, 1, 2, 3 }, { 1, 3, 4, 5 }, { 3, 5, 6, 7 }, { 5, 7, 8, 9 }, { 7, 9, 10, 11 } };

	for (int o = 0; o < suparam.octaves; ++o){
	  for (int i = 0; i <= 1; ++i){
		int height = responseMap.at(filter_map[o][i + 2]).height;
		int width = responseMap.at(filter_map[o][i + 2]).width;
		for (int r = 0; r < height; ++r){
		  for (int c = 0; c < width; ++c){
			if (isExtremum(r, c, responseMap.at(filter_map[o][i + 2]),
			  responseMap.at(filter_map[o][i + 1]), responseMap.at(filter_map[o][i]))){
			  interpolateExtremun(r, c, responseMap.at(filter_map[o][i + 2]), 
			  responseMap.at(filter_map[o][i + 1]), responseMap.at(filter_map[o][i]), ipts);
			}
		  }
		}
	  }
	}
	surfDescriptors(ii_img, ipts);

	releaseResponseMap(responseMap);
	free(ii_img.data);
}

/*
计算surf特征点（关键点、描述子），并绘制在图像上
*/
void surfImage(Mat& image){
	Mat gray;
	cvtColor(image, gray, CV_BGR2GRAY);

	uimg img;
	img.chns = gray.channels();
	img.cols = gray.cols;
	img.rows = gray.rows;
	img.data = gray.data;

	suparam.thresh = (suparam.thresh >= 0) ? suparam.thresh : THRES;
	suparam.stride = (suparam.stride > 0 && suparam.stride <= 6) ? suparam.stride : STRIDE;
	suparam.octaves = (suparam.octaves > 0 && suparam.octaves <= 4) ? suparam.octaves : OCTAVES;
	suparam.intervals = (suparam.intervals > 0 && suparam.intervals <= 4) ? suparam.intervals : INTERVALS;

	vector<Ipoint> ipts;
	surfDetect(img, ipts);
	surfDraw(image, ipts);
}

// 计算两幅图像的surf特征，进行匹配（暴力匹配），
// 计算图像之间的变换关系，计算匹配点的3D坐标（调用MATLAB接口）
// 画出匹配图像
Mat surfMatch(Mat& image1, Mat& image2){
	suparam.thresh = (suparam.thresh >= 0) ? suparam.thresh : THRES;
	suparam.stride = (suparam.stride > 0 && suparam.stride <= 6) ? suparam.stride : STRIDE;
	suparam.octaves = (suparam.octaves > 0 && suparam.octaves <= 4) ? suparam.octaves : OCTAVES;
	suparam.intervals = (suparam.intervals > 0 && suparam.intervals <= 4) ? suparam.intervals : INTERVALS;

	Mat gray1, gray2;
	cvtColor(image1, gray1, CV_BGR2GRAY);
	cvtColor(image2, gray2, CV_BGR2GRAY);

	uimg img1, img2;
	img1.chns = gray1.channels();
	img1.cols = gray1.cols;
	img1.rows = gray1.rows;
	img1.data = gray1.data;
	img2.chns = gray2.channels();
	img2.cols = gray2.cols;
	img2.rows = gray2.rows;
	img2.data = gray2.data;

	vector<Ipoint> ipts1, ipts2;
	surfDetect(img1, ipts1);
	surfDetect(img2, ipts2);
	if (algparam.showprocess){
		surfDraw(image1, ipts1);
		surfDraw(image2, ipts2);
	}
	vector<pair<Ipoint, Ipoint>> matches;
	iptMatch(ipts1, ipts2, matches);				// 计算两组特征点的匹配关系
	dimshow("Match", drawMatches(image1, image2, matches));
	
	Mat homography = computeHomography(matches);	// 根据匹配关系，计算两幅图像之间的单应矩阵
	cout << "matched point count: " << matches.size() << endl;
	if (algparam.runtype == 'U'){
		print3Dpoint(stereoTriangulation(matches));	// 调用MATLAB的接口，根据匹配关系计算得到3D点坐标，并打印出来
	}
	return drawMatches(image1, image2, matches);	// 绘制匹配图像
}

Mat surfMosaic(Mat& image1, Mat& image2){
	suparam.thresh = (suparam.thresh >= 0) ? suparam.thresh : THRES;
	suparam.stride = (suparam.stride > 0 && suparam.stride <= 6) ? suparam.stride : STRIDE;
	suparam.octaves = (suparam.octaves > 0 && suparam.octaves <= 4) ? suparam.octaves : OCTAVES;
	suparam.intervals = (suparam.intervals > 0 && suparam.intervals <= 4) ? suparam.intervals : INTERVALS;

	Mat gray1, gray2;
	cvtColor(image1, gray1, CV_BGR2GRAY);
	cvtColor(image2, gray2, CV_BGR2GRAY);

	uimg img1, img2;
	img1.chns = gray1.channels();
	img1.cols = gray1.cols;
	img1.rows = gray1.rows;
	img1.data = gray1.data;
	img2.chns = gray2.channels();
	img2.cols = gray2.cols;
	img2.rows = gray2.rows;
	img2.data = gray2.data;

	vector<Ipoint> ipts1, ipts2;
	surfDetect(img1, ipts1);	// 计算surf特征点
	surfDetect(img2, ipts2);
	
	vector<pair<Ipoint, Ipoint>> matches;
	iptMatch(ipts1, ipts2, matches);	// 特征点匹配
	Mat homography = computeHomography(matches);	// 计算单应
	dimshow( "Homography", drawMatches(image1, image2, matches) );

	if (homography.empty() || homography.rows != 3 || homography.cols != 3){
		return drawMatches(image1, image2, matches);
	}

	int intercount = 0;
	for (int i = 0; i < matches.size(); ++i){
		if (matches[i].second.x > matches[i].first.x){
			intercount++;
		}
	}
	if (intercount > 0.8 * matches.size()){
		invert(homography, homography);
		swap(image1, image2);
	}

	return imageMosaic(image1, image2, homography);
}


/*Interface to svaf*/
/*
计算surf关键点
*/
void SurfPoint(Mat& image, vector<Point2f>& points, vector<float>& scales, vector<int>& label){
	Mat gray;
	if (image.channels() == 3){
		cvtColor(image, gray, CV_BGR2GRAY);
	}
	else if (image.channels() == 1){
		gray = image.clone();
	}
	else{
		LOG(INFO) << "Unknown Image Format!";
	}

	uimg img;
	img.chns = gray.channels();
	img.cols = gray.cols;
	img.rows = gray.rows;
	img.data = gray.data;

	suparam.thresh = (suparam.thresh >= 0) ? suparam.thresh : THRES;
	suparam.stride = (suparam.stride > 0 && suparam.stride <= 6) ? suparam.stride : STRIDE;
	suparam.octaves = (suparam.octaves > 0 && suparam.octaves <= 4) ? suparam.octaves : OCTAVES;
	suparam.intervals = (suparam.intervals > 0 && suparam.intervals <= 4) ? suparam.intervals : INTERVALS;

	vector<Ipoint> ipts;
	fimg ii_img;
	ii_img.data = (float*)calloc(img.cols * img.rows * img.chns, sizeof(float));
	compute_integral(img, ii_img);

	vector<ResponseLayer> responseMap = buildResponseMap(ii_img);
	static const int filter_map[OCTAVES][INTERVALS]
		= { { 0, 1, 2, 3 }, { 1, 3, 4, 5 }, { 3, 5, 6, 7 }, { 5, 7, 8, 9 }, { 7, 9, 10, 11 } };

	for (int o = 0; o < suparam.octaves; ++o){
		for (int i = 0; i <= 1; ++i){
			int height = responseMap.at(filter_map[o][i + 2]).height;
			int width = responseMap.at(filter_map[o][i + 2]).width;
			for (int r = 0; r < height; ++r){
				for (int c = 0; c < width; ++c){
					if (isExtremum(r, c, responseMap.at(filter_map[o][i + 2]),
						responseMap.at(filter_map[o][i + 1]), responseMap.at(filter_map[o][i]))){
						interpolateExtremun(r, c, responseMap.at(filter_map[o][i + 2]),
							responseMap.at(filter_map[o][i + 1]), responseMap.at(filter_map[o][i]), ipts);
					}
				}
			}
		}
	}
	releaseResponseMap(responseMap);
	free(ii_img.data);

	for (int i = 0; i < ipts.size(); ++i){
		points.push_back(Point2f(ipts[i].x, ipts[i].y));
		scales.push_back(ipts[i].s);
		label.push_back(ipts[i].laplacian);
	}
}

/*
计算surf描述子
*/
void SurfDescriptor(Mat& image, vector<Point2f>& points, vector<float>& scales, 
	vector<vector<float>>& descriptors){
	Mat gray;
	if (image.channels() == 3){
		cvtColor(image, gray, CV_BGR2GRAY);
	} else if (image.channels() == 1){
		gray = image.clone();
	} else{
		LOG(INFO) << "Unknown Image Format!";
	}

	uimg img;
	img.chns = gray.channels();
	img.cols = gray.cols;
	img.rows = gray.rows;
	img.data = gray.data;

	suparam.thresh = (suparam.thresh >= 0) ? suparam.thresh : THRES;
	suparam.stride = (suparam.stride > 0 && suparam.stride <= 6) ? suparam.stride : STRIDE;
	suparam.octaves = (suparam.octaves > 0 && suparam.octaves <= 4) ? suparam.octaves : OCTAVES;
	suparam.intervals = (suparam.intervals > 0 && suparam.intervals <= 4) ? suparam.intervals : INTERVALS;

	fimg ii_img;
	ii_img.data = (float*)calloc(img.cols * img.rows * img.chns, sizeof(float));
	compute_integral(img, ii_img);

	vector<Ipoint> ipts;
	for (int i = 0; i < points.size(); ++i){
		Ipoint ipt;
		ipt.x = points[i].x;
		ipt.y = points[i].y;
		ipt.s = scales[i];
		ipts.push_back(ipt);
	}

	surfDescriptors(ii_img, ipts);
	free(ii_img.data);

	for (int i = 0; i < points.size(); ++i){
		vector<float> descriptor;
		for (int j = 0; j < 64; ++j){
			descriptor.push_back(ipts[i].descriptor[j]);
		}
		descriptors.push_back(descriptor);
	}
}

}
