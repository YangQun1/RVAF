/*
 *	chnsPyramid.h
 *		for VisualStudio2012 & opencv3.0.0
 *
 *	Created on:	Aug 2,	2015
 *		Author:	Peng Chao
 *
 */

#ifndef CHNSPYRAMID_H_
#define CHNSPYRAMID_H_

#include <stdint.h>
#include <vector>

#include <opencv2\opencv.hpp>

#include "chnsCompute.h"

using namespace std;
using namespace cv;

namespace pc{


typedef struct
{
	ChannelsOpt	chnsOpt;
	int32_t		nPerOct;	// ��������ͬһ��octave���ж��ٸ��㣨an octave is the set of scales up to half of the initial scale��
	int32_t		nOctUp;		// ��ԭʼͼ��߶ȴ��octave
	int32_t		nApprox;	// ÿ��octave���ж��ٸ��м���������ý��Ƽ���
	vector<float> lambdas;	// The parameter "lambdas" determines how the channels are normalized
	Size		pad;		// controls the amount the channels are padded after being created(useful for detecting objects near boundaries)
	Size		minDs;		// smallest scale of the pyramid
	int32_t		smooth;		// controls the amount of smoothing after the channels are created(and controls the integration scale of the channels)
} PyramidOpt;

typedef struct
{
	int32_t		nTypes;
	int32_t		nScales;
	vector<Channels> data;
	vector<float> lambdas;
	vector<float> scales;
	vector<float> scalesHt;
	vector<float> scalesWd;
} Pyramid;

void ChnsPyramid(Mat& src, PyramidOpt& opt, Pyramid& pyramid);
void ChnsPyramidScale(Mat& src, PyramidOpt& opt, Pyramid& pyramid, int scaleindex);
}


#endif /* CHNSPYRAMID_H_ */