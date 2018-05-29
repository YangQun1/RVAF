/*
 * acfDetect.h
 *  for VisualStudio & opencv3.0.0
 *
 *  Created on: Aug 2, 2015
 *		Author: Peng Chao
 *	
 */

#ifndef ACFDETECT_H_
#define ACFDETECT_H_

#include <stdint.h>
#include <vector>

#include "chnsPyramid.h"

using namespace std;

namespace pc{

typedef struct _DetectResult
{
	float cs;
	float modelWd;
	float rs;
	float modelHt;
	float hs;
	bool operator > (const _DetectResult& rhs) const
	{
		return hs > rhs.hs;
	}
	int	 scaleindex;
} DetectResult;

class AcfDetector
{
public:
	uint32_t* fids;						// [K x nWeak] feature id for each nodes
	float*    thrs;						// [K x nWeak] threshold corresponding to each fids
	uint32_t* child;					// [K x nWeak] index of (right)child for each node£¬if child == 0,this node is a leaf node
	float*    hs;						// [K x nWeak] log ratio (.5*log(p/(1-p)) at each node
	float*    weights;					// [K x nWeak] total sample weight at each node
	uint32_t* depth;					// [K x nWeak] depth of each node
	uint32_t  treeDepth;				// depth of all leaf nodes (or 0 if leaf depth varies)
	uint32_t  nTrees, nTreeNodes;		// nTrees the number of decision trees,nTreeNodes the node number of one decision tree
	uint32_t  stride;					// spatial stride between detection windows
	int32_t   cascThr;					// constant cascade threshold (affects speed/accuracy)
	int32_t   modelHt, modelWd;			// model height+width without padding (eg [100 41])
	int32_t   modelPadHt, modelPadWd;	// model height+width with padding (eg [128 64])

	AcfDetector(char* path);
	AcfDetector(){};
	void Open(char* path);
	~AcfDetector();

	void Detect(Channels& data, int32_t shrink, vector<DetectResult>& result, float epipolarLine = -10);
};

void AcfDetectImg(Mat& img, PyramidOpt& opt, AcfDetector& detector, vector<DetectResult>& result, float nms = 0.65);
void AcfDetectImgScale(Mat& img, PyramidOpt& opt, AcfDetector& detector, vector<DetectResult>& result, int scaleindex, float epipolarLine = -10, float nums = 0.65);

}
#endif /* ACFDETECT_H_ */
