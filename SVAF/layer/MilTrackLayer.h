/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "Layer.h"
#include "AdaboostLayer.h"
#include "..\..\MilTrack\milsvaf.h"

namespace svaf{

typedef struct BoostTrack_{
	cv::Mat img;					// 待追踪的图像（缩放之后）
	cv::Size trsize;				// 待追踪的图像大小（缩放之后）
	pc::Rect rect;					// 目标位置（缩放之后）
	pc::TrackParam trparam;			// 追踪器参数（包含弱分类器、使用的特征等）
	pc::FeatureParam ftrparam;		// 特征参数
	float	scalefactor_x_;
	float	scalefactor_y_;
} BoostTrack;

class MilTrackLayer :
	public Layer
{
public:
	explicit MilTrackLayer(LayerParameter& layer);
	~MilTrackLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

	static bool	reinit_;										// 用于指示是否进行重新初始化

protected:
	bool InitFirstFrame(vector<Block>&, LayerParameter&);
	bool TrackFrame(vector<Block>&);
	void InitSetParam(BoostTrack&);
	void ComputeScale(int);
	void RecoverScale(vector<Block>&, vector<Block>&);

private:
	AdaboostLayer *adaboost;									// 使用AdaBoost检测进行初始化时用到的检测器
	pc::TrackFun track_frame, track_firstframe;					// 函数指针，可以指向online MILBoost或者online AdaBoost两种追踪方法


	vector<BoostTrack> trackers_;
	int		trackcount_;
	

	Size	trsize_;											// 指定的被追踪图像size
	vector<Size> insize_;										// 输入图像的size
	vector<Size> winsize_;										// 目标窗口的size
	vector<float> x_factor_;									// 通过trsize和源图像大小得到的xy缩放因子
	vector<float> y_factor_;
	float	scalefactor_;										// 指定的被追踪图像缩放因子，xy方向缩放相同比例
	

	MilTrackParameter_InitType init_type_;						// 初始化方式（鼠标、全局的detection、配置文件、默认图像中心）			
	MilTrackParameter_TrackType track_type_;					// 追踪类型（MIL、AdaBoost等）

	cv::Rect	init_rect_;										// 待追踪的目标在第一帧中的位置
	vector<cv::Rect> init_rects_;
	// 下面参数可以在pbf文件中指定
	int		init_negnum_;
	int		negnum_;
	int		posmax_;
	int		numsel_;
	int		numfeat_;											// tracker的特征数量
	int		srchwinsz_;
	int		min_rectnum_;
	int		max_rectnum_;
	int		negsample_strat_;

	float	lrate_;
	float	posrad_;
	float	init_posrad_;

	bool	uselogR_;
};

}

