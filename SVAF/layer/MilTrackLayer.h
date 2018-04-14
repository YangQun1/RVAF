/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "Layer.h"
#include "AdaboostLayer.h"
#include "..\..\MilTrack\milsvaf.h"

namespace svaf{

typedef struct BoostTrack_{
	cv::Mat img;					// ��׷�ٵ�ͼ������֮��
	cv::Size trsize;				// ��׷�ٵ�ͼ���С������֮��
	pc::Rect rect;					// Ŀ��λ�ã�����֮��
	pc::TrackParam trparam;			// ׷������������������������ʹ�õ������ȣ�
	pc::FeatureParam ftrparam;		// ��������
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

	static bool	reinit_;										// ����ָʾ�Ƿ�������³�ʼ��

protected:
	bool InitFirstFrame(vector<Block>&, LayerParameter&);
	bool TrackFrame(vector<Block>&);
	void InitSetParam(BoostTrack&);
	void ComputeScale(int);
	void RecoverScale(vector<Block>&, vector<Block>&);

private:
	AdaboostLayer *adaboost;									// ʹ��AdaBoost�����г�ʼ��ʱ�õ��ļ����
	pc::TrackFun track_frame, track_firstframe;					// ����ָ�룬����ָ��online MILBoost����online AdaBoost����׷�ٷ���


	vector<BoostTrack> trackers_;
	int		trackcount_;
	

	Size	trsize_;											// ָ���ı�׷��ͼ��size
	vector<Size> insize_;										// ����ͼ���size
	vector<Size> winsize_;										// Ŀ�괰�ڵ�size
	vector<float> x_factor_;									// ͨ��trsize��Դͼ���С�õ���xy��������
	vector<float> y_factor_;
	float	scalefactor_;										// ָ���ı�׷��ͼ���������ӣ�xy����������ͬ����
	

	MilTrackParameter_InitType init_type_;						// ��ʼ����ʽ����ꡢȫ�ֵ�detection�������ļ���Ĭ��ͼ�����ģ�			
	MilTrackParameter_TrackType track_type_;					// ׷�����ͣ�MIL��AdaBoost�ȣ�

	cv::Rect	init_rect_;										// ��׷�ٵ�Ŀ���ڵ�һ֡�е�λ��
	vector<cv::Rect> init_rects_;
	// �������������pbf�ļ���ָ��
	int		init_negnum_;
	int		negnum_;
	int		posmax_;
	int		numsel_;
	int		numfeat_;											// tracker����������
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

