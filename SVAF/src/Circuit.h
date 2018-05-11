/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once

#include <opencv2\opencv.hpp>

#include "Param.h"
#include "Figures.h"
#include <windows.h>

using namespace std;
using namespace cv;

namespace svaf{

class Layer;

template<typename T>
struct _Node{
	T		name;
	Layer*	layer;
	void*	param;
	_Node*	next;
	_Node(T str):name(str), next(NULL), layer(NULL), param(NULL){}
};
typedef _Node<string> Node;

typedef struct _World{
	bool		rectified;			// ʲô���ã�
	int			fetchtype;			// ʲô���ã�
	double		x, y, z, a, b, c; // ץȡ�㣬��ץȡ������������

	vector<Point2f> xl, xr;
	vector<Point3f> pointL, pointR;
	vector<Point3f> pointW; // Left Camera -> world

	vector<Point3f> matchpt1;
	vector<Point3f> matchpt0;
	vector<Point2f> matchpt2;
} World;

struct Color3f{
	float r;
	float g;
	float b;
	Color3f(){}
	Color3f(float rr, float gg, float bb) : r(rr), g(gg), b{ bb }{}
};

// ���ڴ洢һ��ͼ�񣬺ʹӸ�ͼ���еõ��ĸ������ݣ��ؼ��㡢�����ӵȣ�
typedef struct _Block{
	string	name;
	Mat		image;
	Rect	roi;
	// �Զ����������ĸ�ʽ
	vector<Point2f>	points;
	vector<float>	points_sc;
	vector<vector<float>>	despciptors;
	// opencv��������ĸ�ʽ
	vector<KeyPoint>	keypoint;		// �ؼ���
	Mat					descriptors;	// ������
	vector<DMatch>		matches;		// ƥ����

	_Block*	pMatch;
	vector<int>		ptidx;				// ƥ�������һ��ͼ���������б��е�����
	vector<Point3f>	point3d;
	vector<Color3f> color3d;

	// roi center reverse
	bool useroi;
	double xct, yct;

	bool isSave;
	bool isShow;
	bool isOutput;
	bool isOutput3DPoint;

	_Block(string str, Mat& mat, bool isshow = true, bool issave = false, bool isout = false) 
		:name(str), image(mat), isShow(isshow), isSave(issave), isOutput(isout), isOutput3DPoint(false), pMatch(NULL),
		roi(Rect(0, 0, mat.cols, mat.rows)){}
	_Block() : roi(Rect(0, 0, 0, 0)), isShow(true), isSave(false), isOutput(false), isOutput3DPoint(false), pMatch(NULL){}
} Block;

// ���ʣ����ö�����͵ĺ�����ʲô��
enum SvafApp{
	NONE = 0, // gui_type = PROTO
	S_SHOW = 1, // ONE
	B_SHOW = 2, // TWO
	S_RECTIFY = 3, // ONE
	B_RECTIFY = 4, // TWO
	S_DETECT = 5, // ONE
	B_DETECT = 6, // TWO
	//S_TRACK = 7, // ONE
	//B_TRACK = 8, // TWO
	S_POINT = 9, // ONE
	B_POINT = 10, // TWO
	S_POINTDESP = 11, // ONE
	B_POINTDESP = 12, // TWO
	S_SUPIX = 13, // ONE
	B_SUPIX = 14, // TWO

	POINT_MATCH = 21, // ONE_BIG
	RANSAC_MATCH = 22, // ONE_BIG
	STEREO_MATCH = 23, // FOUR
	PC_TRIANGLE = 24, // THREE_BIG
	PC_MULMATRIX = 25, // THREE_BIG
	PC_REGISTRATION = 26, // FOUR
	PR_CENTER = 27, // TWO

	SITCH = 31 // THREE_BIG
};

class Circuit
{
public:
	explicit Circuit(SvafTask&, bool);
	~Circuit();

	static string time_id_;
	void RLOG(std::string);

protected:
	void Build();
	void Run();
	void RunStep();
	void InitStep();
	void EndStep();
	bool Disp();
	void Analysis();
	bool ReciveCmd();
	void SendData();

protected:
	cv::VideoCapture cap_[2];

	bool			useMapping_;
	bool			guiMode_;
	HANDLE			c_fileMapping_;//cmd
	HANDLE			c_mutex_;
	LPTSTR			c_pMsg_;
	HANDLE			d_fileMapping_;//data
	HANDLE			d_mutex_;
	LPTSTR			d_pMsg_;
	HANDLE			i_fileMapping_;//info
	HANDLE			i_mutex_;
	LPTSTR			i_pMsg_;

private:
	int			pause_ms_;
	Param		layers_;
	Block		block_;
	SvafTask	svaf_;
	Node*		linklist_;
	vector<Block>	images_;  // always clear begin frame;
	vector<Block>	disp_;

	World		world_;
	unsigned __int64 id_;
	static Figures<float> sout_;

};

}
