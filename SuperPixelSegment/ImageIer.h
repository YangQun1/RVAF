/*****************************************************************************/
/*                                                                           */
/*                            ���ɵ�ͼ����ƽ̨                             */
/*                                                                           */
/* FileName: ImageIer.h 	                                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2013/07/02                                                          */
/*                                                                           */
/* Description: ͼ��ָ��㷨 Ier                                             */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef _IMAGE_IER_H_
#define _IMAGE_IER_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <string.h>

#include "VectorBasic.h"

// ���͡���������
typedef double              floatier;
#define IER_INF             DOUBLE_INF
#define VectorDistIer       VectorDistDouble
#define VectorPlusIer       VectorPlusDouble
#define VectorSubIer        VectorSubDouble
#define VectorMulConstIer   VectorMulConstDouble


#define IERI_INF            INT_INF
#define VectorDistIeri      VectorDistInt
#define VectorPlusIeri      VectorPlusInt
#define VectorSubIeri       VectorSubInt
#define VectorMulConstIeri  VectorMulConstInt
/*
    Q ����˵��
    Q(distc) = Q(distxy)+Q(distwxy)
    Q(distc) = 2*Q(avgc), Q(distxy) = 2*Q(avgxy)
    max(distc) = 16 >>> Q(distc) = 31-16 = 15 >>> Q(avgc) = 7
    >>> Q(avgxy) = 3, Q(distwxy) = 8
*/
#define IER_Q_AVGC              7
#define IER_Q_AVGXY             3
#define IER_Q_AVGWXY            8
#define IER_S_AVGC              128
#define IER_S_AVGXY             8
#define IER_S_AVGWXY            256

#define IER_MAX_NBNUM           5


// ��Ե���ض���
typedef struct tagIerEdgeQueue
{
    POINT3D_S *curQueue;
    POINT3D_S *nextQueue;
    int curQueueNum;
    PIXEL *queueflag;
    int *labelflag;
    int *changeflag;
    int *changeIndex;
    int changeNum;

    int *nbIndex;
    int *nbNumber;

}IerEdgeQueue;

// С������ͨ����
typedef struct tagIerConnectInfo
{
    int *conx;
    int *cony;
    int *conxy;

}IerConnectInfo;

typedef struct tagMeshSplitInfo
{
    int S;		// ����ı߳�����λΪ���أ�
    int n;		// ���ֵ���������
    int n1;		// n1=n-n2,��1~n1�����ԣ��߳�ΪS��
				// n1��n��n2�����ر߳�ΪS+1�������������ܻ�����
    int n2;		// n2=width-n*S,
				// ��������ճ�ʼ�趨�ĸ��Ա߳����֣�
				// ��������Ĳ��ܱ�����Ϊ�������ӵ����ظ���

}MeshSplitInfo;

// ier�ṹ
typedef struct tagIerInfo
{
    // �ⲿ������Ϣ
    int width;                  // ͼ����
    int height;                 // ͼ��߶�
    int channel;                // ��ɫͼ��ͨ����
    floatier *imagecolor;       // ��ɫͼ��
    PIXEL *imagecolori;         // ��ɫͼ��
    
    // ����
    int optint;
    int K;                  // Superpixel number
    int S;                  // sqrt(Superpixel size) �������Ĳ������
    floatier normC;         // ��ɫ��һ������ (1-40)
    floatier normS;         // �ռ��һ�����ӣ���ʼ���������Ĳ������
    int maxiter;

    // ��������Ϣ
    int *label;

    // ��������
    int num;		// �������ĸ���
    int *sumn;		// ���������صĸ���
    int *sumx;		// �������������ص�x����ĺ�
    int *sumy;		// �������������ص�y����ĺ�


    floatier *sumc;	// �������������ص���ɫֵ�ĺ�	
    floatier *avgc;	// �������������ص���ɫֵ��ƽ��ֵ
    floatier *avgx;	// �������������ص�x�����ƽ��ֵ
    floatier *avgy;	// �������������ص�y�����ƽ��ֵ
    floatier wxy;
    
    int *avgxi;		// ����ͬ�ϣ��ö������洢
    int *avgyi;
    int *sumci;
    int *avgci;
    int wxyi;

    // �ڲ���ʱ�ڴ�
    int *newlabel;
    IerEdgeQueue edgeQueue;
    IerConnectInfo connect;

    // �ڴ����
    int maxK;                   // ���ָ�������Ŀ

}IerInfo;


extern void ImageIerPara(OUT IerInfo *ier);
extern void ImageIerInit(OUT IerInfo *ier);
extern void ImageIerDestroy(OUT IerInfo *ier);
extern void IerMemoryAdapt0(IN IerInfo *ier, IN int K, IN int type);
extern void IerStatInit(OUT IerInfo *ier);
extern void IerStatDestroy(OUT IerInfo *ier);
extern void IerEdgeQueueInit(OUT IerEdgeQueue *Q, IN int size, IN int K);
extern void IerEdgeQueueDestroy(OUT IerEdgeQueue *Q, IN int size, IN int K);
extern void IerEdgeQueueInit0(OUT IerEdgeQueue *Q, IN int K);
extern void IerEdgeQueueDestroy0(OUT IerEdgeQueue *Q, IN int K);
extern void IerConnectInfoInit(OUT IerConnectInfo *connect, IN int size, IN int K);
extern void IerConnectInfoDestroy(OUT IerConnectInfo *connect, IN int size, IN int K);


extern void IerPixelInit(IN IerInfo *ier);
extern void IerPixelRefine(IN IerInfo *ier);
extern void IerWeight(IN IerInfo *ier);
extern void IerEdgeInitDetect(IN IerInfo *ier, IN int off);
extern void IerEdgeReDetect(IN IerInfo *ier);
extern void IerEdgeRefine(IN IerInfo *ier);
extern void IerEdgeRefineInt(IN IerInfo *ier);
extern void IerEnforceConnectivity(IN IerInfo *ier);


extern void MeshSplitSize(INOUT MeshSplitInfo *sp, IN int S0, IN int width);
extern void IerMeshStat(INOUT IerInfo *ier, 
                        IN int Sx,
                        IN int Sy,
                        IN int nx,
                        IN int ny,
                        IN int nx1,
                        IN int ny1);
extern void IerMeshStatInt(INOUT IerInfo *ier, 
                           IN int Sx,
                           IN int Sy,
                           IN int nx,
                           IN int ny,
                           IN int nx1,
                           IN int ny1);

extern void IerFlagInit(PIXEL *label, int width, int height, PIXEL neg);
extern void IerLabelInit(int *label, int width, int height, int neg);
extern void IerLabelExtend(int *inl, int *outl, int width, int height);
extern void IerLabelDeextend(int *inl, int *outl, int width, int height);


extern void ImageSegShow(IN IMAGE_S *srcImage, 
                         IN int *flag, 
                         IN PIXEL *contour,
                         OUT IMAGE_S *showImage);

extern void IerEdgeRefineNeighbor1(IN IerInfo *ier);
extern void IerEdgeRefineIntChannel1(IN IerInfo *ier);
extern void IerEdgeRefineIntColor1(IN IerInfo *ier);

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
