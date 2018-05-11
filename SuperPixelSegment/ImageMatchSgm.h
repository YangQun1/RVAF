/*****************************************************************************/
/*                                                                           */
/*                            ���ɵ�ͼ����ƽ̨                             */
/*                                                                           */
/* FileName: ImageMatchSgm.h 			                                     */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2015/05/05                                                          */
/*                                                                           */
/* Description: Sgm�㷨                                                      */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/
#ifndef _IMAGE_MATCH_SGM_H_
#define _IMAGE_MATCH_SGM_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <string.h>

#include "ctmf.h"
#include "ImageGradient.h"
#include "ImageMatchCost.h"
#include "ImageMatchAggregation.h"
#include "ImageMatchPostprocess.h"

typedef struct tagImSgmInfo
{
    // �ⲿ��Ϣ
    int imagenum;
    int height;
    int width;
    int channel;
    int dlength;
    IMAGE_S *src[2];

    // ����
    int mode;
    int dispmr;         // ����label����ֵ�˲��뾶
    float r1;			// P1�ͷ�
    float r2;			// P2�ͷ�

    // �ڲ���Ϣ
    ushort *imagecost;
    ImaScanTreeUshortInfo aggr;
    ImppInvalidInfo invalid;

    // ����Ӳ�
    PIXEL *left;
    PIXEL *right;
    PIXEL *sparse;
    PIXEL *fill;
    PIXEL *dense;

}ImSgmInfo;

extern void ImageMatchSgmPara(INOUT ImSgmInfo *Sgm);
extern void ImageMatchSgmInit(INOUT ImSgmInfo *Sgm);
extern void ImageMatchSgmDestroy(INOUT ImSgmInfo *Sgm);
extern void ImageMatchSgmProc(INOUT ImSgmInfo *Sgm);
extern void SgmGradient(INOUT ImSgmInfo *Sgm);

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
