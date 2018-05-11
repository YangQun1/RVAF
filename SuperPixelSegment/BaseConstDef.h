/*****************************************************************************/
/*                                                                           */
/*                            ���ɵ�ͼ����ƽ̨                             */
/*                                                                           */
/* FileName: BaseConstDef.h			                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: ���峣�����ݺ�                                               */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*****************************************************************************/

#ifndef _BASE_CONST_DEF_H_
#define _BASE_CONST_DEF_H_
//#include <math.h>
/******************************************************************************
 *                              ƽ̨����
******************************************************************************/
#define PLATFORM_VC6
#define PLATFORM_VS2010
#define PLATFORM_SSE 
// #define PLATFORM_GCC
// #define PLATFORM_CCS

#ifdef PLATFORM_SSE
#include <emmintrin.h>
#endif

/******************************************************************************
 *                              �������Ͷ���
******************************************************************************/

// һ������µĸ�������(1 float or 2 double)
#define FLOAT_TYPE_GEN              2

// �ָ�ģ���IQ�Ż�
#define OPT_SEG_IQ                  0
#define FLOAT_TYPE_SEG              2
#if OPT_SEG_IQ == 1
#undef FLOAT_TYPE_SEG
#define FLOAT_TYPE_SEG              0
#endif
#define QFACTOR_REGION_WEIGHT       23

// costģ���IQ�Ż�(��ʱ��֧��)������float��double���һ��
#define OPT_COST_IQ                 0
#define FLOAT_TYPE_COST             1
#if OPT_COST_IQ == 1
#undef FLOAT_TYPE_COST
#define FLOAT_TYPE_COST             0
#endif
#define QFACTOR_COST_WEIGHT         23

// fittingģ���IQ�Ż�(��ʱ��֧��)������float��double���һ��
#define OPT_FITTING_IQ              0
#define FLOAT_TYPE_FITTING          1
#if OPT_FITTING_IQ == 1
#undef FLOAT_TYPE_FITTING
#define FLOAT_TYPE_FITTING          0
#endif
#define QFACTOR_FITTING_WEIGHT      23

// disp��IQ�Ż�(��ʱ��֧��)������float��double���һ��
#define OPT_DISP_IQ                 0
#define FLOAT_TYPE_DISP             1
#if OPT_DISP_IQ == 1
#undef FLOAT_TYPE_DISP
#define FLOAT_TYPE_DISP             0
#endif
#define QFACTOR_DISP_WEIGHT         23


// ����ģ��

// weight = avg(d) - T������0��ֱ���ų���T <= 1     >>>>>  ��Χ [-1 0)
#define QFACTOR_LAYER_WEIGHT        30

// weight = cost�ı仯��[-n n]                      >>>>>  ��Χ (-100 100)
#define QFACTOR_FRAGMENT_WEIGHT     24

// weight = cost�ı仯 [-n 0) (end��n�ϴ�base��n < 100)
#define QFACTOR_ENDRELABEL_WEIGHT       24
#define QFACTOR_BASERELABEL_WEIGHT      24


// weight = -soweight (���޷�Χ����ȷ����ʵ��Ӧ�ý�С)
#define QFACTOR_BORDER_WEIGHT       24

/******************************************************************************
 *                  ����������˵�����������������������
******************************************************************************/

/* ������� */
#ifndef IN
#define IN
#endif

/* ������� */
#ifndef OUT
#define OUT
#endif

/* ������Ҳ������� */
#ifndef INOUT
#define INOUT
#endif

/******************************************************************************
 *                              �������Ͷ���
******************************************************************************/
#ifndef INLINE
#define INLINE  static __inline
#endif

#ifndef EXPORT
#define EXPORT  extern
#endif

/******************************************************************************
 *                              ���峣�÷���ֵ
******************************************************************************/
#ifndef TRUE
#define TRUE                1
#endif

#ifndef FALSE
#define FALSE               0
#endif

/******************************************************************************
 *                              ���峣�����ݺ�
******************************************************************************/

/* 0ָ�� */
#ifndef NULL
#define NULL            0
#endif

/* ��Ч���� */
#ifndef INDEX_NULL
#define INDEX_NULL      -1
#endif

/* Բ���ʦ� */
#ifndef MATH_PI
#define MATH_PI         3.14159265358979
#endif

/* 2�� */
#ifndef MATH_2PI
#define MATH_2PI        6.28318530717958
#endif

/* ��Ȼ����e */
#ifndef MATH_E
#define MATH_E          2.71828182845904
#endif

/* ����ln(2) */
#ifndef MATH_LN2
#define MATH_LN2        0.693147180559945
#endif

/* sqrt(2) */
#ifndef MATH_SQRT2
#define MATH_SQRT2      1.414213562373095
#endif

/* sqrt(��) */
#ifndef MATH_SQRTPI
#define MATH_SQRTPI     1.772453850905516
#endif

/* ���㾫��(��) */
#ifndef FLOAT_EPS
#define FLOAT_EPS       1.0e-6
#endif
#ifndef DOUBLE_EPS
#define DOUBLE_EPS      1.0e-15
#endif

/* ������� */
#ifndef DOUBLE_INF
#define DOUBLE_INF      1.0e38
#endif

/* ������� */
#ifndef INT_INF
#define INT_INF         0x7FFFFFFF
#endif

#ifndef INT16_INF
#define INT16_INF       0x7FFF
#endif

#ifndef INT16_INF2
#define INT16_INF2      0x7FFF7FFF
#endif

/* memset�����������������int, float, double������ */
#ifndef MEMSET_INF
#define MEMSET_INF      0x7f
#endif

/******************************************************************************
 *                                  ͼ��ɫ��
******************************************************************************/

/* ͼ��Ҷȼ� */
#ifndef IMAGE_LEVEL
#define IMAGE_LEVEL     256
#endif

/* �Ҷ�ͼ���ɫ */
#ifndef IMAGE_BLACK
#define IMAGE_BLACK     0
#endif

/* �Ҷ�ͼ���ɫ */
#ifndef IMAGE_WHITE
#define IMAGE_WHITE     255
#endif

/* �Ҷ�ͼ���ɫ */
#ifndef IMAGE_GREY
#define IMAGE_GREY      128
#endif

/* RGBͼ���ɫ */
#ifndef IMAGE_CVALUE_BLACK
#define IMAGE_CVALUE_BLACK      0,0,0
#endif

/* RGBͼ���ɫ */
#ifndef IMAGE_CVALUE_WHITE
#define IMAGE_CVALUE_WHITE      255,255,255
#endif

/* RGBͼ����ɫ */
#ifndef IMAGE_CVALUE_BLUE
#define IMAGE_CVALUE_BLUE       255,0,0
#endif

/* RGBͼ����ɫ */
#ifndef IMAGE_CVALUE_GREEN
#define IMAGE_CVALUE_GREEN      0,255,0
#endif

/* RGBͼ���ɫ */
#ifndef IMAGE_CVALUE_RED
#define IMAGE_CVALUE_RED        0,0,255
#endif

/* RGBͼ����ɫ */
#ifndef IMAGE_CVALUE_PURPLE
#define IMAGE_CVALUE_PURPLE     255,0,255
#endif

/* RGBͼ���ɫ */
#ifndef IMAGE_CVALUE_YELLOW
#define IMAGE_CVALUE_YELLOW     0,255,255
#endif

/* RGBͼ����ɫ */
#ifndef IMAGE_CVALUE_CYAN
#define IMAGE_CVALUE_CYAN       255,255,0
#endif

#endif
