/*****************************************************************************/
/*                                                                           */
/*                            ���ɵ�ͼ����ƽ̨                             */
/*                                                                           */
/* FileName: ImageIer.c 	                                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2013/07/02                                                          */
/*                                                                           */
/* Description: ͼ��ָ��㷨 IER                                             */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "ImageIer.h"
#include "math.h"
#include "stdio.h"

void ImageIerPara(OUT IerInfo *ier)
{
    ier->optint = 0;
    ier->K = 200;
    ier->S = 10;
    ier->normC = 10;
    ier->maxiter = 30;

    ier->width = 500;
    ier->height = 500;
    ier->channel = 3;
    ier->maxK = 5000;

    return;
}

void ImageIerInit(OUT IerInfo *ier)
{
    int width = ier->width;
    int height = ier->height;
    int size = width*height;
    int ensize = (width+2)*(height+2);
    int K = ier->maxK;

    ier->label = MallocType(int, ensize);
    ier->newlabel = MallocType(int, ensize);

    IerStatInit(ier);
    IerEdgeQueueInit(&ier->edgeQueue, size, K);
    IerConnectInfoInit(&ier->connect, size, K);
}

void ImageIerDestroy(OUT IerInfo *ier)
{
    int width = ier->width;
    int height = ier->height;
    int size = width*height;
    int ensize = (width+2)*(height+2);
    int K = ier->maxK;

    FreeType(ier->label, int, ensize);
    FreeType(ier->newlabel, int, ensize);
    IerStatDestroy(ier);
    IerEdgeQueueDestroy(&ier->edgeQueue, size, K);
    IerConnectInfoDestroy(&ier->connect, size, K);
}

void IerMemoryAdapt0(IN IerInfo *ier, IN int K, IN int type)
{
    int maxK, oldmaxK;
    IerEdgeQueue *Q;

    int *tempint;
    floatier *tempfloat;

    oldmaxK = ier->maxK;
    if (K < oldmaxK)        return;

    if (type == 1)  maxK = 2*K;
    else            maxK = K;
    Q = &ier->edgeQueue;

    // ��Ϣ����
    if (type == 1)
    {
        tempint = MallocType(int, 6*K);
        tempfloat = MallocType(floatier, K*ier->channel);

        memcpy(tempint+3*K, ier->sumn, K*sizeof(int));
        memcpy(tempint+4*K, ier->sumx, K*sizeof(int));
        memcpy(tempint+5*K, ier->sumy, K*sizeof(int));
        memcpy(tempfloat, ier->sumc, K*ier->channel*sizeof(floatier));
    }

    // �����ڴ�
    IerStatDestroy(ier);
    ier->maxK = maxK;
    IerStatInit(ier);
    IerEdgeQueueDestroy0(Q, oldmaxK);
    IerEdgeQueueInit0(Q, maxK);

    // ��Ϣ��ԭ
    if (type == 1)
    {
        memcpy(ier->sumn, tempint+3*K, K*sizeof(int));
        memcpy(ier->sumx, tempint+4*K, K*sizeof(int));
        memcpy(ier->sumy, tempint+5*K, K*sizeof(int));
        memcpy(ier->sumc, tempfloat, K*ier->channel*sizeof(floatier));

        FreeType(tempint, int, 6*K);
        FreeType(tempfloat, floatier, K*ier->channel);
    }
}

void IerStatInit(OUT IerInfo *ier)
{
    int K = ier->maxK;
    int KC = K*ier->channel;
    int optint = ier->optint;

    ier->sumn = MallocType(int, K);
    ier->sumx = MallocType(int, K);
    ier->sumy = MallocType(int, K);

    if (optint == 0)
    {
        ier->avgx = MallocType(floatier, K);
        ier->avgy = MallocType(floatier, K);
        ier->sumc = MallocType(floatier, KC);
        ier->avgc = MallocType(floatier, KC);
    }
    else
    {
        ier->avgxi = MallocType(int, K);
        ier->avgyi = MallocType(int, K);
        ier->sumci = MallocType(int, KC);
        ier->avgci = MallocType(int, KC);
    }
}
void IerStatDestroy(OUT IerInfo *ier)
{
    int K = ier->maxK;
    int KC = K*ier->channel;
    int optint = ier->optint;

    FreeType(ier->sumn, int, K);
    FreeType(ier->sumx, int, K);
    FreeType(ier->sumy, int, K);

    if (optint == 0)
    {
        FreeType(ier->avgx, floatier, K);
        FreeType(ier->avgy, floatier, K);
        FreeType(ier->sumc, floatier, KC);
        FreeType(ier->avgc, floatier, KC);
    }
    else
    {
        FreeType(ier->avgxi, int, K);
        FreeType(ier->avgyi, int, K);
        FreeType(ier->sumci, int, KC);
        FreeType(ier->avgci, int, KC);
    }
}
void IerEdgeQueueInit(OUT IerEdgeQueue *Q, IN int size, IN int K)
{
    Q->curQueue = MallocType(POINT3D_S, size);
    Q->nextQueue = MallocType(POINT3D_S, size);
    Q->queueflag = MallocType(PIXEL, 2*size);
    Q->changeIndex = MallocType(int, size);
    Q->nbIndex = MallocType(int, IER_MAX_NBNUM*size);
    Q->nbNumber = MallocType(int, size);    
    IerEdgeQueueInit0(Q, K);
}
void IerEdgeQueueDestroy(OUT IerEdgeQueue *Q, IN int size, IN int K)
{
    FreeType(Q->curQueue, POINT3D_S, size);
    FreeType(Q->nextQueue, POINT3D_S, size);
    FreeType(Q->queueflag, PIXEL, 2*size);
    FreeType(Q->changeIndex, int, size);
    FreeType(Q->nbIndex, int, IER_MAX_NBNUM*size);
    FreeType(Q->nbNumber, int, size);
    IerEdgeQueueDestroy0(Q, K);
}
void IerEdgeQueueInit0(OUT IerEdgeQueue *Q, IN int K)
{
    Q->labelflag = MallocType(int, K+2);
    Q->changeflag = MallocType(int, K);
}
void IerEdgeQueueDestroy0(OUT IerEdgeQueue *Q, IN int K)
{
    FreeType(Q->labelflag, int, K+2);
    FreeType(Q->changeflag, int, K);
}
void IerConnectInfoInit(OUT IerConnectInfo *connect, IN int size, IN int K)
{
    connect->conx = MallocType(int, size);
    connect->cony = MallocType(int, size);
    connect->conxy = MallocType(int, size);
}
void IerConnectInfoDestroy(OUT IerConnectInfo *connect, IN int size, IN int K)
{
    FreeType(connect->conx, int, size);
    FreeType(connect->cony, int, size);
    FreeType(connect->conxy, int, size);
}

void IerPixelInit(IN IerInfo *ier)
{
    int height = ier->height;
    int width = ier->width;
    int area = height*width;
    int K = ier->K;
    int optint = ier->optint;

    MeshSplitInfo spx, spy;

    // ���㲿�ֲ�����ͳ��
	ier->normS = sqrt(area / (double)K);
    ier->S = (int)ier->normS; 
    MeshSplitSize(&spx, ier->S, width);
    MeshSplitSize(&spy, ier->S, height);
    ier->num = spx.n * spy.n;

    if (optint == 0)    IerMeshStat(ier, spx.S, spy.S, spx.n, spy.n, spx.n1, spy.n1);
    else                IerMeshStatInt(ier, spx.S, spy.S, spx.n, spy.n, spx.n1, spy.n1);

    return;
}

void IerPixelRefine(IN IerInfo *ier)
{
    int i;
    int maxiter;
    int optint = ier->optint;
    IerEdgeQueue *Q = &ier->edgeQueue;

    // Ȩֵ����
    IerWeight(ier);

    // ��Ե�Ż�
    memset(Q->labelflag, FALSE, (ier->num+2)*sizeof(int));
    Q->labelflag[0] = TRUE;
    IerEdgeInitDetect(ier, 1);
    if (optint == 0)    IerEdgeRefine(ier);
    else                IerEdgeRefineInt(ier);

    maxiter = ier->maxiter;
    for (i = 0;i < maxiter && Q->curQueueNum > 0;i++)
    {
        IerEdgeReDetect(ier);
        if (optint == 0)    IerEdgeRefine(ier);
        else                IerEdgeRefineInt(ier);
    }

    IerEnforceConnectivity(ier);
    IerLabelDeextend(ier->label, ier->label, ier->width, ier->height);

    return;
}

void IerWeight(IN IerInfo *ier)
{
    floatier normC = ier->normC;
    floatier normS = ier->normS;

    floatier wxy;

    wxy = (normC*normC)/(normS*normS);
    ier->wxy = wxy;
    ier->wxyi = (int)(wxy * IER_S_AVGWXY);
}

/*
1 ���ݱ��ͼ�񣬽���Ե�������Q
2 �Ӷ���Q��ȡ����p(x,y)�������������ڵľ������ľ��룬�ñ�ǩ
3 ��ǩ�ı䣬����������������Ϊ�µı�Ե������뵽����NQ��
4 ����Q��NQ��������һ�ֵ��� 
*/
void IerEdgeInitDetect(IN IerInfo *ier, IN int off)
{
    int *srclabel = ier->label;
    int width = ier->width;
    int height = ier->height;
    int wl = width + 2*off;

    IerEdgeQueue *edgeQueue = &ier->edgeQueue;
    POINT3D_S *Q = edgeQueue->curQueue;
    PIXEL *queueflag = edgeQueue->queueflag;

    int x, y, xpre;
    int xl, yl, pos;
    int k, l;
    int *label, *pl;
    PIXEL *flag, *pf;

    memset(queueflag, 0, width*height*sizeof(PIXEL));

    // ˮƽ�����Ե
    flag = queueflag;
    label = srclabel + off*wl + off;
    for(y = 0; y < height; y++, flag += width, label += wl)
	{
        k = label[0];
        yl = y + off;
        pos = yl * wl;
		for(x = 1;x < width;x++)
		{
		    l = label[x];
            if (k != l)
            {
                xpre = x-1;
                if (!flag[xpre])
                {
                    xl = xpre + off;
                    Q->x = xpre;
                    Q->y = y;
                    Q->s = pos + xl;
                    Q++;
                }
                xl = x + off;
                Q->x = x;
                Q->y = y;
                Q->s = pos + xl;
                Q++;

                flag[xpre] = 1;
                flag[x] = 1;

                k = l;
            }
		}
    }

    // ��ֱ�����Ե
    flag = queueflag;
    label = srclabel + off*wl + off;
    for(x = 0; x < width; x++, flag++, label++)
	{
	    k = *label;
	    pl = label + wl;
        pf = flag + width;
        xl = x + off;
		for(y = 1;y < height;y++, pl += wl, pf += width)
		{
		    l = *pl;
            if (k != l)
            {
                if (!pf[-width])
                {
                    yl = y-1 + off;
                    Q->x = x;
                    Q->y = y-1;
                    Q->s = yl * wl + xl;
                    Q++;
                }
                if (!pf[0])
                {
                    yl = y + off;
                    Q->x = x;
                    Q->y = y;
                    Q->s = yl * wl + xl;
                    Q++;
                    pf[0] = 1;
                }

                k = l;
            }
		}
    }

    edgeQueue->curQueueNum = Q - edgeQueue->curQueue;

    return;
}

void IerEdgeReDetect(IN IerInfo *ier)
{
    int *label = ier->label;
    int width = ier->width;
    int height = ier->height;
    int wl = width+2;
    int hl = height+2;

    IerEdgeQueue *edgeQueue = &ier->edgeQueue;
    POINT3D_S *curQueue = edgeQueue->curQueue;
    POINT3D_S *nextQueue = edgeQueue->nextQueue;
    PIXEL *queueflag = edgeQueue->queueflag;
    int *changeIndex = edgeQueue->changeIndex;
    int changeNum= edgeQueue->changeNum;
    
    int i, x, y, adj;
    
    int cur, flag;
    int pos;
    int *pl;
    PIXEL *pf;
    POINT3D_S *Q, *nQ;

#define IerAddNeighbor(nx, ny, d)           \
{                                           \
    flag = (pl[d] != cur) & (!pf[d]);       \
    if (flag)                               \
    {                                       \
        nQ->x = nx;                         \
        nQ->y = ny;                         \
        nQ->s = d + pos;                    \
        nQ++;                               \
        pf[d] = 1;                          \
    }                                       \
}

    memset(queueflag, 0, wl*hl*sizeof(PIXEL));
    IerFlagInit(queueflag, width, height, 1);
    for (i = 0, nQ = nextQueue;i < changeNum;i++)
    {
        Q = curQueue + changeIndex[i];
        x = Q->x;
        y = Q->y;
        pos = Q->s;
        pl = label + pos;
        pf = queueflag + pos;
        cur = *pl;

        adj = x - 1;
        IerAddNeighbor(adj, y, -1);

        adj = x + 1;
        IerAddNeighbor(adj, y, 1);

        adj = y - 1;
        IerAddNeighbor(x, adj, -wl);

        adj = y + 1;
        IerAddNeighbor(x, adj, wl);
    }

    edgeQueue->nextQueue = curQueue;
    edgeQueue->curQueue = nextQueue;
    edgeQueue->curQueueNum = nQ - nextQueue;

#undef IerAddNeighbor

    return;
}

void IerEdgeRefine(IN IerInfo *ier)
{
    int *label = ier->label;
    int *newlabel = ier->newlabel;
    int width = ier->width;
    int height = ier->height;
    int wl = width+2;
    int hl = height+2;

    int iernum = ier->num;
    int channel = ier->channel;
    floatier *imagecolor = ier->imagecolor;
    floatier *avgc = ier->avgc;
    floatier *sumc = ier->sumc;
    int *sumn = ier->sumn;
    int *sumx = ier->sumx;
    int *sumy = ier->sumy;
    floatier *avgx = ier->avgx;
    floatier *avgy = ier->avgy;
    floatier wxy = ier->wxy;

    IerEdgeQueue *edgeQueue = &ier->edgeQueue;
    POINT3D_S *Queue = edgeQueue->curQueue;
    int queuenum = edgeQueue->curQueueNum;
    int *labelflag = edgeQueue->labelflag + 2;
    int *changeflag = edgeQueue->changeflag;
    int *changeIndex = edgeQueue->changeIndex;
    int changeNum = 0;

    int i, n, nb;
    int x, y, pos, posl;
    int cur, next, adj;
    POINT3D_S *Q;
    int *pl;
    int nbindex[5];

    floatier *pc;
    floatier *psumc;
    floatier *pavgc;
    floatier cx, cy;
	floatier dx, dy;
    floatier distcolor, distxy;
	floatier dist, mindist;
	floatier norm;

#define IerAddNeighbor()            \
{                                   \
    if (labelflag[adj] == FALSE)    \
    {                               \
        nbindex[nb++] = adj;        \
        labelflag[adj] = TRUE;      \
    }                               \
}

    memcpy(newlabel, label, hl*wl*sizeof(int));
    memset(changeflag, FALSE, iernum*sizeof(int));
    for (i = 0, Q = Queue;i < queuenum;i++, Q++)
    {
        x = Q->x;
        y = Q->y;
        pos = y*width + x;
        posl = Q->s;

        // ��ǰ���
        pl = label + posl;
        cur = *pl;
        labelflag[cur] = TRUE;
        nbindex[0] = cur;
        nb = 1;

        // �����ڱ��
        adj = pl[-1];
        IerAddNeighbor();
        
        adj = pl[1];
        IerAddNeighbor();

        adj = pl[-wl];
        IerAddNeighbor();

        adj = pl[wl];
        IerAddNeighbor();

        // �ر��
        mindist = IER_INF;
        pc = imagecolor + pos*channel;
        for(n = 0; n < nb; n++)
		{
		    adj = nbindex[n];
            labelflag[adj] = FALSE;

            // 1: ��ɫ����
            pavgc = avgc + adj*channel;
            distcolor = VectorDistIer(pc, pavgc, channel, VECTOR_DIST_EUCLIDEAN);

            // 2: ���ξ���
            cx = avgx[adj];
            cy = avgy[adj];
    		dx = x - cx;
    		dy = y - cy;
            distxy = dx*dx + dy*dy;

            dist = distcolor + distxy*wxy;

    		if( dist < mindist )
    		{
    			mindist = dist;
    			next = adj;
    		}
		}

        if (next == cur) continue;

        // ��¼�ر�ǩ
        newlabel[posl] = next;
        changeIndex[changeNum++] = i;

        // ����ͳ����Ϣ
	    sumn[cur]--;
	    sumx[cur] -= x;
	    sumy[cur] -= y;
        psumc = sumc + cur*channel;
        VectorSubIer(psumc, pc, psumc, channel);

        sumn[next]++;
	    sumx[next] += x;
	    sumy[next] += y;
	    psumc = sumc + next*channel;
        VectorPlusIer(psumc, pc, psumc, channel);

	    changeflag[next] = TRUE;
        changeflag[cur] = TRUE;
    }

    // ���¾�������
    psumc = sumc;
    pavgc = avgc;
    for(n = 0; n < iernum; n++, psumc += channel, pavgc += channel)
    {
        if (changeflag[n] == FALSE)     continue;

        norm = (floatier)(1.0/sumn[n]);
        avgx[n] = sumx[n]*norm;
        avgy[n] = sumy[n]*norm;
        VectorMulConstIer(psumc, pavgc, norm, channel);
    }

    // ���±�ǩ
    edgeQueue->changeNum = changeNum;
    ier->label = newlabel;
    ier->newlabel = label;

#undef IerAddNeighbor

    return;
}

void IerEdgeRefineIntChannel(IN IerInfo *ier)
{
    int *label = ier->label;
    int *newlabel = ier->newlabel;
    int width = ier->width;
    int height = ier->height;
    int wl = width+2;
    int hl = height+2;

    int iernum = ier->num;
    int channel = ier->channel;
    PIXEL *imagecolor = ier->imagecolori;
    int *avgc = ier->avgci;
    int *sumc = ier->sumci;
    int *sumn = ier->sumn;
    int *sumx = ier->sumx;
    int *sumy = ier->sumy;
    int *avgx = ier->avgxi;
    int *avgy = ier->avgyi;
    int wxy = ier->wxyi;

    IerEdgeQueue *edgeQueue = &ier->edgeQueue;
    POINT3D_S *Queue = edgeQueue->curQueue;
    int queuenum = edgeQueue->curQueueNum;
    int *labelflag = edgeQueue->labelflag + 2;
    int *changeflag = edgeQueue->changeflag;
    int *changeIndex = edgeQueue->changeIndex;
    int changeNum = 0;

    int i, c, n, nb;
    int x, y, pos, posl;
    int cur, next, adj;
    POINT3D_S *Q;
    int *pl;
    int nbindex[5];

    PIXEL *pc;
    int *psumc, *pavgc;
    int *psumc1, *psumc2;
    int v, dc;
    int cx, cy;
	int dx, dy;
    int distcolor, distxy;
	int dist, mindist;
	float norm;

#define IerAddNeighbor()            \
{                                   \
    if (!labelflag[adj])            \
    {                               \
        nbindex[nb++] = adj;        \
        labelflag[adj] = 1;         \
    }                               \
}

    memcpy(newlabel, label, hl*wl*sizeof(int));
    memset(changeflag, 0, iernum*sizeof(int));
    for (i = 0, Q = Queue;i < queuenum;i++, Q++)
    {
        x = Q->x;
        y = Q->y;
        pos = y*width + x;
        posl = Q->s;

        // ��ǰ���
        pl = label + posl;
        cur = *pl;
        labelflag[cur] = 1;
        nbindex[0] = cur;
        nb = 1;

        // �����ڱ��
        adj = pl[-1];
        IerAddNeighbor();
        
        adj = pl[1];
        IerAddNeighbor();

        adj = pl[-wl];
        IerAddNeighbor();

        adj = pl[wl];
        IerAddNeighbor();

        // �ر��
        mindist = IERI_INF;
        pc = imagecolor + pos*channel;
        x <<= IER_Q_AVGXY;
        y <<= IER_Q_AVGXY;
        for(n = 0; n < nb; n++)
		{
		    adj = nbindex[n];
            labelflag[adj] = 0;

            // 1: ��ɫ����
            distcolor = 0;
            pavgc = avgc + adj*channel;
            for (c = 0;c < channel;c++)
            {
                v = pc[c];
                v = v << IER_Q_AVGC;
                dc = v - pavgc[c];
                distcolor += dc*dc;
            }

            // 2: ���ξ���
            cx = avgx[adj];
            cy = avgy[adj];
    		dx = x - cx;
    		dy = y - cy;
            distxy = dx*dx + dy*dy;

            dist = distcolor + distxy*wxy;

    		if( dist < mindist )
    		{
    			mindist = dist;
    			next = adj;
    		}
		}

        if (next == cur) continue;

        // ��¼�ر�ǩ
        newlabel[posl] = next;
        changeIndex[changeNum++] = i;

        // ����ͳ����Ϣ
	    sumn[cur]--;
	    sumx[cur] -= x;
	    sumy[cur] -= y;
        sumn[next]++;
	    sumx[next] += x;
	    sumy[next] += y;
	    psumc1 = sumc + cur*channel;
	    psumc2 = sumc + next*channel;
        for (c = 0;c < channel;c++)
        {
            v = pc[c];
            v = v << IER_Q_AVGC;
            psumc1[c] -= v;
            psumc2[c] += v;
        }
	    changeflag[next] = 1;
        changeflag[cur] = 1;
    }

    // ���¾�������
    psumc = sumc;
    pavgc = avgc;
    for(n = 0; n < iernum; n++, psumc += channel, pavgc += channel)
    {
        if (!changeflag[n])             continue;

        norm = 1.0f/(float)sumn[n];
        avgx[n] = (int)(sumx[n]*norm);
        avgy[n] = (int)(sumy[n]*norm);
        for (c = 0;c < channel;c++)
        {
            pavgc[c] = (int)(psumc[c]*norm);
        }
    }

    // ���±�ǩ
    edgeQueue->changeNum = changeNum;
    ier->label = newlabel;
    ier->newlabel = label;

#undef IerAddNeighbor

    return;
}

void IerEdgeRefineIntColor(IN IerInfo *ier)
{
    int *label = ier->label;
    int *newlabel = ier->newlabel;
    int width = ier->width;
    int height = ier->height;
    int wl = width+2;
    int hl = height+2;

    int iernum = ier->num;
    PIXEL *imagecolor = ier->imagecolori;
    int *avgc = ier->avgci;
    int *sumc = ier->sumci;
    int *sumn = ier->sumn;
    int *sumx = ier->sumx;
    int *sumy = ier->sumy;
    int *avgx = ier->avgxi;
    int *avgy = ier->avgyi;
    int wxy = ier->wxyi;

    IerEdgeQueue *edgeQueue = &ier->edgeQueue;
    POINT3D_S *Queue = edgeQueue->curQueue;
    int queuenum = edgeQueue->curQueueNum;
    int *labelflag = edgeQueue->labelflag + 2;
    int *changeflag = edgeQueue->changeflag;
    int *changeIndex = edgeQueue->changeIndex;
    int changeNum = 0;

    int i, n, nb;
    int x, y, pos, posl;
    int cur, next, adj;
    POINT3D_S *Q;
    int *pl;
    int nbindex[5];

    PIXEL *pc;
    int *psumc, *pavgc;
    int *psumc1, *psumc2;
	int r, g, b;
    int dr, dg, db;
    int cx, cy;
	int dx, dy;
    int distcolor, distxy;
	int dist, mindist;
	float norm;

#define IerAddNeighbor()            \
{                                   \
    if (!labelflag[adj])            \
    {                               \
        nbindex[nb++] = adj;        \
        labelflag[adj] = 1;         \
    }                               \
}

    memcpy(newlabel, label, hl*wl*sizeof(int));
    memset(changeflag, 0, iernum*sizeof(int));
    for (i = 0, Q = Queue;i < queuenum;i++, Q++)
    {
        x = Q->x;
        y = Q->y;
        pos = y*width + x;
        posl = Q->s;

        // ��ǰ���
        pl = label + posl;
        cur = *pl;
        labelflag[cur] = 1;
        nbindex[0] = cur;
        nb = 1;

        // �����ڱ��
        adj = pl[-1];
        IerAddNeighbor();
        
        adj = pl[1];
        IerAddNeighbor();

        adj = pl[-wl];
        IerAddNeighbor();

        adj = pl[wl];
        IerAddNeighbor();

        // �ر��
        mindist = IERI_INF;
        pc = imagecolor + pos*IMAGE_COLOR_MAX;
        r = pc[0]; r <<= IER_Q_AVGC;
        g = pc[1]; g <<= IER_Q_AVGC;
        b = pc[2]; b <<= IER_Q_AVGC;
        x <<= IER_Q_AVGXY;
        y <<= IER_Q_AVGXY;
        for(n = 0; n < nb; n++)
		{
		    adj = nbindex[n];
            labelflag[adj] = 0;

            // 1: ��ɫ����
            distcolor = 0;
            pavgc = avgc + adj*IMAGE_COLOR_MAX;
            dr = r - pavgc[0];
            dg = g - pavgc[1];
            db = b - pavgc[2];
            distcolor = dr*dr + dg*dg + db*db;

            // 2: ���ξ���
            cx = avgx[adj];
            cy = avgy[adj];
    		dx = x - cx;
    		dy = y - cy;
            distxy = dx*dx + dy*dy;

            dist = distcolor + distxy*wxy;

    		if( dist < mindist )
    		{
    			mindist = dist;
    			next = adj;
    		}
		}

        if (next == cur) continue;

        // ��¼�ر�ǩ
        newlabel[posl] = next;
        changeIndex[changeNum++] = i;

        // ����ͳ����Ϣ
	    sumn[cur]--;
	    sumx[cur] -= x;
	    sumy[cur] -= y;
        sumn[next]++;
	    sumx[next] += x;
	    sumy[next] += y;
	    psumc1 = sumc + cur*IMAGE_COLOR_MAX;
	    psumc2 = sumc + next*IMAGE_COLOR_MAX;
        psumc1[0] -= r;
        psumc1[1] -= g;
        psumc1[2] -= b;
        psumc2[0] += r;
        psumc2[1] += g;
        psumc2[2] += b;
	    changeflag[next] = 1;
        changeflag[cur] = 1;
    }

    // ���¾�������
    psumc = sumc;
    pavgc = avgc;
    for(n = 0; n < iernum; n++, psumc += IMAGE_COLOR_MAX, pavgc += IMAGE_COLOR_MAX)
    {
        if (!changeflag[n])             continue;

        norm = 1.0f/(float)sumn[n];
        avgx[n] = (int)(sumx[n]*norm);
        avgy[n] = (int)(sumy[n]*norm);
        pavgc[0] = (int)(psumc[0]*norm);
        pavgc[1] = (int)(psumc[1]*norm);
        pavgc[2] = (int)(psumc[2]*norm);
    }

    // ���±�ǩ
    edgeQueue->changeNum = changeNum;
    ier->label = newlabel;
    ier->newlabel = label;

#undef IerAddNeighbor

    return;
}

void IerEdgeRefineInt(IN IerInfo *ier)
{
#if 0
    if (ier->channel == 3)
    {
        // IerEdgeRefineIntColor(ier);
        IerEdgeRefineNeighbor1(ier);
        IerEdgeRefineIntColor1(ier);
    }
    else
    {
        IerEdgeRefineIntChannel(ier);
    }
#else
    IerEdgeRefineNeighbor1(ier);
    if (ier->channel == 3)
    {        
        IerEdgeRefineIntColor1(ier);
    }
    else
    {
        IerEdgeRefineIntChannel1(ier);
    }
#endif
}

void IerEnforceConnectivity(IN IerInfo *ier)
{
    int width = ier->width;
    int height = ier->height;
    int wl = width+2;
    int S = ier->S;
	int sizeT = S*S/4;

    int label = 0;
    int *prelabel = ier->label;
    int *newlabel = ier->newlabel;
    IerConnectInfo *connect = &ier->connect;
	int *conxy = connect->conxy;

    int i, j, c;
    int pos1, pos2, ind;
    int pre, adj;
    int count;
    int seed;
    int *newl, *prel;
    int adjlabel = 0;

#define IerAdjLabel(d)                          \
{                                               \
    adj = newl[d];                              \
    if(adj >= 0)    adjlabel = adj;             \
}

#define IerAddNeighbor(d)                       \
{                                               \
    if( 0 > newl[d] && pre == prel[d] )         \
    {                                           \
    	conxy[count] = d + seed;                \
    	newl[d] = label;                        \
    	count++;                                \
    }                                           \
}

    // �߽�Ϊ-2��δ���Ϊ-1
    memset(newlabel, -1, wl*(height+2)*sizeof(int));
	for(i = 1, pos1 = wl; i <= height; i++, pos1 += wl)
	{
	    pos2 = pos1 + 1;
		for(j = 1; j <= width; j++, pos2++)
		{
		    if (newlabel[pos2] >= 0)    continue;

			newlabel[pos2] = label;
			conxy[0] = pos2;

			// �����Ҹ�����!
            newl = newlabel + pos2;
            IerAdjLabel(-1);
            IerAdjLabel(-wl);
            IerAdjLabel(1);
            IerAdjLabel(wl);

            // ��������㷨
			count = 1;
			pre = prelabel[pos2];
			for(c = 0; c < count; c++)
			{
                seed = conxy[c];
                newl = newlabel + seed;
                prel = prelabel + seed;
                IerAddNeighbor(-1);
                IerAddNeighbor(-wl);
                IerAddNeighbor(1);
                IerAddNeighbor(wl);
			}

			// �������̫С: �ϲ�������
			if(count <= sizeT)
			{
				for(c = 0; c < count; c++ )
				{
					ind = conxy[c];
					newlabel[ind] = adjlabel;
				}
				label--;
			}
			label++;
		}
	}

    ier->num = label;
    ier->label = newlabel;
    ier->newlabel = prelabel;

#undef IerAdjLabel
#undef IerAddNeighbor

	return;
}

/*
 �������ܣ�
	���ݻ������ĵĲ���������ɴ�����ͼ���size��Ҫ���ֵ�����ĸ���K��ȷ����
	��ȷ�����������ϵ����񻮷ֲ���
 param_in:
	S0:		�������ļ��
	width:	ͼ��Ŀ�Ҳ�����Ǹߣ�
 param_out
	sp:		���񻮷ֲ���
*/
void MeshSplitSize(INOUT MeshSplitInfo *sp, IN int S0, IN int width)
{
    int n, n1, n2;
    int S;
    
    n = width/S0;		
    S = width/n;		
    n2 = width - n*S;	
    n1 = n - n2;
    sp->S = S;
    sp->n1 = n1;
    sp->n2 = n2;
    sp->n = n;
    return;
}

/*
 ��������:
	�����������ļ��ξغ���ɫ���ԣ�����ʼ�������label
 param_in:
	Sx,Sy:		����ı߳�
	nx,ny:		����ĸ���
	nx1,ny1:	
*/
void IerMeshStat(INOUT IerInfo *ier, 
                 IN int Sx,
                 IN int Sy,
                 IN int nx,
                 IN int ny,
                 IN int nx1,
                 IN int ny1)
{
    int width = ier->width;
    int height = ier->height;
    int channel = ier->channel;
    int w = width + 2;
    int widthc = width*channel;

    int *label = ier->label;
    floatier *imagecolor = ier->imagecolor;

    int *sumn = ier->sumn;
    int *sumx = ier->sumx;
    int *sumy = ier->sumy;
    floatier *avgx = ier->avgx;
    floatier *avgy = ier->avgy;
    floatier *sumc = ier->sumc;
    floatier *avgc = ier->avgc;

    int i, j, k, nw;
    int x, y;
    int sx, sy;
    int x0, y0;		
    int x1, y1;		
    int xx, yy;
    double xc, yc;
    int m00, m01, m10;
    int *exlabel, *image;

    floatier *srccolor;
    floatier *psumc, *pavgc;
    floatier *pc, *qc;
    double norm;

    y0 = 0;
    exlabel = label + w + 1;
    srccolor = imagecolor;
    memset(sumc, 0, ier->num*channel*sizeof(floatier));
    for (i = 0, k = 0, sy = Sy;i < ny;i++)
    {
        if (i == ny1)   sy++;
        y1 = y0 + sy - 1;
        yy = y0 + y1;
        yc = yy*0.5;

        x0 = 0;
        m00 = sy*Sx;
        norm = 1.0/m00;
        for (j = 0, sx = Sx;j < nx;j++, k++)
        {
            if (j == nx1)
            {
                sx++;
                m00 += sy;
                norm = 1.0/m00;
            }
            x1 = x0 + sx - 1;
            xx = x0 + x1;
            xc = xx*0.5;

            // ֱ�Ӽ��㼸�ξ�
            m01 = m00*xx/2;
            m10 = m00*yy/2;
            sumn[k] = m00;	// ���������صĸ���
            sumx[k] = m01;	// �������������ص�x����ĺ�
            sumy[k] = m10;	// �������������ص�y����ĺ�
            avgx[k] = xc;	// ...x�����ƽ��ֵ
            avgy[k] = yc;	// ...y�����ƽ��ֵ

            // ͳ��color sum
            pc = srccolor + x0*channel;
            psumc = sumc + k*channel;
            for (y = 0;y < sy;y++)
            {
                for(x = 0, qc = pc;x < sx;x++, qc += channel)
                {
                    VectorPlusIer(psumc, qc, psumc, channel);
                }
                pc += widthc;
            }
            pavgc = avgc + k*channel;
            VectorMulConstIer(psumc, pavgc, norm, channel);

            // ���õ�һ��label
            image = exlabel + x0;
            for(x = 0;x < sx;x++)
            {
                image[x] = k;
            }

            // ������ʼ����
            x0 += sx;
        }

        // ���ú����е�label
        image = exlabel + w;
        for (y = 1;y < sy;y++)
        {
            memcpy(image, exlabel, width*sizeof(int));
            image += w;
        }

        nw = sy*width;
        srccolor += nw*channel;
        exlabel += sy*w;
        y0 += sy;
    }

    // ��չslic�е�label
    IerLabelInit(label, width, height, -2);

    return;
}

/*
 ���������ö����ʾ��������IerMeshStat��ͬ��Ϊ����߼����ٶ�
*/
void IerMeshStatInt(INOUT IerInfo *ier, 
                    IN int Sx,
                    IN int Sy,
                    IN int nx,
                    IN int ny,
                    IN int nx1,
                    IN int ny1)
{
    int width = ier->width;
    int height = ier->height;
    int channel = ier->channel;
    int w = width + 2;
    int widthc = width*channel;

    int *label = ier->label;
    PIXEL *imagecolor = ier->imagecolori;

    int *sumn = ier->sumn;
    int *sumx = ier->sumx;
    int *sumy = ier->sumy;
    int *avgx = ier->avgxi;
    int *avgy = ier->avgyi;
    int *sumc = ier->sumci;
    int *avgc = ier->avgci;

    int i, j, k, c;
    int x, y;
    int sx, sy;
    int x0, y0;
    int x1, y1;
    int xx, yy;
    int xc, yc;
    int size;
    int *exlabel, *image;

    PIXEL *srccolor;
    PIXEL *pc, *qc;
    int *psumc, *pavgc;
    float norm;

    y0 = 0;
    exlabel = label + w + 1;
    srccolor = imagecolor;
    memset(sumc, 0, ier->num*channel*sizeof(int));
    for (i = 0, k = 0, sy = Sy;i < ny;i++)
    {
        if (i == ny1)   sy++;  // ny1�����鲿�ֵĳ��ȣ�һ��ʼsyһֱ��С������ı߳�, ����ԭ���У���ĳЩ��ʼ������߶ȿ��ܱ�Ϊh+1
        y1 = y0 + sy - 1;     // ��һ�ε��� sy - 1���Ժ�ÿ�� +sy  �����sy++��ִ��Ҳͬ���ģ� �Ʋ�y1��Ϊ��С���ĩβֵ������[y0, y1]
        yy = y0 + y1;     // yy
        yc = (yy << IER_Q_AVGXY) >> 1;    // yc = yy / 4;  

        x0 = 0;
        size = sy*Sx;          // ��С������
        norm = 1.0f/(float)size;    // �����֮һ
        for (j = 0, sx = Sx;j < nx;j++, k++)  // ��ʼ������
        {
            if (j == nx1)
            {				// ������˺������һС��
                sx++;
                size += sy;   //���������������һ�У������
                norm = 1.0f/(float)size;
            }
            x1 = x0 + sx - 1;
            xx = x0 + x1;
            xc = (xx << IER_Q_AVGXY) >> 1;

            // ֱ�Ӽ��㼸�ξ�
			// ier�ṹ���еľ������Ĳ���
			// �������������̵�hu�����
            sumn[k] = size;             // ��k��С������
            sumx[k] = size*xc;			// ��k��С���
            sumy[k] = size*yc;
            avgx[k] = xc;               // 
            avgy[k] = yc;

            // ͳ��color sum
            pc = srccolor + x0*channel;       // �ȼӺ���ƫ��
            psumc = sumc + k*channel;
            for (y = 0;y < sy;y++)          // ����߳���pcֵ����ÿ��ɨ������������ԣ�qc����ľ���ͼ������ص�ָ��
            {
                for(x = 0, qc = pc;x < sx;x++, qc += channel)        //����߳��� ÿɨ��һ�����أ�ָ��qcƫ��һ������
                {
                    for (c = 0;c < channel;c++)
                    {
                        psumc[c] += qc[c];            //sumc ��Ϊk��ͼ��С�飬ÿ��ͼ��С����channel���洢λ�ã�����ÿ���洢λ�þ���һ��С��һ��ͨ���ϵĺ�
                    }
                }
                pc += widthc;
            }
            pavgc = avgc + k*channel;
            for (c = 0;c < channel;c++)              // �鿴�ú������˵��
            {
                int v = psumc[c] << IER_Q_AVGC;
                pavgc[c] = (int)(v*norm);
                psumc[c] = v;
            }

            // ���õ�һ��label
            image = exlabel + x0;
            for(x = 0;x < sx;x++)
            {
                image[x] = k;
            }

            // ������ʼ����
            x0 += sx;
        }

        // ���ú����е�label
        image = exlabel + w;
        for (y = 1;y < sy;y++)
        {
            memcpy(image, exlabel, width*sizeof(int));
            image += w;
        }

        srccolor += sy*width*channel;
        exlabel += sy*w;
        y0 += sy;
    }

    // ��չslic�е�label
    IerLabelInit(label, width, height, -2);

    return;
}

void IerFlagInit(PIXEL *label, int width, int height, PIXEL neg)
{
    int i;
    PIXEL *p;
    int w = width + 2;
    int h = height + 2;

    p = label;
    memset(p, neg, w*sizeof(PIXEL));
    p = label + (h - 1)*w;
    memset(p, neg, w*sizeof(PIXEL));

    p = label + w;
    for (i = 1;i < h;i++, p += w)
    {
        *p = neg;
    }
    p = label + 2*w-1;
    for (i = 1;i < h;i++, p += w)
    {
        *p = neg;
    }

    return;
}

void IerLabelInit(int *label, int width, int height, int neg)
{
    int i;
    int *p;
    int w = width + 2;
    int h = height + 2;

    p = label;
    for (i = 0;i < w;i++) // ��һ��
    {
        p[i] = neg;
    }
    p = label + (h - 1)*w;
    for (i = 0;i < w;i++) // ���һ��
    {
        p[i] = neg;
    }
    p = label + w;
    for (i = 1;i < h;i++, p += w)  // ��һ��
    {
        *p = neg;
    }
    p = label + 2*w-1;
    for (i = 1;i < h;i++, p += w)  // ���һ��
    {
        *p = neg;
    }

    return;
}

void IerLabelExtend(int *inl, int *outl, int width, int height)
{
    int i;
    int *p, *q;
    int w = width + 2;
    int h = height + 2;

    p = inl + (height-1)*width;
    q = outl + (h - 1) * w - w + 1;
    for (i = height-1;i >= 0;i--)
    {
        memcpy(q, p, width*sizeof(int));
        p -= width;
        q -= w;
    }

    return;
}

void IerLabelDeextend(int *inl, int *outl, int width, int height)
{
    int i;
    int *p, *q;
    int w = width + 2;

    p = outl;
    q = inl + w + 1;
    for (i = 0;i < height;i++)
    {
        memcpy(p, q, width*sizeof(int));
        p += width;
        q += w;
    }

    return;
}

void ImageSegShow(IN IMAGE_S *srcImage,
                  IN int *flag,
                  IN PIXEL *contour,
                  OUT IMAGE_S *showImage)
{
    PIXEL *src = srcImage->data;
    int height = srcImage->height;
    int width = srcImage->width;
    int channel = srcImage->channel;
    PIXEL *dst = showImage->data;
    int endh = height-1;
    int endw = width-1;
    
    PIXEL defaultc[3] = {255, 255, 255};

    int x,y;
	int xstart;
	int pos;
    int left,up,right,down;
    BOOL isContour;

    if (contour == NULL)    contour = defaultc;

    if (srcImage != showImage)
    {
        memcpy(dst, src, height*width*channel*sizeof(PIXEL));
        showImage->width = width;
        showImage->height = height;
        showImage->channel = channel;
    }

	for (y = 1;y < endh;y++)
	{
		xstart = y*width;
		for (x = 1;x < endw;x++)
		{
			pos = xstart + x;
            left = pos - 1;
            right = pos + 1;
            up = pos - width;
		    down = pos + width;

		    isContour = (flag[left] != flag[pos]) ||
	                    (flag[right] != flag[pos]) ||
	                    (flag[up] != flag[pos]) ||
	                    (flag[down] != flag[pos]);
            if (isContour == FALSE) continue;
            memcpy(dst+pos*channel, contour, channel*sizeof(PIXEL));
		}
	}

	return;
}

// ȥ����label��memcpy
void IerEdgeRefineNeighbor1(IN IerInfo *ier)
{
    int *label = ier->label;
    int width = ier->width;
    int wl = width+2;

    IerEdgeQueue *edgeQueue = &ier->edgeQueue;
    POINT3D_S *Queue = edgeQueue->curQueue;
    int queuenum = edgeQueue->curQueueNum;
    int *labelflag = edgeQueue->labelflag + 2;
    int *nbIndex = edgeQueue->nbIndex;
    int *nbNumber = edgeQueue->nbNumber;

    int i, n, nb;
    int posl;
    int cur, adj;
    POINT3D_S *Q;
    int *pl;

#define IerAddNeighbor()            \
{                                   \
    if (!labelflag[adj])            \
    {                               \
        nbIndex[nb++] = adj;        \
        labelflag[adj] = 1;         \
    }                               \
}

    for (i = 0, Q = Queue;i < queuenum;i++, Q++, nbIndex += IER_MAX_NBNUM)
    {
        // ��ǰ���
        posl = Q->s;
        pl = label + posl;
        cur = *pl;
        labelflag[cur] = 1;
        nbIndex[0] = cur;
        nb = 1;

        // �����ڱ��
        adj = pl[-1];
        IerAddNeighbor();

        adj = pl[1];
        IerAddNeighbor();

        adj = pl[-wl];
        IerAddNeighbor();

        adj = pl[wl];
        IerAddNeighbor();

        nbNumber[i] = nb;
        for(n = 0; n < nb; n++)
		{
		    adj = nbIndex[n];
            labelflag[adj] = 0;
        }
    }

#undef IerAddNeighbor

    return;
}

void IerEdgeRefineIntChannel1(IN IerInfo *ier)
{
    int *label = ier->label;
    int width = ier->width;

    int iernum = ier->num;
    int channel = ier->channel;
    PIXEL *imagecolor = ier->imagecolori;
    int *avgc = ier->avgci;
    int *sumc = ier->sumci;
    int *sumn = ier->sumn;
    int *sumx = ier->sumx;
    int *sumy = ier->sumy;
    int *avgx = ier->avgxi;
    int *avgy = ier->avgyi;
    int wxy = ier->wxyi;

    IerEdgeQueue *edgeQueue = &ier->edgeQueue;
    POINT3D_S *Queue = edgeQueue->curQueue;
    int queuenum = edgeQueue->curQueueNum;
    int *changeflag = edgeQueue->changeflag;
    int *changeIndex = edgeQueue->changeIndex;
    int *nbIndex = edgeQueue->nbIndex;
    int *nbNumber = edgeQueue->nbNumber;
    int changeNum = 0;

    int i, c, n, nb;
    int x, y, pos, posl;
    int cur, next, adj;
    POINT3D_S *Q;

    PIXEL *pc;
    int *psumc, *pavgc;
    int *psumc1, *psumc2;
    int v, dc;
    int cx, cy;
	int dx, dy;
    int distcolor, distxy;
	int dist, mindist;
	float norm;

    memset(changeflag, 0, iernum*sizeof(int));
    for (i = 0, Q = Queue;i < queuenum;i++, Q++, nbIndex += IER_MAX_NBNUM)
    {
        x = Q->x;
        y = Q->y;
        pos = y*width + x;

        // �ر��
        mindist = IERI_INF;
        pc = imagecolor + pos*channel;
        x <<= IER_Q_AVGXY;
        y <<= IER_Q_AVGXY;
        nb = nbNumber[i];
        for(n = 0; n < nb; n++)
		{
		    adj = nbIndex[n];

            // 1: ��ɫ����
            distcolor = 0;
            pavgc = avgc + adj*channel;
            for (c = 0;c < channel;c++)
            {
                v = pc[c];
                v = v << IER_Q_AVGC;
                dc = v - pavgc[c];
                distcolor += dc*dc;
            }

            // 2: ���ξ���
            cx = avgx[adj];
            cy = avgy[adj];
    		dx = x - cx;
    		dy = y - cy;
            distxy = dx*dx + dy*dy;

            dist = distcolor + distxy*wxy;

    		if( dist < mindist )
    		{
    			mindist = dist;
    			next = adj;
    		}
		}

        cur = nbIndex[0];
        if (next == cur) continue;

        // ��¼�ر�ǩ
        posl = Q->s;
        label[posl] = next;
        changeIndex[changeNum++] = i;

        // ����ͳ����Ϣ
	    sumn[cur]--;
	    sumx[cur] -= x;
	    sumy[cur] -= y;
        sumn[next]++;
	    sumx[next] += x;
	    sumy[next] += y;
	    psumc1 = sumc + cur*channel;
	    psumc2 = sumc + next*channel;
        for (c = 0;c < channel;c++)
        {
            v = pc[c];
            v = v << IER_Q_AVGC;
            psumc1[c] -= v;
            psumc2[c] += v;
        }
	    changeflag[next] = 1;
        changeflag[cur] = 1;
    }

    // ���¾�������
    psumc = sumc;
    pavgc = avgc;
    for(n = 0; n < iernum; n++, psumc += channel, pavgc += channel)
    {
        if (!changeflag[n])             continue;

        norm = 1.0f/(float)sumn[n];
        avgx[n] = (int)(sumx[n]*norm);
        avgy[n] = (int)(sumy[n]*norm);
        for (c = 0;c < channel;c++)
        {
            pavgc[c] = (int)(psumc[c]*norm);
        }
    }

    // ���±�ǩ
    edgeQueue->changeNum = changeNum;

    return;
}

void IerEdgeRefineIntColor1(IN IerInfo *ier)
{
    int *label = ier->label;
    int width = ier->width;

    int iernum = ier->num;
    PIXEL *imagecolor = ier->imagecolori;
    int *avgc = ier->avgci;
    int *sumc = ier->sumci;
    int *sumn = ier->sumn;
    int *sumx = ier->sumx;
    int *sumy = ier->sumy;
    int *avgx = ier->avgxi;
    int *avgy = ier->avgyi;
    int wxy = ier->wxyi;

    IerEdgeQueue *edgeQueue = &ier->edgeQueue;
    POINT3D_S *Queue = edgeQueue->curQueue;
    int queuenum = edgeQueue->curQueueNum;
    int *changeflag = edgeQueue->changeflag;
    int *changeIndex = edgeQueue->changeIndex;
    int *nbIndex = edgeQueue->nbIndex;
    int *nbNumber = edgeQueue->nbNumber;
    int changeNum = 0;

    int i, n, nb;
    int x, y, pos, posl;
    int cur, next, adj;
    POINT3D_S *Q;

    PIXEL *pc;
    int *psumc, *pavgc;
    int *psumc1, *psumc2;
	int r, g, b;
    int dr, dg, db;
    int cx, cy;
	int dx, dy;
    int distcolor, distxy;
	int dist, mindist;
#if 0
	int norm;
#else
    float norm;
#endif

    memset(changeflag, 0, iernum*sizeof(int));
    for (i = 0, Q = Queue;i < queuenum;i++, Q++, nbIndex += IER_MAX_NBNUM)
    {
        x = Q->x;
        y = Q->y;
        pos = y*width + x;

        // �ر��
        mindist = IERI_INF;
        pc = imagecolor + pos*IMAGE_COLOR_MAX;
        r = pc[0]; r <<= IER_Q_AVGC;
        g = pc[1]; g <<= IER_Q_AVGC;
        b = pc[2]; b <<= IER_Q_AVGC;
        x <<= IER_Q_AVGXY;
        y <<= IER_Q_AVGXY;
        nb = nbNumber[i];
        for(n = 0; n < nb; n++)
		{
		    adj = nbIndex[n];

            // 1: ��ɫ����
            distcolor = 0;
            pavgc = avgc + adj*IMAGE_COLOR_MAX;
            dr = r - pavgc[0];
            dg = g - pavgc[1];
            db = b - pavgc[2];
            distcolor = dr*dr + dg*dg + db*db;

            // 2: ���ξ���
            cx = avgx[adj];
            cy = avgy[adj];
    		dx = x - cx;
    		dy = y - cy;
            distxy = dx*dx + dy*dy;

            dist = distcolor + distxy*wxy;

    		if( dist < mindist )
    		{
    			mindist = dist;
    			next = adj;
    		}
		}

        cur = nbIndex[0];
        if (next == cur) continue;

        // ��¼�ر�ǩ
        posl = Q->s;
        label[posl] = next;
        changeIndex[changeNum++] = i;

        // ����ͳ����Ϣ
	    sumn[cur]--;
	    sumx[cur] -= x;
	    sumy[cur] -= y;
        sumn[next]++;
	    sumx[next] += x;
	    sumy[next] += y;
	    psumc1 = sumc + cur*IMAGE_COLOR_MAX;
	    psumc2 = sumc + next*IMAGE_COLOR_MAX;
        psumc1[0] -= r;
        psumc1[1] -= g;
        psumc1[2] -= b;
        psumc2[0] += r;
        psumc2[1] += g;
        psumc2[2] += b;
	    changeflag[next] = 1;
        changeflag[cur] = 1;
    }

    // ���¾�������
    psumc = sumc;
    pavgc = avgc;
    for(n = 0; n < iernum; n++, psumc += IMAGE_COLOR_MAX, pavgc += IMAGE_COLOR_MAX)
    {
        if (!changeflag[n])             continue;

// #ifdef PLATFORM_CCS
#if 0
        norm = 0x10000000/sumn[n];
        avgx[n] = _IQNmpyIQx(norm, 28, sumx[n], IER_Q_AVGXY, IER_Q_AVGXY);
        avgy[n] = _IQNmpyIQx(norm, 28, sumy[n], IER_Q_AVGXY, IER_Q_AVGXY);
        pavgc[0] = _IQNmpyIQx(norm, 28, psumc[0], IER_Q_AVGC, IER_Q_AVGC);
        pavgc[1] = _IQNmpyIQx(norm, 28, psumc[1], IER_Q_AVGC, IER_Q_AVGC);
        pavgc[2] = _IQNmpyIQx(norm, 28, psumc[2], IER_Q_AVGC, IER_Q_AVGC);
#else
        norm = 1.0f/(float)sumn[n];
        avgx[n] = (int)(sumx[n]*norm);
        avgy[n] = (int)(sumy[n]*norm);
        pavgc[0] = (int)(psumc[0]*norm);
        pavgc[1] = (int)(psumc[1]*norm);
        pavgc[2] = (int)(psumc[2]*norm);
#endif
    }

    // ���±�ǩ
    edgeQueue->changeNum = changeNum;

    return;
}

#if 0

// ������Ϊ��Ԫ�����(ϣ��������ѭ���е���ˮ����ʵ����������鷳�����Ӷ���Ŀ���)

void IerEdgeRefineNeighbor2(IN IerInfo *ier)
{
    int *label = ier->label;
    int width = ier->width;
    int wl = width+2;

    IerEdgeQueue *edgeQueue = &ier->edgeQueue;
    POINT3D_S *Queue = edgeQueue->curQueue;
    int queuenum = edgeQueue->curQueueNum;
    int *labelflag = edgeQueue->labelflag + 2;
    int *nbIndex = edgeQueue->nbIndex;
    int *nbNumber = edgeQueue->nbNumber;

    int i, n, nb;
    int posl;
    int cur, adj;
    POINT3D_S *Q;
    int *pl;

#define IerAddNeighbor()            \
{                                   \
    if (!labelflag[adj])            \
    {                               \
        nbIndex[nb++] = adj;        \
        labelflag[adj] = 1;         \
    }                               \
}

    for (i = 0, Q = Queue;i < queuenum;i++, Q++, nbIndex += IER_MAX_NBNUM)
    {
        // ��ǰ���
        posl = Q->s;
        pl = label + posl;
        cur = *pl;
        labelflag[cur] = 1;
        nbIndex[0] = cur;
        nb = 1;

        // �����ڱ��
        adj = pl[-1];
        IerAddNeighbor();

        adj = pl[1];
        IerAddNeighbor();

        adj = pl[-wl];
        IerAddNeighbor();

        adj = pl[wl];
        IerAddNeighbor();

        nbNumber[i] = nb;
        for(n = 0; n < nb; n++)
		{
		    adj = nbIndex[n];
            labelflag[adj] = 0;
        }
    }

#undef IerAddNeighbor

    return;
}

void IerEdgeRefineIntColor2(IN IerInfo *ier)
{
    int *label = ier->label;
    int width = ier->width;

    int iernum = ier->num;
    PIXEL *imagecolor = ier->imagecolori;
    int *avgc = ier->avgci;
    int *sumc = ier->sumci;
    int *sumn = ier->sumn;
    int *sumx = ier->sumx;
    int *sumy = ier->sumy;
    int *avgx = ier->avgxi;
    int *avgy = ier->avgyi;
    int wxy = ier->wxyi;

    IerEdgeQueue *edgeQueue = &ier->edgeQueue;
    POINT3D_S *Queue = edgeQueue->curQueue;
    int queuenum = edgeQueue->curQueueNum;
    int *changeflag = edgeQueue->changeflag;
    int *changeIndex = edgeQueue->changeIndex;
    int *nbIndex = edgeQueue->nbIndex;
    int *nbNumber = edgeQueue->nbNumber;
    int changeNum = 0;

    int i, n, nb;
    int x, y, pos, posl;
    int cur, next, adj;
    POINT3D_S *Q;

    PIXEL *pc;
    int *psumc, *pavgc;
    int *psumc1, *psumc2;
	int r, g, b;
    int dr, dg, db;
    int cx, cy;
	int dx, dy;
    int distcolor, distxy;
	int dist, mindist;
	float norm;




    psumc = sumc;
    pavgc = avgc;
    for(n = 0; n < iernum; n++, psumc += IMAGE_COLOR_MAX, pavgc += IMAGE_COLOR_MAX)
    {
        edge = edgelist[n];
        num = edge->num;
        seq = edge->seq;

        // �������
        pavgc = avgc + adj*IMAGE_COLOR_MAX;
        cr = pavgc[0];
        cg = pavgc[0];
        cb = pavgc[0];
        cx = avgx[adj];
        cy = avgy[adj];
        for(i = 0; i < num; i++)
		{
            Q = Queue + seq[i];
            x = Q->x;
            y = Q->y;
            pos = y*width + x;

            pc = imagecolor + pos*IMAGE_COLOR_MAX;
            r = pc[0]; r <<= IER_Q_AVGC;
            g = pc[1]; g <<= IER_Q_AVGC;
            b = pc[2]; b <<= IER_Q_AVGC;
            x <<= IER_Q_AVGXY;
            y <<= IER_Q_AVGXY;

            // 1: ��ɫ����
            dr = r - cr;
            dg = g - cg;
            db = b - cb;
            distcolor = dr*dr + dg*dg + db*db;

            // 2: ���ξ���
    		dx = x - cx;
    		dy = y - cy;
            distxy = dx*dx + dy*dy;

            dist = distcolor + distxy*wxy;

            cind = cseq[i];
            distarr[pos*5+cind] = dist;
		}
    }



    memset(changeflag, 0, iernum*sizeof(int));
    for (i = 0, Q = Queue;i < queuenum;i++, Q++, nbIndex += IER_MAX_NBNUM)
    {
        x = Q->x;
        y = Q->y;
        pos = y*width + x;

        // �ر��
        mindist = IERI_INF;
        pc = imagecolor + pos*IMAGE_COLOR_MAX;
        r = pc[0]; r <<= IER_Q_AVGC;
        g = pc[1]; g <<= IER_Q_AVGC;
        b = pc[2]; b <<= IER_Q_AVGC;
        x <<= IER_Q_AVGXY;
        y <<= IER_Q_AVGXY;
        nb = nbNumber[i];
        for(n = 0; n < nb; n++)
		{
		    adj = nbIndex[n];

            // 1: ��ɫ����
            distcolor = 0;
            pavgc = avgc + adj*IMAGE_COLOR_MAX;
            dr = r - pavgc[0];
            dg = g - pavgc[1];
            db = b - pavgc[2];
            distcolor = dr*dr + dg*dg + db*db;

            // 2: ���ξ���
            cx = avgx[adj];
            cy = avgy[adj];
    		dx = x - cx;
    		dy = y - cy;
            distxy = dx*dx + dy*dy;

            dist = distcolor + distxy*wxy;

    		if( dist < mindist )
    		{
    			mindist = dist;
    			next = adj;
    		}
		}

        cur = nbIndex[0];
        if (next == cur) continue;

        // ��¼�ر�ǩ
        posl = Q->s;
        label[posl] = next;
        changeIndex[changeNum++] = i;

        // ����ͳ����Ϣ
	    sumn[cur]--;
	    sumx[cur] -= x;
	    sumy[cur] -= y;
        sumn[next]++;
	    sumx[next] += x;
	    sumy[next] += y;
	    psumc1 = sumc + cur*IMAGE_COLOR_MAX;
	    psumc2 = sumc + next*IMAGE_COLOR_MAX;
        psumc1[0] -= r;
        psumc1[1] -= g;
        psumc1[2] -= b;
        psumc2[0] += r;
        psumc2[1] += g;
        psumc2[2] += b;
	    changeflag[next] = 1;
        changeflag[cur] = 1;
    }

    // ���¾�������
    psumc = sumc;
    pavgc = avgc;
    for(n = 0; n < iernum; n++, psumc += IMAGE_COLOR_MAX, pavgc += IMAGE_COLOR_MAX)
    {
        if (!changeflag[n])             continue;

        norm = 1.0f/(float)sumn[n];
        avgx[n] = (int)(sumx[n]*norm);
        avgy[n] = (int)(sumy[n]*norm);
        pavgc[0] = (int)(psumc[0]*norm);
        pavgc[1] = (int)(psumc[1]*norm);
        pavgc[2] = (int)(psumc[2]*norm);
    }

    // ���±�ǩ
    edgeQueue->changeNum = changeNum;

    return;
}

#endif



#ifdef __cplusplus
    }
#endif /* end of __cplusplus */

