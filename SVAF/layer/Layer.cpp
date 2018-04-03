/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
Layer����
*/

#include "Layer.h"

namespace svaf{

// ��Щ��̬��Ա����������Layer�����
size_t *Layer::id = NULL;					// ֡id����0��ʼ
Figures<> *Layer::figures = NULL;			// ��¼����Ķ�ά���ݱ�
SvafApp Layer::task_type = SvafApp::NONE;	// ���ڽ����ı����������жϽ�������
bool Layer::gui_mode = false;				// ���Ƿ��ȥGUIģʽ
Circuit *Layer::pCir = NULL;				// ��ָ�����ڵ��÷��������߳�

Layer::Layer()
{
}

Layer::Layer(LayerParameter& layer){
	__bout = false;
	__name = layer.name();
	__show = Layer::gui_mode ? false : layer.show();
	__save = layer.save();
	__logi = layer.logi();
	__logt = layer.logt();
}

Layer::~Layer()
{
}

// ���̼�ͨ�ŷ�����Ϣ
void Layer::RLOG(std::string i){	
	pCir->RLOG(i);
}

}
