/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
������
*/

#include <hash_map>
#include <opencv2\opencv.hpp>
#include <gflags\gflags.h>
#include <glog\logging.h>

#include "..\svaf\svaf.pb.h"
#include "..\svaf\io.hpp"

#include "Circuit.h"

using namespace std;
using namespace cv;

// ��gflags����ó������������: config_file  use_gui
// �������������иó����ʱ�����ָ���������������磺"executable file dir"> SVAF --use_gui=false --config_file=./svaf/svaf.pbf 
// ��������--help��������ȡ�����Ľ���
DEFINE_bool(use_gui, false, "wheather to use process communication module");
DEFINE_string(config_file, "./svaf/svaf.pbf", "config file of the framework");

int main(int argc, char *argv[]){
	google::InitGoogleLogging((const char *)argv[0]);				// ��ʼ��GLOG��־��¹
	google::SetLogDestination(google::GLOG_INFO, "./log/LOG");		// ������־��¼�ļ�
	google::SetStderrLogging(google::GLOG_INFO);					// ������־��¼����
	LOG(INFO) << "Svaf Copyright(c) 2016-2018, Peng Chao";			// ��Ȩ��Ϣ
	gflags::ParseCommandLineFlags(&argc, &argv, true);				// GFLAGS����������
	LOG(INFO) << FLAGS_config_file;									// ����ű��ļ�����
	svaf::SvafTask svafTask;										// ʵ��������
	svaf::ReadProtoFromTextFileOrDie(FLAGS_config_file, &svafTask);	// Protobuf��ȡ�ű�,��svaf.pbf�ű��е����ݽ�����svafTask����
	LOG(INFO) << svafTask.name();									// ��ʾ�ű��е���������
	svaf::Circuit circuit(svafTask, FLAGS_use_gui);					// ִ���㷨·�ߣ�����ʹ��svafTask�е����ݹ����㷨·�ߵȣ�
	LOG(INFO) << "Done.";											// ��������
	google::ShutdownGoogleLogging();								// �ر���־
	return 0;
}
