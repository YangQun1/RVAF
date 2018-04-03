/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
��¼��������ݱ��
*/


#include <glog\logging.h>
#include <iostream>
#include <vector>
#include <map>

using namespace std;

namespace svaf{

// ��������ģ��Ԫ
// ͼ���ÿ��Ԫ��
template<typename T>
struct Cell{
	bool	mask = false;
	T		val;
	void operator=(T &value){val = value; mask = true;}
	Cell(T value, bool valid) : val(value), mask(valid){}
	Cell(T value) : val(value), mask(true){}
	Cell() : mask(false){};
};

// ���ݱ�ģ��
// ÿһ����һ��vector����ʾ�����Լ���field����vector�ĳ��Ⱦ���ͼ���������
// �����У�������map�����һ��vector�������������ǵ���vector�ĳ���
template<typename T = float>	// ���ʣ������=float��ʲô���ã���Ĭ����float����Figure<>���ȼ���Figure<float>
class Figures{
public:
	Figures();
	~Figures();
	size_t rows();
	size_t cols();
	void clear();
	void clearAll();
	void addRow();
	void setRow(size_t size);
	void addCol(string filed);
	void push_back(string filed, size_t id, T value);
	void print2txt(string filename);
	void print2txt_im(string filename, vector<string>& images);
	void print2txt_impair(string filename, vector<pair<string, string>>&);
	void print2scr();

	vector<Cell<T>>& operator[](string& filed);
	vector<Cell<T>>& operator[](const char * filed);

protected:
	void resize(size_t size);

private:
	size_t size_;
	map<string, vector<Cell<T>>> figure_;
	
};

}

namespace svaf{

	// ����
	template<typename T>
	Figures<T>::Figures(){
	}

	// ����
	template<typename T>
	Figures<T>::~Figures(){
	}

	// ��������
	template<typename T>
	size_t Figures<T>::rows(){
		return size_;
	}

	// ��������
	template<typename T>
	size_t Figures<T>::cols(){
		return figure_.size();
	}

	// ���һ��
	template<typename T>
	void Figures<T>::addRow(){
		size_++;
		resize(size_);
	}

	// ��������
	template<typename T>
	void Figures<T>::setRow(size_t size){
		if (size < size_){
			LOG(ERROR) << "Figures Data May Loss.";
		}
		resize(size);
	}

	// ���һ��
	template<typename T>
	void Figures<T>::addCol(string filed){
		vector<Cell<T>> tempvec				// ���ʣ�ʲô�÷���
			figure_[filed] = tempvec;
	}

	// ���ñ���С
	template<typename T>
	void Figures<T>::resize(size_t size){
		size_ = size;
		for (auto &itm : figure_){
			itm.second.resize(size_);
		}

	}

	// �������ݵ������
	template<typename T>
	void Figures<T>::push_back(string filed, size_t id, T value){
		if (id >= size_){
			resize(id+1);
		}
		if (id >= (*this)[filed].size()){

		}
		if (id >= figure_.size()){	// ���ʣ�����ط�ΪʲôҪresize��Ϊʲô��id���������Ƚϣ�
			resize(id + 1);
		}
		(*this)[filed][id].val = value;
	}

	// ���ز��������
	template<typename T>
	vector<Cell<T>>& Figures<T>::operator[](string& filed){
		auto iter = figure_.find(filed);
		if (iter == figure_.end()){
			figure_[filed].resize(size_);
		}
		return figure_[filed];
	}

	template<typename T>
	vector<Cell<T>>& Figures<T>::operator[](const char* filed){
		return figure_[string(filed)];
	}

	// ������ݱ�
	template<typename T>
	void Figures<T>::clear(){
		resize(0);
	}

	template<typename T>
	void Figures<T>::clearAll(){
		size_ = 0;
		figure_.clear();
	}

	// ������ݽ�����ļ�
	// ע�����д�ӡ����
	template<typename T>
	void Figures<T>::print2txt(string filename){
		FILE* fp = fopen(filename.c_str(), "wb+");

		// ��ӡ�кţ�ˮƽ��ӡ��
		fprintf(fp, "id\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%5d\t", i);
		}
		fprintf(fp, "\r\n");
		// ��ӡÿһ�е����ݣ�ˮƽ��ӡ��
		for (auto it : figure_){
			fprintf(fp, "%s\t", it.first.c_str());
			for (int j = 0; j < size_; ++j){
				if (it.second[j].mask){
					fprintf(fp, "%.4f\t", it.second[j].val);
				}
				else{
					fprintf(fp, "NAN\t");
				}
			}
			fprintf(fp, "\r\n");
		}
		fclose(fp);
	}

	// ������ݽ�����ļ�������ͼ���ļ�����
	template<typename T>
	void Figures<T>::print2txt_im(string filename, vector<string>& images){
		FILE* fp = fopen(filename.c_str(), "wb+");

		// ��ӡ�кţ�ˮƽ��ӡ��
		fprintf(fp, "id\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%5d\t", i);
		}
		fprintf(fp, "\r\n");

		// ��ӡname���൱���µ�һ�У���ˮƽ��ӡ��
		fprintf(fp, "name\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%s\t", images[i].c_str());
		}
		fprintf(fp, "\r\n");

		// ��ӡÿһ�����ݣ�ˮƽ��ӡ��
		for (auto it : figure_){
			fprintf(fp, "%s\t", it.first.c_str());
			for (int j = 0; j < size_; ++j){
				if (it.second[j].mask){
					fprintf(fp, "%.4f\t", it.second[j].val);
				}
				else{
					fprintf(fp, "NAN\t");
				}
			}
			fprintf(fp, "\r\n");
		}
		fclose(fp);
	}

	// ������ݽ�����ļ�������һ��ͼ���ļ�����
	template<typename T>
	void Figures<T>::print2txt_impair(string filename, vector<pair<string, string>>& imagepairs){
		FILE* fp = fopen(filename.c_str(), "wb+");

		fprintf(fp, "id\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%5d\t", i);
		}
		fprintf(fp, "\r\n");

		fprintf(fp, "left\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%s\t", imagepairs[i].first.c_str());
		}
		fprintf(fp, "\r\n");

		fprintf(fp, "right\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%s\t", imagepairs[i].second.c_str());
		}
		fprintf(fp, "\r\n");

		for (auto it : figure_){
			fprintf(fp, "%s\t", it.first.c_str());
			for (int j = 0; j < size_; ++j){
				if (it.second[j].mask){
					fprintf(fp, "%.4f\t", it.second[j].val);
				}
				else{
					fprintf(fp, "NAN\t");
				}
			}
			fprintf(fp, "\r\n");
		}
		fclose(fp);
	}

	// ��������Ļ
	template<typename T>
	void Figures<T>::print2scr(){
		for (auto it : figure_){
			printf("%s\t", it.first.c_str());
			for (int j = 0; j < size_; ++j){
				if (it.second[j].mask){
					printf("%.4f\t", it.second[j].val);
				}
				else{
					printf("NAN\t");
				}
			}
			printf("\r\n");
		}
	}

}