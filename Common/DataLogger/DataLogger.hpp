#pragma once
#include <string>
#include <vector>
#include <fstream>

using namespace std;

class DataLogger {
private:
	class DataInterface {
	public:
		virtual void PrintHeader(ofstream &ofs, const bool isFirst) = 0;
		virtual void PrintData(ofstream &ofs, const bool isFirst) = 0;
	};

	template <typename T>class Data : public DataInterface {
	public:
		string header;
		T* data;
		Data(string header, T &data) {
			this->header = header;
			this->data = &data;
		}
		void PrintHeader(ofstream &ofs, const bool isFirst) {
			if (isFirst) {
				ofs << header;
			}
			else {
				ofs << "," << header;
			}
		}
		void PrintData(ofstream &ofs, const bool isFirst) {
			if (isFirst) {
				ofs << *data;
			}
			else {
				ofs << "," << *data;
			}
		}
	};
	
	vector<shared_ptr<DataInterface>> dataList;
	ofstream ofs;
public:
	template<class T> void push_back(string str, T &data) {
		dataList.push_back(shared_ptr<DataInterface>(new Data<T>(str,data)));
	}
	void Open(string fileName) {
		ofs.open(fileName);
	}
	~DataLogger() {
		if (ofs.is_open()) {
			ofs.close();
		}
	}
	void PrintHeader() {
		if (dataList.size() == 0)
			return;
		dataList[0]->PrintHeader(ofs, true);
		for (size_t i = 1; i < dataList.size(); i++)
		{
			dataList[i]->PrintHeader(ofs, false);
		}
		ofs << endl;
	}

	void PrintData() {
		if (dataList.size() == 0)
			return;
		dataList[0]->PrintData(ofs, true);
		for (size_t i = 1; i < dataList.size(); i++)
		{
			dataList[i]->PrintData(ofs, false);
		}
		ofs << endl;
	}
};