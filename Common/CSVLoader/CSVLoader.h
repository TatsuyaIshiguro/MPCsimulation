#pragma once

#ifndef RTCLIB_CSV_LOADER2
#define RTCLIB_CSV_LOADER2

//! ////////////////////////////////////////////////////////////////////////////////////
/// 
/// 自分のための，Realtime制御フレームワーク整備事業  2011 / Jun - 2011? / ??
///                                             Copyright @ 奥田裕之 2011 
/// 
/// CSVLoader class : 簡便なCSV読み込み(NumericCSVLoaderの改良版．ヘッダのみで使用可能)
///
/// 要件：C++, STL, Boost C++ library
/// 
/// ///////////////////////////////////////////////////////////////////////////////////////
///  
/// //---------------- 使い方 ----------------------
///
/// コンストラクタとしてデータファイル名を与えるだけ．
/// 二番目の引数には，ヘッダとして読み飛ばすべき行数を入れる．
/// 詳細なエラー処理が無いため，CSVとしての要件を守ること．
/// データ構造としては，行ベクトルを保持するベクトルになる．
///
/// //読み込み
///	RTCLib::CSVLoader csv("demo_data001.csv",1);
///
/// //読み込んだデータへのアクセス
/// int i=0; // 行
/// int j=0; // 列
/// cout << csv[i][j];
/// 
/// -----------------[CSVファイルの要件]------------
/// ・ヘッダ行数が指定と正しいこと
/// ・コメント行などが無いこと
/// ・途中，データ行には文字が入っていないこと．
///
/// -----------------[その他注意]----------------------
/// Boost::Tokenizerを使っているので，大量のデータに対しては遅いかもしれない．
/// 
/// --------------------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <vector>

#include "Utilities.h"

namespace RTCLib{

	// template<class SendObject, class RecvObject>
	class CSVLoader
	{
	private:
		/// 読み込んだファイル名．フルパスに変換し格納するべし
		std::string m_filename;
		/// 読み込みが正常に行われているか
		bool m_valid;

		/// 列データ
		typedef std::vector<double> COL;
		typedef boost::shared_ptr<COL> pCOL;

		/// 保持データ
		std::vector< pCOL > m_data;

		/// ヘッダ文字列
		std::vector< std::string > m_headers;

		/// 縦横
		int m_rows;
		int m_cols;

	public:
		/// コンストラクタ
		/// 値の初期化のみ
		/// NumericCSVLoader();
		/// 読み込むべきファイル名つきコンストラクタ
		/// 初期化と同時に読み込む
		CSVLoader()
			: m_valid(false), m_rows(0), m_cols(0)
		{
		}
		CSVLoader(std::string fn, int header_lines)
			: m_valid(false), m_rows(0), m_cols(0)
		{
			m_filename = fn;
			Load(fn, header_lines);
		}

		/// デストラクタ
		~CSVLoader(){};

		/// 読み込みが正常に行われているか
		bool IsValid(){return m_valid;}

		/// ファイルを読み込む
		int Load(std::string fn, int header_lines)
		{
			std::ifstream ifs;
			ifs.open(fn.c_str(), std::ios::in);

			// 開くの失敗
			if( ifs.fail() )
			{
				std::cerr << "File not found." << std::endl;
				return -1;
			}

			// 読み込みバッファ
			std::string line;

			// ヘッダ分の読み飛ばし
			int h_count = 0;
			while( ! ifs.eof() && h_count < header_lines )
			{
				getline( ifs,  line );
				h_count ++;
			}
			//ヘッダ解析(ヘッダの列数に合わせる)
			Scan_Header( line );

			// データ読み込み
			int count = 0;
			while( ! ifs.eof() )
			{
				getline( ifs,  line );
				if( trimstring(line).empty() )continue;
				pCOL tmp(new COL);
				tmp->reserve(m_cols);

				Parse_Line( line, *tmp );
				if( m_cols == 0 ) m_cols = tmp->size();
				if( m_cols != tmp->size())
				{
					std::cerr << "This function can not treat jag array." << std::endl;
				}

				m_data.push_back(tmp);
				count ++;

			}
			m_rows = m_data.size();
			m_valid = true;
			return 0;
		}

		/// サイズ
		int Rows(){return m_rows;};
		int Cols(){
			return m_cols;
		};

		/// 表示
		void Print( std::ostream &os )
		{
			if(!IsValid())return;
			for( std::vector<std::string>::iterator i = m_headers.begin(); i!=m_headers.end(); i++ )
			{
				os << *i;
				if( i + 1 != m_headers.end() ) os << ',';
			}
			os << std::endl;

			for(int i=0; i < m_rows; i++ )
			{
				for(int j=0; j < m_cols; j++)
				{
					os << (*m_data.at(i) )[j];
					if( j != m_cols - 1)
						os << ',';
				}
				os << std::endl;
			}
		}

		/// 行要素へのアクセス．行超過に関しては，最終行が取得される．
		/// 行要素はvectorで定義されているので，NumericCSVLoader[i][j]でアクセス可能
		/// (iが行，jが列)
		const std::vector<double>& operator[](const int &n) const
		{
			if(n<m_rows)
				return (*m_data[n] );
			else
				return (*m_data[m_rows-1] );
		}	

		/// ヘッダ文字列取得
		const std::string& Header(int n)const {return m_headers[n];}

		/// ヘッダ検索:列番号を返す．見つからなければ-1を返す．
		int GetColOf( const std::string &label )
		{
			std::vector<std::string>::iterator pos = find( m_headers.begin(), m_headers.end(), label );
			if( pos == m_headers.end() )
				return -1;
			else
				return (pos - m_headers.begin());
		}


		int GetColOf( const char *label )
		{
			return GetColOf( std::string( label));
		}

	private:
		void Scan_Header( std::string headerstr )
		{
			typedef  boost::escaped_list_separator<char> esc_sep;
			typedef  boost::tokenizer<esc_sep > tokenizer;

			esc_sep sep('@', ',', '\"');
			tokenizer tokens(headerstr, sep);

			for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
				std::string trimed =  trimstring(*tok_iter);
				if( trimed != "")
					m_headers.push_back( trimed );
				//  cout << m_headers.back() << endl;
			}
			m_cols = m_headers.size();
		}

		void Parse_Line( std::string &line, std::vector<double> &out )
		{
			if( line.empty() )return;

			typedef boost::escaped_list_separator<char> esc_sep;
			typedef boost::tokenizer<esc_sep > tokenizer;

			esc_sep sep('@', ',', '\"');
			tokenizer tokens(line, sep);
			std::string token;

			for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
				token = trimstring( *tok_iter ); 
				out.push_back ( atof(token.c_str()) );
			}
		}

		std::string trimstring(const std::string& s)
		{
			if(s.length() == 0)
				return s;
			int b = s.find_first_not_of(" \t\r\n");
			int e = s.find_last_not_of(" \t\r\n");
			if(b == -1) // 左右両端に、スペース、タブ、改行しかない。
				return "";
			return std::string(s, b, e - b + 1);
		}
	};

	//! CSVLoaderのデータから線形補間でデータを取得する．
	//! 引数となる変数の軸と，値を設定するべし．
	class TableLinearInterporater
	{

	public:

		// data
		RTCLib::CSVLoader* m_data;
		int i_prev_index;

		// last index for cache
		// under construction

	public:
		TableLinearInterporater() : m_data(nullptr) {};
		TableLinearInterporater(RTCLib::CSVLoader &data)
			: m_data(&data)
		{
			i_prev_index = 0;
		}


		//! 線形補間データを計算する．
		//! x_index : 引数となるデータの列番号，たとえば時間データの列番号
		//! x : 値を得たい引数(横軸)，たとえば時間など
		//! y_index : 得たい値の出力(縦軸)，たとえば速度などの列番号
		double GetLinearInterp(int x_index, double x, int y_index)
		{
			// return value
			double ret = 0;

			int rows = m_data->Rows();
			int cols = m_data->Cols();
			if (cols <= x_index || cols <= y_index || rows == 0)
			{
				throw std::out_of_range("out of range at linear interporation.");
			}

			// data is only one;
			if (rows == 1) {
				return (*m_data)[0][y_index];
			}

			// search speed 
			double prev_val = 0, prev_x = 0;
			double next_val = 0, next_x = 0;
			int i = 0;
			// search x index
			double bx = (*m_data)[i_prev_index][x_index];
			if (bx <= x)
			{
				for (i = i_prev_index; i < m_data->Rows(); i++) {
					double nx = (*m_data)[i][x_index];
					if (x <= nx)
					{
						break;
					}
				}
			}
			else
			{
				for (i = i_prev_index; 0 < i; i--) {
					double nx = (*m_data)[i][x_index];
					if (nx < x)
					{
						i = i + 1;
						break;
					}
				}
			}

			if (i == 0)
			{	// start or before start
				ret = (*m_data)[0][1];
			}
			else if (i == m_data->Rows())
			{	// end or after end
				ret = (*m_data)[m_data->Rows() - 1][x_index];
			}
			else {
				// middle
				prev_x = (*m_data)[i - 1][x_index];
				prev_val = (*m_data)[i - 1][y_index];

				next_x = (*m_data)[i][x_index];
				next_val = (*m_data)[i][y_index];
				// interporation
				ret = linear_interporate(x, prev_x, next_x, prev_val, next_val);
			}
			i_prev_index = i;
			return ret;
		}
	};
}

#endif //RTCLIB_CSV_LOADER2