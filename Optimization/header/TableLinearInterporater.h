#pragma once
#include <iostream>
#include <stdexcept>
#include <Data/Data.h>

//目標点を探索するために使用
struct Points_search
{
	double c_current_x;
	double c_current_y;
	double c_next_x;
	double c_next_y;
};

class LinearInterporater
{
public:
private:
	Points_search Points;
	std::vector<std::vector<double>> c_data;
	int i_prev_index;
	int i_prev_index_pp;

public:

	void GetCourse(double course[cprm_num][csize])
	{
		c_data.resize(5);
		for (size_t i = 0; i < c_data.size(); i++)
		{
			c_data[i].resize(csize);
		}

		for (size_t i = 0; i < csize; i++)
		{
			c_data[0][i] = course[2][i]; //u
			c_data[1][i] = course[5][i]; //v_max
			c_data[2][i] = course[4][i]; //v_min
			c_data[3][i] = course[3][i]; //v
			c_data[4][i] = course[10][i]; //rho
		}
		i_prev_index = 0;
		i_prev_index_pp = 0;
	}

	//! 線形補間データを計算する．
	//! x_index : 引数となるデータの列番号，たとえば時間データの列番号
	//! x : 値を得たい引数(横軸)，たとえば時間など
	//! y_index : 得たい値の出力(縦軸)，たとえば速度などの列番号
	double GetLinearInterporation(int x_index, double x, int y_index)
	{
		double ret = 0;

		int cols = c_data.size();
		int rows = c_data[0].size();
		if (cols <= x_index || cols <= y_index)
		{
			throw std::out_of_range("out of range at linear interporation.");
		}

		double prev_val = 0, prev_x = 0;
		double next_val = 0, next_x = 0;
		int i = 0;
		double nx = c_data[x_index][i_prev_index];
		if (nx <= x)
		{
			for (i = i_prev_index; i < rows; i++) {
				double nx = c_data[x_index][i];
				if (x <= nx)
				{
					break;
				}
			}
		}
		else
		{
			for (i = i_prev_index; 0 < i; i--) {
				double nx = c_data[x_index][i];
				if (nx < x)
				{
					i = i + 1;
					break;
				}
			}
		}

		if (i == 0)
		{	// start or before start
			ret = c_data[y_index][0];
		}
		else if (i == rows)
		{	// end or after end
			ret = c_data[y_index][rows];
		}
		else {
			// middle
			prev_x = c_data[x_index][i - 1];
			prev_val = c_data[y_index][i - 1];

			next_x = c_data[x_index][i];
			next_val = c_data[y_index][i];
			// interporation
			ret = linear_interporate(x, prev_x, next_x, prev_val, next_val);
		}
		i_prev_index = i;
		return ret;
	}

	void GetTarget(double ret[], int x_index, int y_index, double x, double y, double Lf)
	{
		for (int k = 0; k < 2; k++) {
			ret[k] = 0.0;
		}

		int cols = c_data.size();
		int rows = c_data[0].size();
		if (cols <= x_index || cols <= y_index || rows == 0)
		{
			throw std::out_of_range("out of range for target point.");
		}

		int i = 0;
		// search x index，ここでtargetpointのx座標取得
		for (i = i_prev_index_pp; i < rows; i++) {
			double nearest_point_distance = calc_distance(x, y, c_data[x_index][i], c_data[y_index][i]);
			double next_point_distance = calc_distance(x, y, c_data[x_index][i + 1], c_data[y_index][i + 1]);
			if (0 < (next_point_distance - nearest_point_distance) && nearest_point_distance < 4.0) //ポイントが徐々に近くなり、2m以下
			{
				break;
			}
		}
		if (i == 0)
		{	// start or before start
			ret[0] = c_data[x_index][0];
			ret[1] = c_data[y_index][0];
		}
		else if (i == rows)
		{	// end or after end
			ret[0] = c_data[x_index][rows - 1];
			ret[1] = c_data[y_index][rows - 1];
		}
		else {
			// middle

			int j = i;
			i_prev_index = i; //今回のインデックスを保存

			while (calc_distance(x, y, c_data[x_index][j], c_data[y_index][j]) < Lf)
			{
				if (rows <= (j + 1))
				{
					break;
				}
				j++;
			}
			Points.c_current_x = c_data[x_index][j - 1];
			Points.c_current_y = c_data[y_index][j - 1];
			Points.c_next_x = c_data[x_index][j];
			Points.c_next_y = c_data[y_index][j];

			ret[0] = calc_targetpoint_x(x, y, Points, Lf);
			ret[1] = calc_targetpoint_y(x, y, Points, Lf, ret[0]);
		}
	}

	double linear_interporate(double x, double x0, double x1, double y0, double y1)
	{
		double alpha = (x - x0) / (x1 - x0);
		double ret = y0 + alpha * (y1 - y0);
		return ret;
	}

	double calc_distance(double x, double y, double path_x, double path_y) {
		double dx = x - path_x;
		double dy = y - path_y;
		double distance = sqrt(dx * dx + dy * dy);
		return distance;
	}

	double calc_targetpoint_x(double x_current, double y_current, Points_search P, double L) {
		if (P.c_current_x == P.c_next_x) {
			return P.c_current_x;
		}
		else {
			double a = (P.c_next_y - P.c_current_y) / (P.c_next_x - P.c_current_x);
			double b = -a * P.c_current_x + P.c_current_y - y_current;
			double temp_x_minus = (-(a * b - x_current) - sqrt((a * b - x_current) * (a * b - x_current) - (1 + a * a) * (x_current * x_current + b * b - L * L))) / (1 + a * a);
			double temp_x_plus = (-(a * b - x_current) + sqrt((a * b - x_current) * (a * b - x_current) - (1 + a * a) * (x_current * x_current + b * b - L * L))) / (1 + a * a);
			if (P.c_current_x <= temp_x_plus && temp_x_plus <= P.c_next_x) {
				return temp_x_plus;
			}
			else
			{
				return temp_x_minus;
			}
		}
	}

	double calc_targetpoint_y(double x_current, double y_current, Points_search P, double L, double target_x) {
		if (P.c_current_x == P.c_next_x) {
			double temp_y_minus = y_current - sqrt(L * L - (P.c_current_x - x_current) * (P.c_current_x - x_current));
			double temp_y_plus = y_current + sqrt(L * L - (P.c_current_x - x_current) * (P.c_current_x - x_current));
			if (P.c_current_y <= temp_y_plus && temp_y_plus <= P.c_next_y) {
				return temp_y_plus;
			}
			else
			{
				return temp_y_minus;
			}
		}
		else {
			double a = (P.c_next_y - P.c_current_y) / (P.c_next_x - P.c_current_x);
			return a * (target_x - P.c_current_x) + P.c_current_y;
		}
	}
};