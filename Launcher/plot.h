//#define _CRT_SECURE_NO_WARNINGS //fopenを使うために必要
#pragma once
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector> 
#include <Data/Data.h>
#include <algorithm>

class plot {
private:

public:
	void plot_course(std::vector<double> x_ref, std::vector<double> y_ref, std::vector<double> x_max, std::vector<double> y_max, std::vector<double> x_min, std::vector<double> y_min, SharedData* shareddata);
	void result_course_plot(SharedData* shareddata);
	void result_plot(SharedData* shareddata, FILE* gp);
};

inline void plot::plot_course(std::vector<double> x_ref, std::vector<double> y_ref, std::vector<double> x_max, std::vector<double> y_max, std::vector<double> x_min, std::vector<double> y_min, SharedData* shareddata)
{
	FILE* gp, * fp;
	gp = _popen("C:\\gnuplot\\bin\\gnuplot.exe -persist", "w");//gnuplotを起動//-persistがあるとずっと表示
	fprintf(gp, "unset key\n");//グラフ凡例の表示/非表示
	//  fprintf(gp, "set key left top\n");//グラフ凡例の位置の指定
	x_ref.resize(400);
	y_ref.resize(400);
	x_max.resize(400);
	y_max.resize(400);
	x_min.resize(400);
	y_min.resize(400);

	for (int i = 0; i < 400; i++) {
		x_ref[i] = shareddata->course[0][i];
		y_ref[i] = shareddata->course[1][i];
		x_max[i] = shareddata->course[8][i];
		y_max[i] = shareddata->course[9][i];
		x_min[i] = shareddata->course[6][i];
		y_min[i] = shareddata->course[7][i];
	}
	double x_ref_max, y_ref_max, x_ref_min, y_ref_min;
	x_ref_max = *std::max_element(x_ref.begin(), x_ref.end());
	y_ref_max = *std::max_element(y_ref.begin(), y_ref.end());
	x_ref_min = *std::min_element(x_ref.begin(), x_ref.end());
	y_ref_min = *std::min_element(y_ref.begin(), y_ref.end());

	fprintf(gp, "set xrange[%f:%f]\n", x_ref_min - 10.0, x_ref_max + 10.0);//x軸の範囲を指定
	//fprintf(gp, "set xrange[%f:%f]\n",25.0, 75.0);
	fprintf(gp, "set yrange[% f:% f]\n", y_ref_min - 5.0, y_ref_max + 5.0);
	fprintf(gp, "set xlabel \"x\"\n");//x軸のラベルを指定
	fprintf(gp, "set ylabel \"y\"\n");

	fp = fopen("course.dat", "w");

	for (int i = 0; i < 400; i++)
	{
		fprintf(fp, "%f %f\n", x_max[i], y_max[i]);
	}

	fclose(fp);

	fp = fopen("course.dat", "a");

	for (int i = 0; i < 400; i++)
	{
		fprintf(fp, "%f %f\n", x_min[i], y_min[i]);
	}

	fclose(fp);

	fp = fopen("course.dat", "a");

	for (int i = 0; i < 400; i++)
	{
		fprintf(fp, "%f %f\n", x_ref[i], y_ref[i]);
	}

	fclose(fp);

	fprintf(gp, "plot 'course.dat'  pt 7 ps 0.4 lc 'red'\n");

	fclose(fp);
	fflush(gp);

	fprintf(gp, "exit\n");//gnuplotの終了
	_pclose(gp);
}


inline void plot::result_course_plot(SharedData* shareddata)
{
	FILE* fp;
	std::vector<double> x_max;
	std::vector<double> y_max;
	std::vector<double> x_min;
	std::vector<double> y_min;

	x_max.resize(400);
	y_max.resize(400);
	x_min.resize(400);
	y_min.resize(400);

	for (int i = 0; i < 400; i++) {
		x_max[i] = shareddata->course[8][i];
		y_max[i] = shareddata->course[9][i];
		x_min[i] = shareddata->course[6][i];
		y_min[i] = shareddata->course[7][i];
	}

	fp = fopen("result_point.dat", "w");

	for (int i = 0; i < 400; i++)
	{
		fprintf(fp, "%f %f\n", x_max[i], y_max[i]);
	}

	fclose(fp);

	fp = fopen("result_point.dat", "a");

	for (int i = 0; i < 400; i++)
	{
		fprintf(fp, "%f %f\n", x_min[i], y_min[i]);
	}

	fclose(fp);
}

inline void plot:: result_plot(SharedData* shareddata, FILE* gp)
{
	double x_MPC = shareddata->x[0];
	double y_MPC = shareddata->y[0];
#ifdef PD
	double x_pd = shareddata->x_pd;
	double y_pd = shareddata->y_pd;
#endif//PD

	std::vector<double> x_ref;
	std::vector<double> y_ref;


	x_ref.resize(400);
	y_ref.resize(400);

	for (int i = 0; i < 400; i++) {
		x_ref[i] = shareddata->course[0][i];
		y_ref[i] = shareddata->course[1][i];

	}
	double x_ref_max, y_ref_max, x_ref_min, y_ref_min;
	x_ref_max = *std::max_element(x_ref.begin(), x_ref.end());
	y_ref_max = *std::max_element(y_ref.begin(), y_ref.end());
	x_ref_min = *std::min_element(x_ref.begin(), x_ref.end());
	y_ref_min = *std::min_element(y_ref.begin(), y_ref.end());

	double x_range_min, x_range_max;

#ifdef SINE
	x_range_min = x_ref_min - 10.0;
	x_range_max = x_ref_max + 10.0;
#endif//SINE

#ifdef OA
	x_range_min = 25.0;
	x_range_max = 85.0;
#endif//OA

#ifdef CSV
	//x_range_min = x_ref_min - 10.0;
	//x_range_max = x_ref_max + 10.0;
	x_range_min = 0.0;
	x_range_max = 70.0;
#endif //CSV

	fprintf(gp, "set xrange[%f:%f]\n", x_range_min, x_range_max);//x軸の範囲を指定
	fprintf(gp, "set yrang[%f:%f]\n", y_ref_min - 5.0, y_ref_max + 5.0);
	fprintf(gp, "set xlabel \"x\"\n");//x軸のラベルを指定
	fprintf(gp, "set ylabel \"y\"\n");
	fprintf(gp, "unset key\n");//グラフ凡例の表示/非表示

	FILE* fp;

	//  fprintf(gp, "set key left top\n");//グラフ凡例の位置の指定

	fp = fopen("result_point.dat", "a");
	fprintf(fp, "%f %f\n", x_MPC, y_MPC);

#ifdef PD
	fprintf(fp, "%f %f\n", x_pd, y_pd);
#endif//PD

	fprintf(gp, "plot 'result_point.dat'  pt 7 ps 0.4 lc 'blue'\n");

	fclose(fp);
	fflush(gp);
}