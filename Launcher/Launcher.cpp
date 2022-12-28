#define _CRT_SECURE_NO_WARNINGS //fopenを使うために必要

#include <Windows.h>
#include <Course/Course.h>
#include <header/Frenetcoordinate.h>
#include <Course/TableLinearInterporater.h>
#include <DataLogger/CopyParam.h>
#include <DataLogger/SaveLog.h>
#include <ParameterLoader/MyParameters.h>
#include "Pedestrian.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <random.h>
#include <plot.h>
#include <ped_function.h>
#include <update_vel_ref.h>


//Setting of shared memory
constexpr auto SHARED_MEMORY_NAME = L"MySharedMemory";
constexpr auto SHARED_MEMORY_SIZE = 9 * 6500;
static HANDLE hSharedMemory = NULL;
SharedData* shareddata;

FILE*gp = _popen("C:\\MPCsimulation\\gnuplot\\bin\\gnuplot.exe ", "w");//プロットのため

char path[] = "\"C:\\MPCsimulation\\Optimization\\exe\\x64\\Release\\Optimization.exe\"";

bool CreateSharedMemory(const wchar_t* sharedMemoryName, DWORD size)
{
	if (hSharedMemory)
	{
		return FALSE;
	}

	hSharedMemory = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, size, sharedMemoryName);
	if (!hSharedMemory || hSharedMemory == INVALID_HANDLE_VALUE)
	{
		return FALSE;
	}

	return TRUE;
}

bool InitializeSharedMemory()
{
	hSharedMemory = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, SHARED_MEMORY_NAME);

	if (!hSharedMemory || hSharedMemory == INVALID_HANDLE_VALUE)
	{
		return FALSE;
	}

	return TRUE;
}

void UnInitializeSharedMemory()
{
	if (hSharedMemory || hSharedMemory != INVALID_HANDLE_VALUE)
	{
		CloseHandle(hSharedMemory);
	}
}

bool ReadSharedMemory(DWORD size)
{
	if (!hSharedMemory)
	{
		return FALSE;
	}

	shareddata = (SharedData*)MapViewOfFile(hSharedMemory, FILE_MAP_ALL_ACCESS, NULL, NULL, size);
	if (shareddata == NULL || !shareddata)
	{
		return FALSE;
	}

	return TRUE;
}

void InitSetting(vector<vector<double>> course, Prm prm, double vel_ref)
{
	for (int i = 0; i < cprm_num; i++)
	{
		for (int j = 0; j < csize; j++)
		{
			shareddata->course[i][j] = course[i][j];
		}
	}
	shareddata->method = prm.Method;
	shareddata->vel_ref = vel_ref;
	shareddata->T_delta = prm.T_delta;
	shareddata->eps = prm.eps;
	shareddata->l_f = prm.l_f;
	shareddata->l_r = prm.l_r;
	shareddata->width = prm.width;
	shareddata->dist_front = prm.dist_front;
	shareddata->dist_rear = prm.dist_rear;
	shareddata->theta_front = prm.theta_front;
	shareddata->theta_rear = prm.theta_rear;
	shareddata->a11 = prm.a11;
	shareddata->a12 = prm.a12;
	shareddata->a21 = prm.a21;
	shareddata->a22 = prm.a22;
	shareddata->b1 = prm.b1;
	shareddata->b2 = prm.b2;
	shareddata->Q_vel = prm.Q_vel;
	shareddata->Q_acc = prm.Q_acc;
	shareddata->Q_v = prm.Q_v;
	shareddata->Q_v_dot = prm.Q_v_dot;
	shareddata->Q_v_2dot = prm.Q_v_2dot;
	shareddata->Q_theta = prm.Q_theta;
	shareddata->Q_theta_dot = prm.Q_theta_dot;
	shareddata->Q_theta_2dot = prm.Q_theta_2dot;
	shareddata->Q_delta = prm.Q_delta;
	shareddata->Q_delta_dot = prm.Q_delta_dot;
	shareddata->Sf_vel = prm.Sf_vel;
	shareddata->Sf_acc = prm.Sf_acc;
	shareddata->Sf_v = prm.Sf_v;
	shareddata->Sf_v_dot = prm.Sf_v_dot;
	shareddata->Sf_v_2dot = prm.Sf_v_2dot;
	shareddata->Sf_theta = prm.Sf_theta;
	shareddata->Sf_theta_dot = prm.Sf_theta_dot;
	shareddata->Sf_theta_2dot = prm.Sf_theta_2dot;
	shareddata->Sf_delta = prm.Sf_delta;
	shareddata->Sf_delta_dot = prm.Sf_delta_dot;
}

void InitState(double u_start, double v_start, double theta_start, double vel_start, double T_delta)
{
	for (int i = 0; i < vsize; i++)
	{
		shareddata->u[i] = u_start + 3 * T_delta * i;
		shareddata->vel[i] = vel_start;
		shareddata->acc[i] = 0;
		shareddata->v[i] = v_start;
		shareddata->v_dot[i] = 0;
		shareddata->v_2dot[i] = 0;
		shareddata->theta[i] = theta_start;
		shareddata->theta_dot[i] = 0;
		shareddata->theta_2dot[i] = 0;
		shareddata->delta[i] = 0;
		shareddata->delta_dot[i] = 0;
		//
		shareddata->vel_ref_pre[i] = vel_start;
	}
	shareddata->success = 0;
}

void UpdateState()
{
	for (int i = 0; i < vsize - 1; i++)
	{
		shareddata->u[i] = shareddata->u[i + 1];
		shareddata->vel[i] = shareddata->vel[i + 1];
		shareddata->acc[i] = shareddata->acc[i + 1];
		shareddata->v[i] = shareddata->v[i + 1];
		shareddata->v_dot[i] = shareddata->v_dot[i + 1];
		shareddata->v_2dot[i] = shareddata->v_2dot[i + 1];
		shareddata->theta[i] = shareddata->theta[i + 1];
		shareddata->theta_dot[i] = shareddata->theta_dot[i + 1];
		shareddata->theta_2dot[i] = shareddata->theta_2dot[i + 1];
		shareddata->delta[i] = shareddata->delta[i + 1];
		shareddata->delta_dot[i] = shareddata->delta_dot[i + 1];
	}
}





void Launch(vector<vector<double>> course, CourseSetting setting, Frenet frenet, double u_start, double u_end, double v_start, double theta_start, double vel_ref,int attempt_num,int loop_num)
{
	if (!CreateSharedMemory(SHARED_MEMORY_NAME, SHARED_MEMORY_SIZE))
	{
		cout << "共有メモリの作成に失敗しました。\n";
	}

	if (!InitializeSharedMemory())
	{
		cout << "共有メモリの初期化に失敗しました。\n";
	}

	if (!ReadSharedMemory(SHARED_MEMORY_SIZE))
	{
		cout << "共有メモリの読み取りに失敗しました。\n";
	}

	//csv読み込み
	RTCLib::CSVLoader CSV_prm("C:\\MPCsimulation\\Common\\Parameter_setting\\parameter.csv", 1);
	Prm prm;
	prm.Load_Prm(CSV_prm, 0);

#ifdef PD	//歩行者の初期設定
	Pedestrian ped(vel_ref, course[5], shareddata,attempt_num,loop_num);
	ped_func ped_func;
#endif //PD
	//参照速度の予測用
	update_vel_ref up_vel;

	//線形補間用
	LinearInterporater table;
	table.GetCourse(course);

	//結果を出力するcsv
	DataLogger logger_MPC;
	DataLogger logger_Course;
	//パラメータファイルコピー
	SaveParam saveparam;
	logger_MPC.Open(CreateLogFileName("data", "mpc_", setting));
	logger_Course.Open(CreateLogFileName("data", "course_", setting));
	saveparam.save_prm(CreateLogFileName("data", "prm_", setting));
	//ログの保存
	SetData_MPC(logger_MPC, shareddata);
	OutData_Course(logger_Course, course);
	InitSetting(course, prm, vel_ref);
	InitState(u_start, v_start, theta_start, vel_ref, prm.T_delta);
	
#ifdef PLOT
	//plotのクラスの呼び出し
	plot plot;


	//コースのプロット
	plot.plot_course(course[0], course[1], course[6], course[7], course[8], course[9],shareddata);

	//結果のプロットの準備
	plot.result_course_plot(shareddata);
#endif//PLOT


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	shareddata->trigger = 0;
	//初期の評価関数を保持
	ped_func.preserve_init_weight(shareddata);

	shareddata->attempt_count = loop_num;
	
	//loop
	while (shareddata->u[0] < u_end)
	{
		//ただ参照速度のベクトルをshareddataに書き込んでるだけ
		for (int i = 0; i < vsize; i++)
		{
			shareddata->vel_ref_pre[i] = up_vel.down_vel(shareddata)[i];
		}

#ifdef PD
		ped_func.ped_prediction(ped, prm.T_delta, shareddata);
		ped_func.UpdatePed_run_out(ped, prm.T_delta, vel_ref, shareddata);
		ped_func.TTC(ped, shareddata);
		ped_func.collision_judge(ped, shareddata);

#endif //PD
#ifdef vel_ref_down
		for (int i = 0; i < vsize; i++)
		{
			shareddata->vel_ref_pre[i] = ped_func.down_vel(ped, prm.T_delta, shareddata)[i];
		}
#endif //vel_ref_down



		system(path);
		UpdateState();


#ifdef PLOT
		plot.result_plot(shareddata,gp);//結果プロット
#endif//PLOT

		
		
		if (shareddata->error_code != 0)
		{
			printf("------------infeasible------------\n");

			//whileの時のエラー発生時のための処置
			shareddata->count_error++;
			if (shareddata->count_error >= 10)
			{
				break;
			}

		}

		if (!ReadSharedMemory(SHARED_MEMORY_SIZE))
		{
			std::cout << "共有メモリの読み取りに失敗しました。\n";
		}
		logger_MPC.PrintData();
		std::cout << shareddata->u[0] << std::endl;
	}

	UnInitializeSharedMemory();


}

void SetFrenet(vector<vector<double>>& course, CourseSetting setting, Frenet& frenet)
{
	frenet.frenetlib.LoadPath(course[0], course[1], false);
	frenet.frenetlib.OutputRho(course);

#ifdef OA
	Frenet temp_frenet;
	temp_frenet.frenetlib.LoadPath(course[2], course[3], false);
	temp_frenet.frenetlib.OutputRho(course);
#endif // OA

	double temp_theta;
	for (size_t i = 0; i < course[0].size(); i++)
	{
#ifndef OA
		frenet.Cache_f = frenet.frenetlib.GetFrenet(course[0][i], course[1][i], 0.0, course[2][i], course[3][i], temp_theta, frenet.Cache_f); //u, vを取得
#endif // OA
		frenet.Cache_g = frenet.frenetlib.GetGlobal(course[2][i], course[4][i], 0.0, course[6][i], course[7][i], temp_theta, frenet.Cache_g); //制約y_minをfrenet->global
		frenet.Cache_g = frenet.frenetlib.GetGlobal(course[2][i], course[5][i], 0.0, course[8][i], course[9][i], temp_theta, frenet.Cache_g); //制約y_maxをfrenet->global
	}
	frenet.Cache_f.initialized = false;
	frenet.Cache_g.initialized = false;
}

int main()
{
	CourseSetting setting;
	GenCourse gencourse;
	vector<vector<double>> course;
	Frenet frenet;

#ifdef OA
	setting.a = 2.5;
	setting.width_1 = 0.7;
	setting.width_2 = 0.7;
	setting.dist = 13;
	setting.pos1 = "upper";
	setting.pos2 = "lower";
	double u_start = 25;
	double u_end = 80;
	double v_start = 0;
	double theta_start = 0;
	double vel_ref = 6;

	gencourse.GetSetting(setting);
	course = gencourse.Gen_SOA();
	SetFrenet(course, setting, frenet);
	Launch(course, setting, frenet, u_start, u_end, v_start, theta_start, vel_ref);
#endif // OA

#ifdef SINE
	setting.cycle = 80;
	setting.ampl = 30;
	double u_start = 0;
	double u_end = 80;
	double v_start = 0;
	double theta_start = 0;
	double vel_ref = 6;

	gencourse.GetSetting(setting);
	course = gencourse.Gen_SINE();
	SetFrenet(course, setting, frenet);
	Launch(course, setting, frenet, u_start, u_end, v_start, theta_start, vel_ref);
#endif // SINE

#ifdef CSV
	setting.Path_coursecsv = "C:\\MPCsimulation\\py_course\\pd_st100.csv"; //Path of course csv //pedestrian// pd_st100.csv
	double u_start = 10; //Initial u
	double u_end = 80; //goal of u
	double v_start = 0; //Initial v
	double theta_start = 0; //Initial theta

	// vel_ref[km/h] =   10   ,   15  ,    20  ,   25   ,   30   ,    35  ,   40    ,   45,    50   ,    55
	// vel_ref[m/s]  = 2.77778,4.16667, 5.55556, 6.94444, 8.33333, 9.72222, 11.11111, 12.5, 13.88889, 15.27778
	double vel_ref = 9.72222; //Reference velocity 

	int attempt_num = 3;//繰り返し回数
	int count = 1;


	for (int vel_count = 3; vel_count < 6; vel_count++)
	{
		vel_ref = 2.77778 + 1.38889 * vel_count;
		while (count <= attempt_num) 
		{
			course = gencourse.Gen_Course_csv(setting.Path_coursecsv);
			SetFrenet(course, setting, frenet);
			Launch(course, setting, frenet, u_start, u_end, v_start, theta_start, vel_ref, attempt_num, count);
	 		frenet.Cache_f.initialized = false;
			frenet.Cache_g.initialized = false;
			count++;
		}
	}

	//while (count <= attempt_num) {
	//	course = gencourse.Gen_Course_csv(setting.Path_coursecsv);
	//	SetFrenet(course, setting, frenet);
	//	Launch(course, setting, frenet, u_start, u_end, v_start, theta_start, vel_ref,attempt_num,count);
	//	frenet.Cache_f.initialized = false;
	//	frenet.Cache_g.initialized = false;
	//	count++;
	//}

#endif // CSV

	return 0;
}