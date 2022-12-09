#define _CRT_SECURE_NO_WARNINGS //fopen���g�����߂ɕK�v

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


//Setting of shared memory
constexpr auto SHARED_MEMORY_NAME = L"MySharedMemory";
constexpr auto SHARED_MEMORY_SIZE = 8 * 6500;
static HANDLE hSharedMemory = NULL;
SharedData* shareddata;

FILE*gp = _popen("C:\\gnuplot\\bin\\gnuplot.exe ", "w");//�v���b�g�̂���

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



//���s�҂̍X�V//���f�Ȃ�
void UpdatePed(Pedestrian& ped, double T_delta ,double vel_ref)
{
	double x_car, y_car;
	x_car = shareddata->x[0];
	y_car = shareddata->y[0];
	//double dist_Car_Pd = sqrt(pow(ped.x_pd - x_car, 2) + pow(ped.y_pd - y_car, 2));
	double closs_pd = vel_ref * ((ped.x_pd / vel_ref) - (ped.y_pd / ped.vel_pd));



	ped.x_pd = ped.x_pd;

	if (x_car+ped.closs_range >= closs_pd) {
		ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;
	}
	else {
		ped.y_pd = ped.y_pd;
	}


	//�f�[�^�̎󂯓n��
	shareddata->x_pd = ped.x_pd;
	shareddata->y_pd = ped.y_pd;
	shareddata->vel_pd = ped.vel_pd;
	shareddata->closs_pd = ped.closs_pd;

}

//���s�҂̍X�V//���f����
void UpdatePed_judge(Pedestrian& ped, double T_delta, double vel_ref, SharedData* shareddata)
{
	double x_car, y_car;
	x_car = shareddata->x[0];
	y_car = shareddata->y[0];
	double closs_pd = vel_ref * ((ped.x_pd / vel_ref) - (ped.y_pd / ped.vel_pd));
	int trigger = shareddata->trigger;
	ped.x_pd = ped.x_pd;
	if (trigger==0)
	{
		if (x_car + ped.closs_range >= closs_pd) { //���s�҂̓����o��
			ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;
			trigger = 1;
		}
		else {
			ped.y_pd = ped.y_pd;
		}
	}
	else if (trigger == 1)
	{
		if (ped.y_pd >= 1.7)//���H�킫�ɗ���
		{
			ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;
		}
		else 
		{
			ped.y_pd = ped.y_pd;
			trigger = 2;
		}
	}
	else if (trigger == 2)
	{
		random_num rand_num;
		//int judgement = rand_num.Make_num() % 3;//0=stop, 1=go, 2=go_fast
		int judgement = 1;
		if (judgement == 0) {
			ped.y_pd = ped.y_pd;
			ped.vel_pd = 0;
			trigger = 3;
		}
		else if (judgement == 1) {
			ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;
			trigger = 4;
		}
		else if (judgement == 2) {
			ped.vel_pd = 1.5 * ped.vel_pd;
			ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;
			trigger = 4;
		}

	}
	else if (trigger == 3) {
		ped.y_pd = ped.y_pd;
		if (x_car >= ped.x_pd + 5.0) {
			ped.vel_pd = ped.vel_pd_start;
			trigger = 4;
		}
	}
	else {
		ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;
	}

	shareddata->x_pd = ped.x_pd;
	shareddata->y_pd = ped.y_pd;
	shareddata->vel_pd = ped.vel_pd;
	shareddata->closs_pd = ped.closs_pd;
	shareddata->trigger=trigger ;
}



//�R�[�X�̕\��
void plot_course(std::vector<double> x_ref, std::vector<double> y_ref, std::vector<double> x_max, std::vector<double> y_max, std::vector<double> x_min, std::vector<double> y_min)
{
	FILE* gp, * fp;
	gp = _popen("C:\\gnuplot\\bin\\gnuplot.exe -persist", "w");//gnuplot���N��//-persist������Ƃ����ƕ\��
	fprintf(gp, "unset key\n");//�O���t�}��̕\��/��\��
	//  fprintf(gp, "set key left top\n");//�O���t�}��̈ʒu�̎w��
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

	fprintf(gp, "set xrange[%f:%f]\n", x_ref_min - 10.0, x_ref_max + 10.0);//x���͈̔͂��w��
	//fprintf(gp, "set xrange[%f:%f]\n",25.0, 75.0);
	fprintf(gp, "set yrange[% f:% f]\n", y_ref_min - 5.0, y_ref_max + 5.0);
	fprintf(gp, "set xlabel \"x\"\n");//x���̃��x�����w��
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

	fprintf(gp, "exit\n");//gnuplot�̏I��
	_pclose(gp);
}

//���ʂ̕\���̃R�[�X
void result_course_plot(SharedData* shareddata)
{
	FILE *fp;
	vector<double> x_max;
	vector<double> y_max;
	vector<double> x_min;
	vector<double> y_min;

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

//���ʂ̕\��(line)
void result_plot(SharedData* shareddata ,FILE*gp)
{
	double x_MPC = shareddata->x[0];
	double y_MPC = shareddata->y[0];
#ifdef PD
	double x_pd = shareddata->x_pd;
	double y_pd = shareddata->y_pd;
#endif//PD

	vector<double> x_ref;
	vector<double> y_ref;


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

	fprintf(gp, "set xrange[%f:%f]\n", x_range_min, x_range_max);//x���͈̔͂��w��
	fprintf(gp, "set yrang[%f:%f]\n", y_ref_min - 5.0, y_ref_max + 5.0);
	fprintf(gp, "set xlabel \"x\"\n");//x���̃��x�����w��
	fprintf(gp, "set ylabel \"y\"\n");
	fprintf(gp, "unset key\n");//�O���t�}��̕\��/��\��

	FILE * fp;
	//gp = _popen("C:\\gnuplot\\bin\\gnuplot.exe ", "w");


	fprintf(gp, "set term gif animate\n");	// �o�̓^�[�~�i����gif, animate�I�v�V������t����
	fprintf(gp, "set output 'pd_avoid.gif'\n");	// �o�̓t�@�C����
	

	//  fprintf(gp, "set key left top\n");//�O���t�}��̈ʒu�̎w��

	fp = fopen("result_point.dat", "a");
	fprintf(fp, "%f %f\n", x_MPC, y_MPC);

#ifdef PD
	fprintf(fp, "%f %f\n", x_pd, y_pd);
#endif//PD

	fprintf(gp, "plot 'result_point.dat'  pt 7 ps 0.4 lc 'blue'\n");

	fclose(fp);
	fflush(gp);
}



void Launch(vector<vector<double>> course, CourseSetting setting, Frenet frenet, double u_start, double u_end, double v_start, double theta_start, double vel_ref)
{
	if (!CreateSharedMemory(SHARED_MEMORY_NAME, SHARED_MEMORY_SIZE))
	{
		cout << "���L�������̍쐬�Ɏ��s���܂����B\n";
	}

	if (!InitializeSharedMemory())
	{
		cout << "���L�������̏������Ɏ��s���܂����B\n";
	}

	if (!ReadSharedMemory(SHARED_MEMORY_SIZE))
	{
		cout << "���L�������̓ǂݎ��Ɏ��s���܂����B\n";
	}

	//csv�ǂݍ���
	RTCLib::CSVLoader CSV_prm("C:\\MPCsimulation\\Common\\Parameter_setting\\parameter.csv", 1);
	Prm prm;
	prm.Load_Prm(CSV_prm, 0);

#ifdef PD	//���s�҂̏����ݒ�
	Pedestrian ped(vel_ref,course[5]);
#endif //PD

	//���`��ԗp
	LinearInterporater table;
	table.GetCourse(course);

	//���ʂ��o�͂���csv
	DataLogger logger_MPC;
	DataLogger logger_Course;
	//�p�����[�^�t�@�C���R�s�[
	SaveParam saveparam;
	logger_MPC.Open(CreateLogFileName("data", "mpc_", setting));
	logger_Course.Open(CreateLogFileName("data", "course_", setting));
	saveparam.save_prm(CreateLogFileName("data", "prm_", setting));
	//���O�̕ۑ�
	SetData_MPC(logger_MPC, shareddata);
	OutData_Course(logger_Course, course);
	InitSetting(course, prm, vel_ref);
	InitState(u_start, v_start, theta_start, vel_ref, prm.T_delta);
	


	//�R�[�X�̃v���b�g
	//plot_course(course[0], course[1], course[6], course[7], course[8], course[9]);

	//���ʂ̃v���b�g�̏���
	//result_course_plot(shareddata);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	shareddata->trigger = 0;


	//loop
	while (shareddata->u[0] < u_end)
	{
		system(path);
		UpdateState();
#ifdef PD
		//UpdatePed(ped,prm.T_delta,vel_ref);//���s��
		UpdatePed_judge(ped, prm.T_delta, vel_ref, shareddata);
#endif //PD
		
		//result_plot(shareddata,gp);//���ʃv���b�g

	

		if (!ReadSharedMemory(SHARED_MEMORY_SIZE))
		{
			std::cout << "���L�������̓ǂݎ��Ɏ��s���܂����B\n";
		}
		logger_MPC.PrintData();
		std::cout << shareddata->u[0] << std::endl;
	}

	UnInitializeSharedMemory();

	fprintf(gp, "set output \n");	// GIF�̏o��
	fprintf(gp, "set terminal wxt enhanced \n");	// GIF�̏o��

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
		frenet.Cache_f = frenet.frenetlib.GetFrenet(course[0][i], course[1][i], 0.0, course[2][i], course[3][i], temp_theta, frenet.Cache_f); //u, v���擾
#endif // OA
		frenet.Cache_g = frenet.frenetlib.GetGlobal(course[2][i], course[4][i], 0.0, course[6][i], course[7][i], temp_theta, frenet.Cache_g); //����y_min��frenet->global
		frenet.Cache_g = frenet.frenetlib.GetGlobal(course[2][i], course[5][i], 0.0, course[8][i], course[9][i], temp_theta, frenet.Cache_g); //����y_max��frenet->global
	}
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
	setting.Path_coursecsv = "C:\\py_course\\pd_st100.csv"; //Path of course csv //pedestrian// pd_st100.csv
	double u_start = 5; //Initial u
	double u_end = 80; //goal of u
	double v_start = 0; //Initial v
	double theta_start = 0; //Initial theta
	double vel_ref = 6; //Reference velocity defo=6

	

	course = gencourse.Gen_Course_csv(setting.Path_coursecsv);
	SetFrenet(course, setting, frenet);
	Launch(course, setting, frenet, u_start, u_end, v_start, theta_start, vel_ref);

#endif // CSV

	return 0;
}