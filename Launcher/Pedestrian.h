#pragma once
#include <random.h>

struct Pedestrian
{

	double x_pd, y_pd, vel_pd;
	double x_pd_mpc, y_pd_mpc, vel_pd_mpc;
	double x_pd_start, y_pd_start, vel_pd_start;
	double closs_range;
	double vel_ref;
	vector<double>  course_width;
	double range_min, range_max;
	double closs_y_pd;
	int action_num;
	double Q_pena_ped;
	

	random_num rand_num;


	//初期値を決定
	// 30 <= x_pd <= 60 delta=1
	// y_pd=2.0 で固定
	//0.5 <= vel_pd <= 1.5 delta=0.00001
	//range_min <= closs_range <= range_max

	Pedestrian(double vel_ref, std::vector<double> course_width, SharedData* shareddata,int attempt_num,int loop_num) {

		this->vel_ref = vel_ref;
		this->course_width = course_width;

		x_pd_mpc = 50;// x_pd_start;//無限遠にいるとする
		y_pd_mpc = 3.5;//y_pd_start;//無限遠にいるとする
		vel_pd_mpc = 0;

		x_pd_start = 50.0;
		y_pd_start = 3.0;
		//vel_pd_start = 0.75;



		int select_vel;
		//int select_vel = rand_num.Make_num() % 4;
		 select_vel = 0;

		if (select_vel == 0) {
			vel_pd_start = 0.88889;//slow
		}
		else if (select_vel == 1) {
			vel_pd_start = 1.11111;//normal
		}
		else if (select_vel == 2) {
			vel_pd_start = 1.55556;//fast
		}
		else if (select_vel == 3) {
			vel_pd_start = 1.77778;//jog
		}

		x_pd = x_pd_start;
		y_pd = y_pd_start;
		vel_pd = vel_pd_start;

		 int action_num = 0;
		// 0:normal // 1:slow->fast // 2:fast->slow // 3:normal->stop // 4:normal->back // 5:danger (go->stop->go->stop->go)
		// action_num=5 はACTIONのdefineをoffにして使う
		


		range_min = (-1.0) * vel_ref * course_width[0] / vel_pd_start;
		range_max = vel_ref * course_width[0] / vel_pd_start;


		Q_pena_ped = 0;

		

		
		closs_range = 0;

#ifdef WHILE
		closs_range = 2.0 * range_max * loop_num / (attempt_num)+range_min;
#endif //WHILE
		

		closs_y_pd = vel_pd_start * closs_range / vel_ref;//x_car=x_pdのときの歩行者のy座標
		
		shareddata->closs_range = closs_range;
		shareddata->closs_y_pd = closs_y_pd;
		shareddata->action_num = action_num;
		shareddata->vel_pd_num = select_vel;
		shareddata->x_pd_mpc = x_pd_mpc;
		shareddata->y_pd_mpc = y_pd_mpc;
		shareddata->vel_pd_mpc = vel_pd_mpc;
		shareddata->Q_pena_ped = Q_pena_ped;


	}


};