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
	double cross_pd;
	int simu_num;
	double car_vel_ref;
	std::string action;

	

	random_num rand_num;



	//range_min <= closs_range <= range_max

	Pedestrian(double vel_ref, std::vector<double> course_width, SharedData* shareddata,int attempt_num,int loop_num,double u_start, int ped_act_num,int vel_pd_num) {

		this->vel_ref = vel_ref;
		this->course_width = course_width;
		simu_num = loop_num;
		car_vel_ref = vel_ref;

		x_pd_mpc = 50;// x_pd_start;//無限遠にいるとする
		y_pd_mpc = 3.5;//y_pd_start;//無限遠にいるとする
		vel_pd_mpc = 0;

		x_pd_start = 50.0;
		y_pd_start = 3.0;
		//vel_pd_start = 0.75;



		int select_vel;
		 select_vel = vel_pd_num;

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

		 int action_num = ped_act_num;
		// 0:normal // 1:slow->fast // 2:fast->slow // 3:normal->stop // 4:normal->back // 5:danger_1 (go->stop->go->stop->go) // 6:danger_2 (go->long_stop ->go)
		// action_num=5 はACTIONのdefineをoffにして使う
		 if (ped_act_num == 0) //等速直線運動（挙動変化無し）
		 {
			 action="straight";
		 }
		 else if (ped_act_num == 1) //車両回避挙動(車両を認識後に歩行者が車両を避けるような挙動を取ると仮定）
		 {
			 action = "avoid";
		 }
		 else if (ped_act_num == 2)//不規則な停止を繰り替えす挙動（危険な挙動変化）
		 {
			 action = "danger1";
		 }
		 else if (ped_act_num == 3)
		 {
			 action = "danger2";
		 }
		


		range_min = (-1.0) * vel_ref * course_width[0] / vel_pd_start;
		range_max = vel_ref * course_width[0] / vel_pd_start;


		Q_pena_ped = 0;

		

		
		closs_range = 0;

#ifdef WHILE
		closs_range = 2.0 * range_max *( loop_num-1) / (attempt_num - 1)+range_min;
#endif //WHILE
		
		cross_pd = x_pd_start - vel_ref * (y_pd_start / vel_pd_start);
		if (u_start > cross_pd - closs_range)
		{
			y_pd = y_pd_start - vel_pd_start * (u_start + closs_range - cross_pd) / vel_ref;
		}


		closs_y_pd = -1*vel_pd_start * closs_range / vel_ref;//x_car=x_pdのときの歩行者のy座標
		
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