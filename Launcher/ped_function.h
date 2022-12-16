#pragma once
#include "Pedestrian.h"
#include <Data/Data.h>
#include <random.h>

class ped_func
{
private:

public:

	void UpdatePed(Pedestrian& ped, double T_delta, double vel_ref, SharedData* shareddata);
	void UpdatePed_judge(Pedestrian& ped, double T_delta, double vel_ref, SharedData* shareddata);
	void UpdatePed_run_out(Pedestrian& ped, double T_delta, double vel_ref, SharedData* shareddata);


};

inline void ped_func::UpdatePed(Pedestrian& ped, double T_delta, double vel_ref, SharedData* shareddata) {
	double x_car, y_car;
	x_car = shareddata->x[0];
	y_car = shareddata->y[0];
	double closs_pd = vel_ref * ((ped.x_pd_start / vel_ref) - (ped.y_pd_start / ped.vel_pd));



	ped.x_pd = ped.x_pd;

	if (x_car + ped.closs_range >= closs_pd) {
		ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;
	}
	else {
		ped.y_pd = ped.y_pd;
	}


	//データの受け渡し
	shareddata->x_pd = ped.x_pd;
	shareddata->y_pd = ped.y_pd;
	shareddata->vel_pd = ped.vel_pd;
	shareddata->closs_pd = closs_pd;
}

inline void ped_func::UpdatePed_judge(Pedestrian& ped, double T_delta, double vel_ref, SharedData* shareddata) {
	double x_car, y_car;
	x_car = shareddata->x[0];
	y_car = shareddata->y[0];
	double closs_pd = ped.x_pd_start - vel_ref * (ped.y_pd_start / ped.vel_pd_start);
	int trigger = shareddata->trigger;
	ped.x_pd = ped.x_pd;
	if (trigger == 0)
	{
		if (x_car + ped.closs_range >= closs_pd) { //歩行者の動き出し
			ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;
			trigger = 1;
		}
		else {
			ped.y_pd = ped.y_pd;
		}
	}
	else if (trigger == 1)
	{
		if (ped.y_pd >= 1.7)//道路わきに来た
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
			ped.vel_pd = 1.5 * ped.vel_pd_start;
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

	shareddata->closs_y_pd = ped.closs_y_pd;
	shareddata->x_pd = ped.x_pd;
	shareddata->y_pd = ped.y_pd;
	shareddata->vel_pd = ped.vel_pd;
	shareddata->closs_pd = closs_pd;
	shareddata->trigger = trigger;
}

//歩行者の飛び出し
//センサーなどの認識を考慮(Lidarを例にして20メートルとした)
inline void ped_func::UpdatePed_run_out(Pedestrian& ped, double T_delta, double vel_ref, SharedData* shareddata)
{
	random_num rand_num;
	double x_car, y_car;
	x_car = shareddata->x[0];
	y_car = shareddata->y[0];
	double closs_pd = ped.x_pd_start - vel_ref * (ped.y_pd_start / ped.vel_pd_start);
	double turning_point = (rand_num.Make_num() % 151 - 75.0) / 100.0;

	int action_num = ped.action_num;// 0:normal // 1:slow->fast // 2:fast->slow // 3:normal->stop // 4:normal->back 

	double dist_sensor = pow(pow(x_car - ped.x_pd, 2) + pow(y_car - ped.y_pd, 2), 0.5);

	ped.x_pd = ped.x_pd;

	double sensor = 30.0;





	if (action_num == 0) {// 0:normal
		if (x_car + ped.closs_range >= closs_pd) { //歩行者の動き出し
			ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;

			if (ped.y_pd <= (-1.0) * (ped.course_width[0] + 0.5)) {
				ped.vel_pd = 0.0;


			}
		}
		else {
			ped.y_pd = ped.y_pd_start;

		}
	}

	else if (action_num == 1) {// 1:slow->fast
		if (x_car + ped.closs_range >= closs_pd) { //歩行者の動き出し
			ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;

			if (dist_sensor <= 5.0) {
				ped.vel_pd = 1.3 * ped.vel_pd_start;

			}
			if (ped.y_pd <= (-1.0) * (ped.course_width[0] + 0.5)) {
				ped.vel_pd = 0.0;

			}
		}
		else {
			ped.y_pd = ped.y_pd_start;

		}
	}

	else if (action_num == 2) {// 2:fast->slow
		if (x_car + ped.closs_range >= closs_pd) { //歩行者の動き出し
			ped.y_pd = ped.y_pd - 1.3 * ped.vel_pd * T_delta;

			if (dist_sensor <= 5.0) {
				ped.vel_pd = ped.vel_pd_start / 1.3;

			}
			if (ped.y_pd <= (-1.0) * (ped.course_width[0] + 0.5)) {
				ped.vel_pd = 0.0;

			}
		}
		else {
			ped.y_pd = ped.y_pd_start;

		}
	}
	else if (action_num == 3) {// 3:normal->stop
		if (x_car + ped.closs_range >= closs_pd) { //歩行者の動き出し
			ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;

			if (ped.y_pd <= turning_point) {
				ped.vel_pd = 0.0;


			}
		}
		else {
			ped.y_pd = ped.y_pd_start;

		}

	}

	else if (action_num == 4) {// 3:normal->back
		if (x_car + ped.closs_range >= closs_pd) { //歩行者の動き出し
			ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;

			if (ped.y_pd <= turning_point) {
				ped.vel_pd = -1 * ped.vel_pd_start;


			}
		}
		else {
			ped.y_pd = ped.y_pd_start;

		}

	}
	//センサーによる認識を再現
	if (dist_sensor <= sensor) {
		ped.x_pd_mpc = ped.x_pd;
		ped.y_pd_mpc = ped.y_pd;
		ped.vel_pd_mpc = ped.vel_pd;

	}
	else {
		ped.x_pd_mpc = ped.x_pd_mpc;
		ped.y_pd_mpc = ped.y_pd_mpc;
		ped.vel_pd_mpc = ped.vel_pd_mpc;
	}

	shareddata->x_pd = ped.x_pd;
	shareddata->y_pd = ped.y_pd;
	shareddata->vel_pd = ped.vel_pd;
	shareddata->closs_pd = closs_pd;
	shareddata->x_pd_mpc = ped.x_pd_mpc;
	shareddata->y_pd_mpc = ped.y_pd_mpc;
	shareddata->vel_pd_mpc = ped.vel_pd_mpc;
	shareddata->closs_y_pd = ped.closs_y_pd;

}

//衝突判定の関数