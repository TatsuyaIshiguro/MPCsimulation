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
	void collision_judge(Pedestrian& ped, SharedData* shareddata);
	void ped_prediction(Pedestrian& ped, double T_delta, SharedData* shareddata);
	std::vector<double> down_vel(Pedestrian& ped, double T_delta, SharedData* shareddata);
	void preserve_init_weight(SharedData* shareddata);
	void TTC(Pedestrian& ped, SharedData* shareddata);

};

inline void ped_func::UpdatePed(Pedestrian& ped, double T_delta, double vel_ref, SharedData* shareddata) {
	double x_car, y_car;
	x_car = shareddata->x[0];
	y_car = shareddata->y[0];
	double closs_pd = vel_ref * ((ped.x_pd_start / vel_ref) - (ped.y_pd_start / ped.vel_pd));



	double dist_sensor = pow(pow(x_car - ped.x_pd, 2) + pow(y_car - ped.y_pd, 2), 0.5);

	ped.x_pd = ped.x_pd;

	double sensor = 20.0;

	if (x_car + ped.closs_range >= closs_pd) {
		ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;
	}
	else {
		ped.y_pd = ped.y_pd;
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
	double turning_point = 0.5;


	int action_num = shareddata->action_num;// 0:normal // 1:slow->fast // 2:fast->slow // 3:normal->stop // 4:normal->back 

	double dist_sensor = pow(pow(x_car - ped.x_pd, 2) + pow(y_car - ped.y_pd, 2), 0.5);

	ped.x_pd = ped.x_pd;

	double sensor = 25.0;

	





	if (action_num == 0) {// 0:normal
		if (x_car + ped.closs_range >= closs_pd) { //歩行者の動き出し
			ped.y_pd = ped.y_pd - ped.vel_pd * T_delta;

			if (ped.y_pd <= (-1.0) * (ped.course_width[0] + 1.5)) {
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
				shareddata->count_stop_num++;
				if (shareddata->count_stop_num > 33)
				{
					ped.vel_pd = ped.vel_pd_start;
				}


			}
		}
		else {
			ped.y_pd = ped.y_pd_start;

		}

	}

	else if (action_num == 4) {// 4:normal->back
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

#ifdef Penalty_vel
		shareddata->Q_pena_vel = 5.0;
		shareddata->Q_vel = shareddata->Init_Q_vel / 1000;
		shareddata->Sf_vel = shareddata->Init_Sf_vel / 1000;
		//shareddata->Q_v = shareddata->Init_Q_v / 1.25;
		//shareddata->Sf_v = shareddata->Init_Sf_v / 1.25;

		if (ped.y_pd <= y_car - 0.55 && ped.x_pd <= x_car)
		{
			shareddata->Q_pena_vel = 0;
			shareddata->Q_vel = shareddata->Init_Q_vel;
			shareddata->Sf_vel = shareddata->Init_Sf_vel;
			//shareddata->Q_v = shareddata->Init_Q_v ;
			//shareddata->Sf_v = shareddata->Init_Sf_v ;
		}
		if (ped.x_pd <= x_car - 10)
		{
			shareddata->Q_pena_vel = 0;
			shareddata->Q_vel = shareddata->Init_Q_vel;
			shareddata->Sf_vel = shareddata->Init_Sf_vel;
			//shareddata->Q_v = shareddata->Init_Q_v ;
			//shareddata->Sf_v = shareddata->Init_Sf_v ;
		}	
#endif //Penalty_vel

#ifdef Penalty_dist
		shareddata->Q_pena_dist = 2.0;
		shareddata->Q_v = shareddata->Init_Q_v / 1.25;//あまり小さくしすぎないほうが良いかも
		shareddata->Sf_v = shareddata->Init_Sf_v/ 1.25;

		if (ped.y_pd <= y_car - 0.55 && ped.x_pd <= x_car)
		{
			shareddata->Q_pena_dist = 0;
			shareddata->Q_v = shareddata->Init_Q_v;
			shareddata->Sf_v = shareddata->Init_Sf_v;
		}
		if (ped.x_pd <= x_car - 10)
		{
			shareddata->Q_pena_dist = 0;
			shareddata->Q_v = shareddata->Init_Q_v;
			shareddata->Sf_v = shareddata->Init_Sf_v;
		}
#endif //Penalty_dist



		

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
inline void ped_func::collision_judge(Pedestrian& ped, SharedData* shareddata)
{
	double dist_g, dist_f_l, dist_f_r, dist_r_l, dist_r_r;
	double x_car, y_car;
	double x_pd, y_pd;
	double u_front_r, u_front_l, u_rear_r, u_rear_l;
	double v_front_r, v_front_l, v_rear_r, v_rear_l;
	int collision_judge;

	//直線道路においてのみ、x:u,y:vで対応している

	x_car = shareddata->x[0];
	y_car = shareddata->y[0];

	u_front_l = shareddata->u_front_l[0];
	u_front_r = shareddata->u_front_r[0];
	u_rear_l = shareddata->u_rear_l[0];
	u_rear_r = shareddata->u_rear_r[0];

	v_front_l = shareddata->v_front_l[0];
	v_front_r = shareddata->v_front_r[0];
	v_rear_l = shareddata->v_rear_l[0];
	v_rear_r = shareddata->v_rear_r[0];


	x_pd = ped.x_pd;
	y_pd = ped.y_pd;

	dist_g = pow(pow(x_car - x_pd, 2) + pow(x_car - y_pd, 2), 0.5);
	dist_f_l = pow(pow(u_front_l - x_pd, 2) + pow(v_front_l - y_pd, 2), 0.5);
	dist_f_r = pow(pow(u_front_r - x_pd, 2) + pow(v_front_r - y_pd, 2), 0.5);
	dist_r_l = pow(pow(u_rear_l - x_pd, 2) + pow(v_rear_l - y_pd, 2), 0.5);
	dist_r_r = pow(pow(u_rear_r - x_pd, 2) + pow(v_rear_r - y_pd, 2), 0.5);


	if (dist_g >= 1.0 && dist_f_l >= 0.55 && dist_f_r >= 0.55 && dist_r_l >= 0.55 && dist_r_r >= 0.55)
	{
		collision_judge = 0;
	}
	else
	{
		collision_judge = 1;
	}

	shareddata->collision_num = collision_judge;

}


//歩行者の予測
inline void ped_func::ped_prediction(Pedestrian& ped, double T_delta, SharedData* shareddata)
{
	

	shareddata->x_pd_pre[0] = ped.x_pd_mpc;
	shareddata->y_pd_pre[0] = ped.y_pd_mpc;



	for (int i = 0; i < vsize-1; i++)
	{
		shareddata->x_pd_pre[i + 1] = shareddata->x_pd_pre[i];
		shareddata->y_pd_pre[i + 1] = shareddata->y_pd_pre[i] - ped.vel_pd_mpc * T_delta;
	}


	for (int i = 0; i < vsize; i++)
	{
		shareddata->dist_pd[i] = pow(pow(shareddata->x[i] - shareddata->x_pd_pre[i], 2) + pow(shareddata->y[i] - shareddata->y_pd_pre[i], 2), 0.5);
	}
}


//歩行者に近づいた時の減速
inline vector<double> ped_func::down_vel(Pedestrian& ped, double T_delta, SharedData* shareddata)
{
	double now_vel_ref = shareddata->vel_ref_pre[1];
	double Init_vel_ref = shareddata->vel_ref;
	double x_car, y_car;
	x_car = shareddata->x[0];
	y_car = shareddata->y[0];
	double x_pd = ped.x_pd;
	double y_pd = ped.y_pd;
	double dist_g = pow(pow(x_car - x_pd, 2) + pow(x_car - y_pd, 2), 0.5);
	double acc_down = -0.75;
	double sensor = 25.0;
	double vel_ref_min = 0.1;
	std::vector<double> vel_ref_pre;

	vel_ref_pre.resize(vsize);
	for (int i = 0; i < vsize; i++)
	{
		vel_ref_pre[i] = now_vel_ref;
	}
	shareddata->dist_g = dist_g;

#ifdef vel_ref_down
	if (dist_g <= sensor)
	{
		for (int i = 0; i < vsize-1; i++)
		{
			vel_ref_pre[i + 1] = vel_ref_pre[i] + acc_down * T_delta;
			if (vel_ref_pre[i + 1] <= vel_ref_min) {
				vel_ref_pre[i + 1] = vel_ref_min;
			}
		}
	}

	if (x_car  >= x_pd) {
		for (int i = 0; i < vsize; i++)
		{
			vel_ref_pre[i] = Init_vel_ref;
		}
	}

	if (y_car-0.55 >= y_pd) {
		for (int i = 0; i < vsize; i++)
		{
			vel_ref_pre[i] = Init_vel_ref;
		}
	}
#endif //vel_ref_down
	return vel_ref_pre;
}

inline void ped_func::preserve_init_weight(SharedData* shareddata)
{
	shareddata->Init_Q_v = shareddata->Q_v;
	shareddata->Init_Q_vel = shareddata->Q_vel;
	shareddata->Init_Sf_v= shareddata->Sf_v;
	shareddata->Init_Sf_vel = shareddata->Sf_vel;
		
}

inline void ped_func::TTC(Pedestrian& ped, SharedData* shareddata)
{
	double dist_pd_c, dist_car_c;
	double x_car, y_car, theta,vel_car;
	double x_c, y_c;//closs_pointの座標
	double TTC_pd, TTC_car;
	x_car= shareddata->x[0];
	y_car = shareddata->y[0];
	theta = shareddata->theta[0];
	vel_car = shareddata->vel[0];
	x_c = ped.x_pd;
	y_c = y_car + tan(theta) * (ped.x_pd - x_car);
	dist_pd_c = pow(pow(y_c - ped.y_pd, 2), 0.5);
	dist_car_c = pow(pow(x_c - x_car, 2) + pow(y_c - y_car, 2), 0.5) - 1;
	TTC_pd = dist_pd_c / ped.vel_pd_start;
	TTC_car = dist_car_c / vel_car;
	shareddata->TTC_pd = TTC_pd;
	shareddata->TTC_car = TTC_car;





}