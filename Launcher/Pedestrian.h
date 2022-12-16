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
	

	random_num rand_num;


	//‰Šú’l‚ğŒˆ’è
	// 30 <= x_pd <= 60 delta=1
	// y_pd=2.0 ‚ÅŒÅ’è
	//0.5 <= vel_pd <= 1.5 delta=0.00001
	//range_min <= closs_range <= range_max

	Pedestrian(double vel_ref, std::vector<double> course_width, SharedData* shareddata) {

		this->vel_ref = vel_ref;
		this->course_width = course_width;
		x_pd_start = rand_num.Make_num() %31 + 30.0;
		y_pd_start = 2.0;
		//vel_pd_start = ((rand_num.Make_num() % 101 + 50.0) * 1000 + rand_num.Make_num() % 1000)/100000;
		int select_vel = rand_num.Make_num() % 4;
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

		action_num = rand_num.Make_num() % 5;// 0:normal // 1:slow->fast // 2:fast->slow // 3:normal->stop // 4:normal->back 

		range_min = (-1.0) * vel_ref * course_width[0] / vel_pd_start;
		range_max = vel_ref * course_width[0] / vel_pd_start;

		x_pd_mpc = 500;//–³ŒÀ‰“‚É‚¢‚é‚Æ‚·‚é
		y_pd_mpc = 500;//–³ŒÀ‰“‚É‚¢‚é‚Æ‚·‚é
		vel_pd_mpc = 0;

		x_pd = x_pd_start;
		y_pd = y_pd_start;
		vel_pd = vel_pd_start;
		

		closs_range = (rand_num.Make_num() % 32767 - 16383.0) / 16383.0 * range_max;
		closs_y_pd = vel_pd_start * closs_range / vel_ref;//x_car=x_pd‚Ì‚Æ‚«‚Ì•àsÒ‚ÌyÀ•W
		
		shareddata->closs_range = closs_range;
		shareddata->closs_y_pd = closs_y_pd;
		shareddata->action_num = action_num;
		shareddata->vel_pd_num = select_vel;
		shareddata->x_pd_mpc = x_pd_mpc;
		shareddata->y_pd_mpc = y_pd_mpc;
		shareddata->vel_pd_mpc = vel_pd_mpc;


	}


};