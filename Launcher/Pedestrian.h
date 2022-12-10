#pragma once
#include <random.h>

struct Pedestrian
{
	
	double x_pd, y_pd, vel_pd, closs_pd;
	double x_pd_start, y_pd_start, vel_pd_start;
	double closs_range;
	double vel_ref;
	vector<double>  course_width ;
	double range_min, range_max;
	double closs_y_pd;



	random_num rand_num;
	
	
	//‰Šú’l‚ğŒˆ’è
	// 25 <= x_pd <= 75 delta=1
	// y_pd=2.0 ‚ÅŒÅ’è
	//0.5 <= vel_pd <= 1.5 delta=0.00001
	//range_min <= closs_range <= range_max

	Pedestrian(double vel_ref,std::vector<double> course_width) {
		
		this->vel_ref =vel_ref;
		this->course_width = course_width;
		x_pd_start= rand_num.Make_num()%51+25.0;
		y_pd_start = 2.0;
		vel_pd_start = ((rand_num.Make_num() % 101 + 50.0) * 1000 + rand_num.Make_num() % 1000)/100000;

		range_min = (-1.0) * vel_ref * course_width[0] / vel_pd_start;
		range_max = vel_ref * course_width[0]/ vel_pd_start;

		x_pd = x_pd_start;
		y_pd = y_pd_start;
		vel_pd = vel_pd_start;

		closs_range = rand_num.num_range(range_min, range_max);
		closs_y_pd = vel_pd * closs_range / vel_ref;
		
	
	}


};