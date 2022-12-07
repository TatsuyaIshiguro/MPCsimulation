#pragma once
#include <random.h>

struct Pedestrian
{
	double x_pd, y_pd, vel_pd, closs_pd;
	double x_pd_start, y_pd_start, vel_pd_start;
	double closs_range;

	random_num rand_num;
	
	//‰Šú’l‚ğŒˆ’è
	// 30 <= x_pd <= 60 delta=1
	// 2.5 <= y_pd <= 5.0 delta=0.5
	//0.5 <= vel_pd <= 1.5 delta=0.00001
	//-10.0 <= closs_range <= 10.0 delta=0.1
	Pedestrian() {
		/*x_pd_start= rand_num.Make_num()%31+30.0;
		y_pd_start = (rand_num.Make_num()%6+5.0)*0.5;
		vel_pd_start = ((rand_num.Make_num() % 101 + 50.0) * 1000 + rand_num.Make_num() % 1000)/100000;
		x_pd = x_pd_start;
		y_pd = y_pd_start;
		vel_pd = vel_pd_start;
		closs_range = (rand_num.Make_num() % 201 - 100.0) / 10;*/
		x_pd = 50;
		y_pd = 4.0;
		vel_pd = 0.833333;//  3km/h
		//vel_pd = 1.111111;//  4km/h
		closs_range = 1.5;

	}


};