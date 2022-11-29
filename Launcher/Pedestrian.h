#pragma once

struct Pedestrian
{
	double x_pd, y_pd, vel_pd;
	double x_pd_start=50, y_pd_start=3, vel_pd_start=1.388889;
	double closs_pd = (y_pd_start) * sqrt(pow(6, 2) + pow(vel_pd, 2)) / (vel_pd);//pow(6, 2)‚ðpow(vel_ref,2)‚É‚µ‚½‚¢
	Pedestrian() {
		x_pd = 50;
		y_pd = 3;
		vel_pd = 0.0000000000001;

	}


};