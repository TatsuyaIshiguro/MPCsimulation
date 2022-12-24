#pragma once
#include "Pedestrian.h"
#include <Data/Data.h>

class update_vel_ref
{
private:


public:
	std::vector<double> down_vel( SharedData* shareddata);
};

//参照速度の更新
inline std::vector<double> update_vel_ref::down_vel( SharedData* shareddata)
{
	double now_vel_ref = shareddata->vel_ref_pre[1];
	double Init_vel_ref = shareddata->vel_ref;
	std::vector<double> vel_ref_pre;

	vel_ref_pre.resize(vsize);
	for (int i = 0; i < vsize; i++)
	{
		vel_ref_pre[i] = now_vel_ref;
	}

	return vel_ref_pre;
}