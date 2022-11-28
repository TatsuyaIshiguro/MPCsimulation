#pragma once

#include <vector>
#include <header/TableLinearInterporater.h>

class getconstraint {
public:
	std::vector<double> v_max, v_ref, v_min, vel_max, vel_ref, rho, v_front_min, v_front_max, v_rear_max, v_rear_min; //u, v -> frenet coordinateAvel -> velocity

	getconstraint(int vsize);
	void InitCourse(LinearInterporater table);
	void set_vector(LinearInterporater table, std::vector<double>& vec_x, std::vector<double>& vec_const, int row);
	void GetConstraint(std::vector<double> u_future, std::vector<double> u_front_l, std::vector<double> u_front_r, std::vector<double> u_center_l, std::vector<double> u_center_r, std::vector<double> u_rear_l, std::vector<double> u_rear_r);

private:
	LinearInterporater table;
};

inline getconstraint::getconstraint(int vsize)
{
	v_max.resize(vsize);
	v_min.resize(vsize);
	v_ref.resize(vsize);
	vel_max.resize(vsize);
	vel_ref.resize(vsize);
	rho.resize(vsize);
	v_front_max.resize(vsize);
	v_front_min.resize(vsize);
	v_rear_max.resize(vsize);
	v_rear_min.resize(vsize);
}

inline void getconstraint::InitCourse(LinearInterporater table)
{
	this->table = table;
}

inline void getconstraint::set_vector(LinearInterporater table, std::vector<double>& vec_x, std::vector<double>& vec_const, int row)
{
	for (int i = 0; i < vec_x.size(); ++i) {
		vec_const.at(i) = table.GetLinearInterporation(0, vec_x.at(i), row);
	}
}

inline void getconstraint::GetConstraint(std::vector<double> u_future, std::vector<double> u_front_l, std::vector<double> u_front_r, std::vector<double> u_center_l, std::vector<double> u_center_r, std::vector<double> u_rear_l, std::vector<double> u_rear_r)
{
	set_vector(table, u_center_l, v_max, 1);
	set_vector(table, u_center_r, v_min, 2);
	set_vector(table, u_future, v_ref, 3);
	set_vector(table, u_future, rho, 4);
	set_vector(table, u_front_l, v_front_max, 1);
	set_vector(table, u_front_r, v_front_min, 2);
	set_vector(table, u_rear_l, v_rear_max, 1);
	set_vector(table, u_rear_r, v_rear_min, 2);
}