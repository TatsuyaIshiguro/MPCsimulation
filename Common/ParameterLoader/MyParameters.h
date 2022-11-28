#pragma once
#include <CSVLoader/CSVLoader.h>

class Prm
{
public:
	int Method, rcd_horizon;
	double a11, a12, a21, a22, b1, b2;
	double  l_f, l_r, Wheelbase, width;
	double T_delta, eps;

	//衝突判定用の幾何学的なパラメータ
	double dist_front, dist_rear, theta_front, theta_rear;

	double Q_vel, Q_acc, Q_v, Q_v_dot, Q_v_2dot, Q_theta, Q_theta_dot, Q_theta_2dot, Q_delta, Q_delta_dot;
	double Sf_vel, Sf_acc, Sf_v, Sf_v_dot, Sf_v_2dot, Sf_theta, Sf_theta_dot, Sf_theta_2dot, Sf_delta, Sf_delta_dot;

	void Load_Prm(RTCLib::CSVLoader& prm, int n);

private:
	//車両パラメータ
	double M, Iz, C_f, C_r, overhang_front, overhang_rear;
};


inline void Prm::Load_Prm(RTCLib::CSVLoader& prm, int n) {
	//パラメータ読み込み
	Method = prm[n][prm.GetColOf("0SQPor1IPM")];
	rcd_horizon = prm[n][prm.GetColOf("rcd_horizon")];
	T_delta = prm[n][prm.GetColOf("T_delta")];
	eps = prm[n][prm.GetColOf("eps")];
	M = prm[n][prm.GetColOf("M")];
	Iz = prm[n][prm.GetColOf("Iz")];
	l_f = prm[n][prm.GetColOf("l_f")];
	l_r = prm[n][prm.GetColOf("l_r")];
	C_f = prm[n][prm.GetColOf("C_f")];
	C_r = prm[n][prm.GetColOf("C_r")];
	overhang_front = prm[n][prm.GetColOf("overhang_front")];
	overhang_rear = prm[n][prm.GetColOf("overhang_rear")];
	width = prm[n][prm.GetColOf("width")];
	Q_vel = prm[n][prm.GetColOf("Q_vel")];
	Q_acc = prm[n][prm.GetColOf("Q_acc")];
	Q_v = prm[n][prm.GetColOf("Q_v")];
	Q_v_dot = prm[n][prm.GetColOf("Q_v_dot")];
	Q_v_2dot = prm[n][prm.GetColOf("Q_v_2dot")];
	Q_theta = prm[n][prm.GetColOf("Q_theta")];
	Q_theta_dot = prm[n][prm.GetColOf("Q_theta_dot")];
	Q_theta_2dot = prm[n][prm.GetColOf("Q_theta_2dot")];
	Q_delta = prm[n][prm.GetColOf("Q_delta")];
	Q_delta_dot = prm[n][prm.GetColOf("Q_delta_dot")];
	Sf_vel = prm[n][prm.GetColOf("Sf_vel")];
	Sf_acc = prm[n][prm.GetColOf("Sf_acc")];
	Sf_v = prm[n][prm.GetColOf("Sf_v")];
	Sf_v_dot = prm[n][prm.GetColOf("Sf_v_dot")];
	Sf_v_2dot = prm[n][prm.GetColOf("Sf_v_2dot")];
	Sf_theta = prm[n][prm.GetColOf("Sf_theta")];
	Sf_theta_dot = prm[n][prm.GetColOf("Sf_theta_dot")];
	Sf_theta_2dot = prm[n][prm.GetColOf("Sf_theta_2dot")];
	Sf_delta = prm[n][prm.GetColOf("Sf_delta")];
	Sf_delta_dot = prm[n][prm.GetColOf("Sf_delta_dot")];

	a11 = (C_f + C_r) / M;
	a12 = -(l_f * C_f - l_r * C_r) / M;
	a21 = (l_f * C_f - l_r * C_r) / Iz;
	a22 = -(l_f * l_f * C_f + l_r * l_r * C_r) / Iz;
	b1 = C_f / M;
	b2 = l_f * C_f / Iz;
	Wheelbase = l_f + l_r;
	dist_front = sqrt((l_f + overhang_front) * (l_f + overhang_front) + (width / 2) * (width / 2));
	dist_rear = sqrt((l_r + overhang_rear) * (l_r + overhang_rear) + (width / 2) * (width / 2));
	theta_front = atan((width / 2) / (l_f + overhang_front));
	theta_rear = atan((width / 2) / (l_r + overhang_rear));
}