#pragma once

#include <header/GetConstraint.h>
#include <header/NoiseMake.h>

class MyProblem {
public:
	// main関数でも利用するデータ
	int rcd_horizon, error_code, iters, fevals, method; //2,3,4番目 -> nuoptのマニュアル参照、method -> IPM or SQP
	double T_delta, eps, elapse_time, optValue, tolerance, residual; //nuoptのマニュアル参照
	std::vector<double> v_ref, v_max, v_min, vel_ref, vel_max, Rho, v_front_max, v_front_min; //u, v, theta -> frenet coordinate
	std::vector<double> u, v, v_dot, v_2dot, theta, theta_dot, theta_2dot, delta, delta_dot, vel, acc, u_center_l, u_center_r, lateral_G, lateral_jerk, longitudinal_jerk;
	std::vector<double> x, y, yaw; // x, y, yaw -> global coordinate
	//
	std::vector<double> v_front_l, v_front_r, v_rear_l, v_rear_r;
	std::vector<double> u_front_l, u_front_r, u_rear_l, u_rear_r;
	//
	double M, Iz, l_f, l_r, C_f, C_r, width, dist_front, dist_rear, theta_front, theta_rear; // 車両モデルのパラメータ
	double a11, a12, a21, a22, b1, b2;

	//歩行者
	double x_pd_mpc, y_pd_mpc, vel_pd_mpc, Q_pena_dist, Q_pena_vel;
	double Q_vel, Sf_vel, Q_v, Sf_v;
	
	std::vector<double> x_PD, y_PD;


	//jerk用
	double B_y_2dot, average_lateral_jerk, max_jerk, average_longitudinal_jerk;

	Noise noise;

	// 関数の定義
	MyProblem(SharedData* shareddata);
	~MyProblem();
	void Solve();

	// 初期化関係
	void InitOptVec();
	void InitState(SharedData* shareddata);
	void SetFront_u();
	void UpdateState(); //状態更新用

	// パラメータのセット
	void SetV(double current_v[70]);
	void SetAllState();

	//制約のセット
	void SetConstraints(std::vector<double> v_max, std::vector<double> v_min, std::vector<double> v_ref, std::vector<double> vel_max, std::vector<double> rho, std::vector<double> v_front_max, std::vector<double> v_front_min, std::vector<double> v_rear_max, std::vector<double> v_rear_min);
	void SetYmax(std::vector<double> v_max);
	void SetYmin(std::vector<double> v_min);
	void SetYref(std::vector<double> v_ref);
	void SetVref(double vel_ref_pre[70]);
	void SetVmax(std::vector<double> vel_max);
	void SetRho(std::vector<double> Rho);
	void SetYmax_front(std::vector<double> v_front_max);
	void SetYmin_front(std::vector<double> v_front_min);
	void SetYmax_rear(std::vector<double> v_rear_max);
	void SetYmin_rear(std::vector<double> v_rear_min);

private:
	std::shared_ptr<void> model;
};