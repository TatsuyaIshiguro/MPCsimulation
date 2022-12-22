#pragma once

const int cprm_num = 11; //コースのパラメータ数
const int csize = 400;
const int vsize = 70; //状態ベクトルのサイズ

struct SharedData
{
	double course[cprm_num][csize]; //コースデータを保存する配列
	int error_code, iters, fevals, method, success; //1,2,3番目->nuoptマニュアル参照、sim_step->MPCを実行するステップ数、method->IPM or SQP、success->プログラム強制終了の場合は0、noise_count->ノイズありで同じ初期状態に対して繰り返す回数
	double T_delta, eps, elapse_time, optValue, tolerance, residual, average_lateral_jerk, average_longitudinal_jerk; //2,3,4,5,6番目->マニュアル参照、average_jerk->乗り心地の指標

	double vel_ref;
	double u[vsize], vel[vsize], acc[vsize], v[vsize], v_dot[vsize], v_2dot[vsize], theta[vsize], theta_dot[vsize], theta_2dot[vsize], delta[vsize], delta_dot[vsize], front_u[vsize], lateral_G[vsize], lateral_jerk[vsize], longitudinal_jerk[vsize];
	double x[vsize], y[vsize], yaw[vsize];
	double x_PD[vsize], y_PD[vsize];//歩行者の状態量
	double dist_pd[vsize];

	double l_f, l_r, width, dist_front, dist_rear, theta_front, theta_rear; //dist_front->フロントオーバーハングを考慮したときの左右の頂点と重心との距離、theta_front, theta_rear->左右頂点と重心間の線分と軸との角度
	double a11, a12, a21, a22, b1, b2; //DBMにおける係数

	//評価関数用の重み
	double Q_vel, Q_acc, Q_v, Q_v_dot, Q_v_2dot, Q_theta, Q_theta_dot, Q_theta_2dot, Q_delta, Q_delta_dot;
	double Sf_vel, Sf_acc, Sf_v, Sf_v_dot, Sf_v_2dot, Sf_theta, Sf_theta_dot, Sf_theta_2dot, Sf_delta, Sf_delta_dot;

	double x_pd, y_pd, vel_pd, closs_pd, closs_y_pd, closs_range, dist_g;
	int trigger, action_num, vel_pd_num, collision_num;
	double x_pd_mpc, y_pd_mpc, vel_pd_mpc, Q_pena_dist, Q_pena_vel;

	double u_front_r[vsize], u_front_l[vsize], u_rear_r[vsize], u_rear_l[vsize];
	double v_front_r[vsize], v_front_l[vsize], v_rear_r[vsize], v_rear_l[vsize];

	double x_pd_pre[vsize], y_pd_pre[vsize];
	double vel_ref_pre[vsize];

};