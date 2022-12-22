#pragma once

const int cprm_num = 11; //�R�[�X�̃p�����[�^��
const int csize = 400;
const int vsize = 70; //��ԃx�N�g���̃T�C�Y

struct SharedData
{
	double course[cprm_num][csize]; //�R�[�X�f�[�^��ۑ�����z��
	int error_code, iters, fevals, method, success; //1,2,3�Ԗ�->nuopt�}�j���A���Q�ƁAsim_step->MPC�����s����X�e�b�v���Amethod->IPM or SQP�Asuccess->�v���O���������I���̏ꍇ��0�Anoise_count->�m�C�Y����œ���������Ԃɑ΂��ČJ��Ԃ���
	double T_delta, eps, elapse_time, optValue, tolerance, residual, average_lateral_jerk, average_longitudinal_jerk; //2,3,4,5,6�Ԗ�->�}�j���A���Q�ƁAaverage_jerk->���S�n�̎w�W

	double vel_ref;
	double u[vsize], vel[vsize], acc[vsize], v[vsize], v_dot[vsize], v_2dot[vsize], theta[vsize], theta_dot[vsize], theta_2dot[vsize], delta[vsize], delta_dot[vsize], front_u[vsize], lateral_G[vsize], lateral_jerk[vsize], longitudinal_jerk[vsize];
	double x[vsize], y[vsize], yaw[vsize];
	double x_PD[vsize], y_PD[vsize];//���s�҂̏�ԗ�
	double dist_pd[vsize];

	double l_f, l_r, width, dist_front, dist_rear, theta_front, theta_rear; //dist_front->�t�����g�I�[�o�[�n���O���l�������Ƃ��̍��E�̒��_�Əd�S�Ƃ̋����Atheta_front, theta_rear->���E���_�Əd�S�Ԃ̐����Ǝ��Ƃ̊p�x
	double a11, a12, a21, a22, b1, b2; //DBM�ɂ�����W��

	//�]���֐��p�̏d��
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