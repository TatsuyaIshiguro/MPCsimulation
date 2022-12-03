// -*- Mode: C++ -*-
// auto generate
#ifdef __GNUC__
#include <string.h>
#endif
#include "simple.h"
#include "SimpleObjProxy.h"
#include "SystemInterface.h"
#include "nuopt_exception.h"
#include "throw_exception.h"
#include <Data/Data.h>

extern void solout();
extern void clearSolout();
extern void nuopt_setBasein(void* basein_);
extern void nuopt_setBaseout(void** baseout_);
extern const char* nuopt_restartDescChar();
static void __dummy() {}
#ifdef NO_HEADER
class System_NUOPT : public SystemInterface {
public:
	System_NUOPT(std::string path);
	ParameterProxy rcd_horizon;
	ParameterProxy T_delta;
	ParameterProxy eps;
	SetProxy tempSet;
	ElementProxy i;
	ParameterProxy a;
	SetProxy Step_cn;
	ElementProxy Idx;
	ParameterProxy b;
	SetProxy Step_eval;
	ElementProxy Idx_eval;
	ParameterProxy a11;
	ParameterProxy a12;
	ParameterProxy a21;
	ParameterProxy a22;
	ParameterProxy b1;
	ParameterProxy b2;
	ParameterProxy Q_vel;
	ParameterProxy Q_acc;
	ParameterProxy Q_v;
	ParameterProxy Q_v_dot;
	ParameterProxy Q_v_2dot;
	ParameterProxy Q_theta;
	ParameterProxy Q_theta_dot;
	ParameterProxy Q_theta_2dot;
	ParameterProxy Q_delta;
	ParameterProxy Q_delta_dot;
	ParameterProxy Sf_vel;
	ParameterProxy Sf_acc;
	ParameterProxy Sf_v;
	ParameterProxy Sf_v_dot;
	ParameterProxy Sf_v_2dot;
	ParameterProxy Sf_theta;
	ParameterProxy Sf_theta_dot;
	ParameterProxy Sf_theta_2dot;
	ParameterProxy Sf_delta;
	ParameterProxy Sf_delta_dot;
	ParameterProxy M;
	ParameterProxy Iz;
	ParameterProxy C_f;
	ParameterProxy C_r;
	ParameterProxy l_f;
	ParameterProxy l_r;
	ParameterProxy width;
	ParameterProxy dist_front;
	ParameterProxy dist_rear;
	ParameterProxy theta_front;
	ParameterProxy theta_rear;
	VariableProxy u;
	VariableProxy vel;
	VariableProxy acc;
	VariableProxy v;
	VariableProxy v_dot;
	VariableProxy v_2dot;
	VariableProxy theta;
	VariableProxy theta_dot;
	VariableProxy theta_2dot;
	VariableProxy delta;
	VariableProxy delta_dot;
	VariableProxy v_front_l; //衝突判定用の状態
	VariableProxy v_front_r;
	VariableProxy v_center_l;
	VariableProxy v_center_r;
	VariableProxy V_inv;
	VariableParameterProxy init_u;
	VariableParameterProxy init_vel;
	VariableParameterProxy acc_init;
	VariableParameterProxy init_v;
	VariableParameterProxy init_v_dot;
	VariableParameterProxy y_2dot_init;
	VariableParameterProxy init_theta;
	VariableParameterProxy init_theta_dot;
	VariableParameterProxy theta_2dot_init;
	VariableParameterProxy init_delta;
	VariableParameterProxy delta_dot_init;
	VariableParameterProxy vel_ref;
	VariableParameterProxy vel_max;
	VariableParameterProxy v_ref;
	VariableParameterProxy v_max;
	VariableParameterProxy v_min;
	VariableParameterProxy Rho;
	VariableParameterProxy v_front_max; //衝突判定用の制約
	VariableParameterProxy v_front_min;
	ObjectiveProxy obj;
};
#else /* ! NO_HEADER */
#include "NUOPT.h"
#endif /* NO_HEADER */

//====================
//====================

// macros
#define index indsimple
// if you use Interval, please define USE_SIMPLE_INTERVAL macro
#ifdef USE_SIMPLE_INTERVAL
#define left leftsimple
#define right rightsimple
#endif

System_NUOPT::System_NUOPT()
	: SystemInterface("NUOPT")
	, rcd_horizon(this), tempSet(this), tempElem(this), a(this), b(this), Step_cn(this), Idx(this), Step_eval(this), Idx_eval(this), eps(this), u(this), vel(this), acc(this), v(this), v_dot(this), v_2dot(this), theta(this), theta_dot(this), theta_2dot(this), delta(this), delta_dot(this), v_front_l(this), v_front_r(this), v_center_l(this), v_center_r(this), v_rear_l(this), v_rear_r(this), V_inv(this), T_delta(this), a11(this), a12(this), a21(this), a22(this), b1(this), b2(this), Q_vel(this), Q_acc(this), Q_v(this), Q_v_dot(this), Q_v_2dot(this), Q_theta(this), Q_theta_dot(this), Q_theta_2dot(this), Q_delta(this), Q_delta_dot(this), Sf_vel(this), Sf_acc(this), Sf_v(this), Sf_v_dot(this), Sf_v_2dot(this), Sf_theta(this), Sf_theta_dot(this), Sf_theta_2dot(this), Sf_delta(this), Sf_delta_dot(this), width(this), dist_front(this), dist_rear(this), theta_front(this), theta_rear(this), init_u(this), init_vel(this), acc_init(this), init_v(this), init_v_dot(this), y_2dot_init(this), init_theta(this), init_theta_dot(this), theta_2dot_init(this), init_delta(this), delta_dot_init(this), vel_ref(this), vel_max(this), v_ref(this), v_max(this), v_min(this), v_front_max(this), v_front_min(this), v_rear_max(this), v_rear_min(this), Rho(this), obj(this),x_pd(this),y_pd(this),closs_pd(this),vel_pd(this), Dist(this),x_PD(this),y_PD(this)
{
	{
		options.outfilename = "NUOPT";
#line 1 "C:\\Users\\Yusuke\\AppData\\Roaming\\Mathematical Systems Inc\\NUOPT\\mknuopt\\mknuopt_4568\\NUOPT_before_preprocess.cc"
#line 1 "NUOPT.smp"

		smp_line(__LINE__, __FILE__); Parameter rcd_horizon(name = "rcd_horizon"); this->rcd_horizon.setEntity(rcd_horizon); rcd_horizon.entryOutput();
		smp_line(__LINE__, __FILE__); rcd_horizon = vsize - 1;
		smp_line(__LINE__, __FILE__); Set tempSet(name = "tempSet"); this->tempSet.setEntity(tempSet); tempSet.entryOutput(); tempSet = "0 .. 100";
		smp_line(__LINE__, __FILE__); Element tempElem(set = tempSet, name = "tempElem");	this->tempElem.setEntity(tempElem); tempElem.entryOutput();
		smp_line(__LINE__, __FILE__); Parameter a(index = tempElem, name = "a"); this->a.setEntity(a); a.entryOutput();
		smp_line(__LINE__, __FILE__); Parameter b(index = tempElem, name = "b"); this->b.setEntity(b); b.entryOutput();
		smp_line(__LINE__, __FILE__); for (int i = 0; i < rcd_horizon + 1; i++)
		{
			smp_line(__LINE__, __FILE__); a[i] = 1;
		}
		smp_line(__LINE__, __FILE__); for (int i = 0; i < rcd_horizon; i++)
		{
			smp_line(__LINE__, __FILE__); b[i] = 1;
		}
		smp_line(__LINE__, __FILE__); Set Step_cn(name = "Step_cn"); this->Step_cn.setEntity(Step_cn); Step_cn.entryOutput(); Step_cn = setOf(tempElem, a[tempElem] > 0);
		smp_line(__LINE__, __FILE__); Element Idx(set = Step_cn, name = "Idx"); this->Idx.setEntity(Idx); Idx.entryOutput();
		smp_line(__LINE__, __FILE__); Set Step_eval(name = "Step_eval"); this->Step_eval.setEntity(Step_eval); Step_eval.entryOutput(); Step_eval = setOf(tempElem, b[tempElem] > 0);
		smp_line(__LINE__, __FILE__); Element Idx_eval(set = Step_eval, name = "Idx_eval"); this->Idx_eval.setEntity(Idx_eval); Idx_eval.entryOutput();
		
		smp_line(__LINE__, __FILE__); Parameter eps(name = "eps"); this->eps.setEntity(eps); eps.entryOutput();

		smp_line(__LINE__, __FILE__); Variable u(index = Idx, name = "u"); this->u.setEntity(u); u.entryOutput();
		smp_line(__LINE__, __FILE__); Variable vel(index = Idx, name = "vel"); this->vel.setEntity(vel); vel.entryOutput();
		smp_line(__LINE__, __FILE__); Variable acc(index = Idx, name = "acc"); this->acc.setEntity(acc); acc.entryOutput();
		smp_line(__LINE__, __FILE__); Variable v(index = Idx, name = "v"); this->v.setEntity(v); v.entryOutput();
		smp_line(__LINE__, __FILE__); Variable v_dot(index = Idx, name = "v_dot"); this->v_dot.setEntity(v_dot); v_dot.entryOutput();
		smp_line(__LINE__, __FILE__); Variable v_2dot(index = Idx, name = "v_2dot"); this->v_2dot.setEntity(v_2dot); v_2dot.entryOutput();
		smp_line(__LINE__, __FILE__); Variable theta(index = Idx, name = "theta"); this->theta.setEntity(theta); theta.entryOutput();
		smp_line(__LINE__, __FILE__); Variable theta_dot(index = Idx, name = "theta_dot"); this->theta_dot.setEntity(theta_dot); theta_dot.entryOutput();
		smp_line(__LINE__, __FILE__); Variable theta_2dot(index = Idx, name = "theta_2dot"); this->theta_2dot.setEntity(theta_2dot); theta_2dot.entryOutput();
		smp_line(__LINE__, __FILE__); Variable delta(index = Idx, name = "delta"); this->delta.setEntity(delta); delta.entryOutput();
		smp_line(__LINE__, __FILE__); Variable delta_dot(index = Idx, name = "delta_dot"); this->delta_dot.setEntity(delta_dot); delta_dot.entryOutput();
		smp_line(__LINE__, __FILE__); Variable v_front_l(index = Idx, name = "v_front_l"); this->v_front_l.setEntity(v_front_l); v_front_l.entryOutput();
		smp_line(__LINE__, __FILE__); Variable v_front_r(index = Idx, name = "v_front_r"); this->v_front_r.setEntity(v_front_r); v_front_r.entryOutput();
		smp_line(__LINE__, __FILE__); Variable v_center_l(index = Idx, name = "v_center_l"); this->v_center_l.setEntity(v_center_l); v_center_l.entryOutput();
		smp_line(__LINE__, __FILE__); Variable v_center_r(index = Idx, name = "v_center_r"); this->v_center_r.setEntity(v_center_r); v_center_r.entryOutput();
		smp_line(__LINE__, __FILE__); Variable v_rear_l(index = Idx, name = "v_rear_l"); this->v_rear_l.setEntity(v_rear_l); v_rear_l.entryOutput();
		smp_line(__LINE__, __FILE__); Variable v_rear_r(index = Idx, name = "v_rear_r"); this->v_rear_r.setEntity(v_rear_r); v_rear_r.entryOutput();
		smp_line(__LINE__, __FILE__); Variable V_inv(index = Idx, name = "V_inv"); this->V_inv.setEntity(V_inv); V_inv.entryOutput();
		
		smp_line(__LINE__, __FILE__); VariableParameter T_delta(name = "T_delta"); this->T_delta.setEntity(T_delta); T_delta.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter a11(name = "a11"); this->a11.setEntity(a11); a11.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter a12(name = "a12"); this->a12.setEntity(a12); a12.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter a21(name = "a21"); this->a21.setEntity(a21); a21.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter a22(name = "a22"); this->a22.setEntity(a22); a22.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter b1(name = "b1");	this->b1.setEntity(b1); b1.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter b2(name = "b2");	this->b2.setEntity(b2); b2.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Q_vel(name = "Q_vel"); this->Q_vel.setEntity(Q_vel); Q_vel.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Q_acc(name = "Q_acc"); this->Q_acc.setEntity(Q_acc); Q_acc.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Q_v(name = "Q_v"); this->Q_v.setEntity(Q_v); Q_v.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Q_v_dot(name = "Q_v_dot"); this->Q_v_dot.setEntity(Q_v_dot); Q_v_dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Q_v_2dot(name = "Q_v_2dot"); this->Q_v_2dot.setEntity(Q_v_2dot); Q_v_2dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Q_theta(name = "Q_theta"); this->Q_theta.setEntity(Q_theta); Q_theta.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Q_theta_dot(name = "Q_theta_dot"); this->Q_theta_dot.setEntity(Q_theta_dot); Q_theta_dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Q_theta_2dot(name = "Q_theta_2dot"); this->Q_theta_2dot.setEntity(Q_theta_2dot); Q_theta_2dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Q_delta(name = "Q_delta"); this->Q_delta.setEntity(Q_delta); Q_delta.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Q_delta_dot(name = "Q_delta_dot"); this->Q_delta_dot.setEntity(Q_delta_dot); Q_delta_dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Sf_vel(name = "Sf_vel"); this->Sf_vel.setEntity(Sf_vel); Sf_vel.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Sf_acc(name = "Sf_acc"); this->Sf_acc.setEntity(Sf_acc); Sf_acc.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Sf_v(name = "Sf_v"); this->Sf_v.setEntity(Sf_v); Sf_v.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Sf_v_dot(name = "Sf_v_dot"); this->Sf_v_dot.setEntity(Sf_v_dot); Sf_v_dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Sf_v_2dot(name = "Sf_v_2dot"); this->Sf_v_2dot.setEntity(Sf_v_2dot); Sf_v_2dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Sf_theta(name = "Sf_theta"); this->Sf_theta.setEntity(Sf_theta); Sf_theta.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Sf_theta_dot(name = "Sf_theta_dot"); this->Sf_theta_dot.setEntity(Sf_theta_dot); Sf_theta_dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Sf_theta_2dot(name = "Sf_theta_2dot"); this->Sf_theta_2dot.setEntity(Sf_theta_2dot); Sf_theta_2dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Sf_delta(name = "Sf_delta"); this->Sf_delta.setEntity(Sf_delta); Sf_delta.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Sf_delta_dot(name = "Sf_delta_dot"); this->Sf_delta_dot.setEntity(Sf_delta_dot); Sf_delta_dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter width(name = "width"); this->width.setEntity(width); width.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter dist_front(name = "dist_front"); this->dist_front.setEntity(dist_front); dist_front.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter dist_rear(name = "dist_rear"); this->dist_rear.setEntity(dist_rear); dist_rear.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter theta_front(name = "theta_front"); this->theta_front.setEntity(theta_front); theta_front.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter theta_rear(name = "theta_rear"); this->theta_rear.setEntity(theta_rear); theta_rear.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter init_u(name = "init_u"); this->init_u.setEntity(init_u); init_u.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter init_vel(name = "init_vel"); this->init_vel.setEntity(init_vel); init_vel.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter init_v(name = "init_v"); this->init_v.setEntity(init_v); init_v.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter init_v_dot(name = "init_v_dot"); this->init_v_dot.setEntity(init_v_dot); init_v_dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter y_2dot_init(name = "y_2dot_init"); this->y_2dot_init.setEntity(y_2dot_init); y_2dot_init.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter init_theta(name = "init_theta"); this->init_theta.setEntity(init_theta); init_theta.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter init_theta_dot(name = "init_theta_dot"); this->init_theta_dot.setEntity(init_theta_dot); init_theta_dot.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter theta_2dot_init(name = "theta_2dot_init"); this->theta_2dot_init.setEntity(theta_2dot_init); theta_2dot_init.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter init_delta(name = "init_delta"); this->init_delta.setEntity(init_delta); init_delta.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter delta_dot_init(index = Idx, name = "delta_dot_init"); this->delta_dot_init.setEntity(delta_dot_init); delta_dot_init.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter acc_init(index = Idx, name = "acc_init"); this->acc_init.setEntity(acc_init); acc_init.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter vel_ref(index = Idx, name = "vel_ref"); this->vel_ref.setEntity(vel_ref); vel_ref.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter vel_max(index = Idx, name = "vel_max"); this->vel_max.setEntity(vel_max); vel_max.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter v_ref(index = Idx, name = "v_ref");	this->v_ref.setEntity(v_ref); v_ref.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter v_max(index = Idx, name = "v_max");	this->v_max.setEntity(v_max); v_max.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter v_min(index = Idx, name = "v_min");	this->v_min.setEntity(v_min); v_min.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter v_front_max(index = Idx, name = "v_front_max");	this->v_front_max.setEntity(v_front_max); v_front_max.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter v_front_min(index = Idx, name = "v_front_min");	this->v_front_min.setEntity(v_front_min); v_front_min.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter v_rear_max(index = Idx, name = "v_rear_max");	this->v_rear_max.setEntity(v_rear_max); v_rear_max.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter v_rear_min(index = Idx, name = "v_rear_min");	this->v_rear_min.setEntity(v_rear_min); v_rear_min.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter Rho(index = Idx, name = "Rho");	this->Rho.setEntity(Rho); Rho.entryOutput();

		//歩行者
		smp_line(__LINE__, __FILE__); VariableParameter x_pd(name = "x_pd");	this->x_pd.setEntity(x_pd); x_pd.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter y_pd(name = "y_pd");	this->y_pd.setEntity(y_pd); y_pd.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter vel_pd(name = "vel_pd");	this->vel_pd.setEntity(vel_pd); vel_pd.entryOutput();
		smp_line(__LINE__, __FILE__); VariableParameter closs_pd(name = "closs_pd");	this->closs_pd.setEntity(closs_pd); closs_pd.entryOutput();
		smp_line(__LINE__, __FILE__); Variable Dist(index = Idx, name = "Dist");	this->Dist.setEntity(Dist); Dist.entryOutput();
		smp_line(__LINE__, __FILE__); Variable x_PD(index = Idx, name = "x_PD");	this->x_PD.setEntity(x_PD); x_PD.entryOutput();
		smp_line(__LINE__, __FILE__); Variable y_PD(index = Idx, name = "y_PD");	this->y_PD.setEntity(y_PD); y_PD.entryOutput();

		smp_line(__LINE__, __FILE__); u[0] == init_u;
		smp_line(__LINE__, __FILE__); vel[0] == init_vel;
		smp_line(__LINE__, __FILE__); v[0] == init_v;
		smp_line(__LINE__, __FILE__); theta[0] == init_theta;
		smp_line(__LINE__, __FILE__); delta[0] == init_delta;
		smp_line(__LINE__, __FILE__); v_dot[0] == init_v_dot;
		smp_line(__LINE__, __FILE__); theta_dot[0] == init_theta_dot;
		smp_line(__LINE__, __FILE__); v_front_l[0] == v[0] + dist_front * sin(theta[0] + theta_front);
		smp_line(__LINE__, __FILE__); v_front_r[0] == v[0] + dist_front * sin(theta[0] - theta_front);
		smp_line(__LINE__, __FILE__); v_center_l[0] == v[0] + (dist_front * sin(theta[0] + theta_front) + dist_rear * sin(theta[0] + M_PI - theta_rear)) / 2;
		smp_line(__LINE__, __FILE__); v_center_r[0] == v[0] + (dist_front * sin(theta[0] - theta_front) + dist_rear * sin(theta[0] + M_PI + theta_rear)) / 2;
		smp_line(__LINE__, __FILE__); v_rear_l[0] == v[0] + dist_rear * sin(theta[0] + M_PI - theta_rear);
		smp_line(__LINE__, __FILE__); v_rear_r[0] == v[0] + dist_rear * sin(theta[0] + M_PI + theta_rear);
		//
		smp_line(__LINE__, __FILE__); x_PD[0] == x_pd;
		smp_line(__LINE__, __FILE__); y_PD[0] == y_pd;
		//
		


		///////////// Dynamic Bicycle Model ////////////////////
		smp_line(__LINE__, __FILE__); for (int k = 0; k < rcd_horizon; k++)
		{
			smp_line(__LINE__, __FILE__); vel[k + 1] == vel[k] + acc[k] * T_delta;
			smp_line(__LINE__, __FILE__); V_inv[k] == 1 / (vel[k] + log(1 + exp(-2 * vel[k])));
			smp_line(__LINE__, __FILE__); u[k + 1] == u[k] + vel[k] * T_delta;
			smp_line(__LINE__, __FILE__); v_2dot[k] == -a11 * v_dot[k] * V_inv[k] + (a11 + acc[k]) * theta[k] + a12 * theta_dot[k] * V_inv[k] + b1 * delta[k] + (a12 - vel[k] * vel[k]) * Rho[k];
			smp_line(__LINE__, __FILE__); v_dot[k + 1] == v_dot[k] + v_2dot[k] * T_delta;
			smp_line(__LINE__, __FILE__); v[k + 1] == v[k] + v_dot[k] * T_delta;
			smp_line(__LINE__, __FILE__); theta_2dot[k] == -a21 * v_dot[k] * V_inv[k] + a21 * theta[k] + a22 * theta_dot[k] * V_inv[k] + b2 * delta[k] + (a22 - acc[k]) * Rho[k];
			smp_line(__LINE__, __FILE__); theta_dot[k + 1] == theta_dot[k] + theta_2dot[k] * T_delta;
			smp_line(__LINE__, __FILE__); theta[k + 1] == theta[k] + theta_dot[k] * T_delta;
			smp_line(__LINE__, __FILE__); delta[k + 1] == delta[k] + delta_dot[k] * T_delta;
			smp_line(__LINE__, __FILE__); v_front_l[k + 1] == v[k + 1] + dist_front * sin(theta[k + 1] + theta_front);
			smp_line(__LINE__, __FILE__); v_front_r[k + 1] == v[k + 1] + dist_front * sin(theta[k + 1] - theta_front);
			smp_line(__LINE__, __FILE__); v_center_l[k + 1] == v[k + 1] + (dist_front * sin(theta[k + 1] + theta_front) + dist_rear * sin(theta[k + 1] + M_PI - theta_rear)) / 2;
			smp_line(__LINE__, __FILE__); v_center_r[k + 1] == v[k + 1] + (dist_front * sin(theta[k + 1] - theta_front) + dist_rear * sin(theta[k + 1] + M_PI + theta_rear)) / 2;
			smp_line(__LINE__, __FILE__); v_rear_l[k + 1] == v[k + 1] + dist_rear * sin(theta[k + 1] + M_PI - theta_rear);
			smp_line(__LINE__, __FILE__); v_rear_r[k + 1] == v[k + 1] + dist_rear * sin(theta[k + 1] + M_PI + theta_rear);
			//
			smp_line(__LINE__, __FILE__); Dist[k] == pow(pow(u[k] - x_PD[k], 2) + pow(v[k] - y_PD[k], 2), 0.5);
			//

			//歩行者
			smp_line(__LINE__, __FILE__); x_PD[k + 1] == x_PD[k];
			smp_line(__LINE__, __FILE__); y_PD[k + 1] == y_PD[k] - vel_pd * T_delta;
			

		}
		smp_line(__LINE__, __FILE__); Dist[rcd_horizon] == pow(pow(u[rcd_horizon] - x_PD[rcd_horizon], 2) + pow(v[rcd_horizon] - y_PD[rcd_horizon], 2), 0.5);

		//制約条件
		smp_line(__LINE__, __FILE__); acc[Idx] >= -3, Idx;
		smp_line(__LINE__, __FILE__); acc[Idx] <= 3.3, Idx;
		smp_line(__LINE__, __FILE__); vel[Idx] >= 0, Idx;
		smp_line(__LINE__, __FILE__); vel[Idx] <= vel_max[Idx], Idx;
		smp_line(__LINE__, __FILE__); v_front_r[Idx] >= v_front_min[Idx], Idx;
		smp_line(__LINE__, __FILE__); v_front_l[Idx] <= v_front_max[Idx], Idx;
		smp_line(__LINE__, __FILE__); v_center_r[Idx] >= v_min[Idx], Idx;
		smp_line(__LINE__, __FILE__); v_center_l[Idx] <= v_max[Idx], Idx;
		//smp_line(__LINE__, __FILE__); v_rear_r[Idx] >= v_rear_min[Idx], Idx;
		//smp_line(__LINE__, __FILE__); v_rear_l[Idx] <= v_rear_max[Idx], Idx;

		//歩行者がいる時の制約
		smp_line(__LINE__, __FILE__); Dist[Idx] >= 0.8, Idx;
		//歩行者と車の距離が1m以上の制約




		smp_line(__LINE__, __FILE__); delta[Idx] <= 1.0472, Idx;
		smp_line(__LINE__, __FILE__); delta[Idx] >= -1.0472, Idx;
		//smp_line(__LINE__, __FILE__); delta_dot[Idx] <= 0.5, Idx;
		//smp_line(__LINE__, __FILE__); delta_dot[Idx] >= -0.5, Idx;
		//smp_line(__LINE__, __FILE__); delta_dot[Idx] <= 0.15, Idx;
		//smp_line(__LINE__, __FILE__); delta_dot[Idx] >= -0.15, Idx;


		smp_line(__LINE__, __FILE__); Objective obj(type = minimize, name = "obj"); this->obj.setEntity(obj); obj.entryOutput();
		smp_line(__LINE__, __FILE__);
		obj =
			+Sf_v * (v[rcd_horizon] - v_ref[rcd_horizon]) * (v[rcd_horizon] - v_ref[rcd_horizon])
			+ Sf_v_dot * v_dot[rcd_horizon] * v_dot[rcd_horizon]
			+ Sf_theta * theta[rcd_horizon] * theta[rcd_horizon]
			+ Sf_theta_dot * theta_dot[rcd_horizon] * theta_dot[rcd_horizon]
			+ Sf_v_2dot * v_2dot[rcd_horizon] * v_2dot[rcd_horizon]
			+ Sf_theta_2dot * theta_2dot[rcd_horizon] * theta_2dot[rcd_horizon]
			+ Sf_vel * (vel[rcd_horizon] - vel_ref[rcd_horizon]) * (vel[rcd_horizon] - vel_ref[rcd_horizon])
			+ Sf_acc * acc[rcd_horizon] * acc[rcd_horizon]
			+ Sf_delta * delta[rcd_horizon] * delta[rcd_horizon]
			+ Sf_delta_dot * delta_dot[rcd_horizon] * delta_dot[rcd_horizon]

			+sum(Q_v * (v[Idx_eval] - v_ref[Idx_eval]) * (v[Idx_eval] - v_ref[Idx_eval])
				+ Q_v_dot * v_dot[Idx_eval] * v_dot[Idx_eval]
				+ Q_theta * theta[Idx_eval] * theta[Idx_eval]
				+ Q_theta_dot * theta_dot[Idx_eval] * theta_dot[Idx_eval]
				+ Q_v_2dot * v_2dot[Idx_eval] * v_2dot[Idx_eval]
				+ Q_theta_2dot * theta_2dot[Idx_eval] * theta_2dot[Idx_eval]
				+ Q_vel * (vel[Idx_eval] - vel_ref[Idx_eval]) * (vel[Idx_eval] - vel_ref[Idx_eval])
				+ Q_acc * acc[Idx_eval] * acc[Idx_eval]
				+ Q_delta * delta[Idx_eval] * delta[Idx_eval]
				+ Q_delta_dot * delta_dot[Idx_eval] * delta_dot[Idx_eval], Idx_eval);

		smp_line(__LINE__, __FILE__); smp_line(244 - 1, "NUOPT.smp");
	}
	endSystem();
}

#ifndef DATA_CHECK
void checkdata(int check) {
	printf("This load module does not have datacheck mode.\n");
}
#endif /* DATACHECK */