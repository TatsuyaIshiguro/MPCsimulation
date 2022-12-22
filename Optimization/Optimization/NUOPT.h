// -*- Mode: C++ -*-
// auto generated
#line 1 "NUOPT.h"

#ifndef __NUOPT__h
#define __NUOPT__h

#ifdef __GNUC__
  #include <string.h>
#endif
#pragma sint off
#ifndef CINT
#include "SimpleObjProxy.h"
#include "SystemInterface.h"
#include "nuopt_exception.h"
#include "throw_exception.h"
#endif // ! CINT

#define index indsimple
// if you use Interval, please define USE_SIMPLE_INTERVAL macro
#ifdef USE_SIMPLE_INTERVAL
#define left  leftsimple
#define right rightsimple
#endif

class System_NUOPT : public SystemInterface {
public:
System_NUOPT(double init_x_pd[70],double init_y_pd[70]);
  ParameterProxy rcd_horizon;
  SetProxy tempSet;
  ElementProxy tempElem;
  ParameterProxy a;
  ParameterProxy b;
  SetProxy Step_cn;
  ElementProxy Idx;
  SetProxy Step_eval;
  ElementProxy Idx_eval;
  ParameterProxy eps;
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
  //
  VariableProxy v_front_l; //衝突判定用の状態
  VariableProxy v_front_r;
  VariableProxy v_center_l;
  VariableProxy v_center_r;
  VariableProxy v_rear_l;
  VariableProxy v_rear_r;
  //
  VariableParameterProxy u_front_l; //衝突判定用の状態
  VariableParameterProxy u_front_r;
  VariableParameterProxy u_rear_l;
  VariableParameterProxy u_rear_r;
  //
  VariableProxy V_inv;
  VariableParameterProxy T_delta;
  VariableParameterProxy a11;
  VariableParameterProxy a12;
  VariableParameterProxy a21;
  VariableParameterProxy a22;
  VariableParameterProxy b1;
  VariableParameterProxy b2;
  VariableParameterProxy Q_vel;
  VariableParameterProxy Q_acc;
  VariableParameterProxy Q_v;
  VariableParameterProxy Q_v_dot;
  VariableParameterProxy Q_v_2dot;
  VariableParameterProxy Q_theta;
  VariableParameterProxy Q_theta_dot;
  VariableParameterProxy Q_theta_2dot;
  VariableParameterProxy Q_delta;
  VariableParameterProxy Q_delta_dot;
  VariableParameterProxy Sf_vel;
  VariableParameterProxy Sf_acc;
  VariableParameterProxy Sf_v;
  VariableParameterProxy Sf_v_dot;
  VariableParameterProxy Sf_v_2dot;
  VariableParameterProxy Sf_theta;
  VariableParameterProxy Sf_theta_dot;
  VariableParameterProxy Sf_theta_2dot;
  VariableParameterProxy Sf_delta;
  VariableParameterProxy Sf_delta_dot;
  VariableParameterProxy width;
  VariableParameterProxy dist_front;
  VariableParameterProxy dist_rear;
  VariableParameterProxy theta_front;
  VariableParameterProxy theta_rear;
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

  //歩行者用のパラメータ
  VariableParameterProxy x_pd;
  VariableParameterProxy y_pd;
  VariableParameterProxy vel_pd;
  //VariableParameterProxy Q_pena_vel;
  //VariableParameterProxy Q_pena_dist;
  //
  //
  VariableParameterProxy x_PD;
  VariableParameterProxy y_PD;
  //
  //KBM用　
  VariableParameterProxy l_f;
  VariableParameterProxy l_r;
  VariableProxy beta;

  //


  VariableParameterProxy delta_dot_init;
  VariableParameterProxy vel_ref;
  VariableParameterProxy vel_max;
  VariableParameterProxy v_ref;
  VariableParameterProxy v_max;
  VariableParameterProxy v_min;
  VariableParameterProxy Rho;
  VariableParameterProxy v_front_max; //衝突判定用の制約
  VariableParameterProxy v_front_min;
  VariableParameterProxy v_rear_max; //衝突判定用の制約
  VariableParameterProxy v_rear_min;
  ObjectiveProxy obj;
};

#pragma sint on
#endif // ! __NUOPT__h
