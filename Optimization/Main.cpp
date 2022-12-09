#include <Windows.h>
#include <header/Frenetcoordinate.h>
#include <Optimization/MyOptimization.h>

//共有メモリの設定
constexpr auto SHARED_MEMORY_NAME = L"MySharedMemory";
constexpr auto SHARED_MEMORY_SIZE = 8 * 6500;
static HANDLE hSharedMemory = NULL;
SharedData* shareddata;

bool OpenSharedMemory(const wchar_t* sharedmemoryname)
{
	if (hSharedMemory)
	{
		return FALSE;
	}

	hSharedMemory = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, sharedmemoryname);
	if (!hSharedMemory || hSharedMemory == INVALID_HANDLE_VALUE)
	{
		return FALSE;
	}

	return TRUE;
}

bool ReadSharedMemory(DWORD size)
{
	if (!hSharedMemory)
	{
		return FALSE;
	}

	shareddata = (SharedData*)MapViewOfFile(hSharedMemory, FILE_MAP_ALL_ACCESS, NULL, NULL, size);
	if (shareddata == NULL || !shareddata)
	{
		return FALSE;
	}

	return TRUE;
}

//結果の共有メモリへの書き込み
bool SetSharedData(MyProblem myProblem)
{
	if (!shareddata || !hSharedMemory)
	{
		return FALSE;
	}

	for (int i = 0; i < vsize; i++)
	{
		shareddata->u[i] = myProblem.u[i];
		shareddata->v[i] = myProblem.v[i];
		shareddata->theta[i] = myProblem.theta[i];
		shareddata->vel[i] = myProblem.vel[i];
		shareddata->v_dot[i] = myProblem.v_dot[i];
		shareddata->theta_dot[i] = myProblem.theta_dot[i];
		shareddata->acc[i] = myProblem.acc[i];
		shareddata->v_2dot[i] = myProblem.v_2dot[i];
		shareddata->theta_2dot[i] = myProblem.theta_2dot[i];
		shareddata->delta[i] = myProblem.delta[i];
		shareddata->delta_dot[i] = myProblem.delta_dot[i];
		shareddata->x[i] = myProblem.x[i];
		shareddata->y[i] = myProblem.y[i];
		shareddata->yaw[i] = myProblem.yaw[i];
		shareddata->lateral_jerk[i] = myProblem.lateral_jerk[i];
		shareddata->longitudinal_jerk[i] = myProblem.longitudinal_jerk[i];
		//
		shareddata->x_PD[i] = myProblem.x_PD[i];
		shareddata->y_PD[i] = myProblem.y_PD[i];
		shareddata->u_front_r[i] = myProblem.u_front_r[i];
		shareddata->u_front_l[i] = myProblem.u_front_r[i];
		shareddata->u_center_r[i] = myProblem.u_center_r[i];
		shareddata->u_center_l[i] = myProblem.u_center_l[i];
		shareddata->u_rear_r[i] = myProblem.u_rear_r[i];
		shareddata->u_rear_l[i] = myProblem.u_rear_l[i];


		//
	}

	shareddata->error_code = myProblem.error_code;
	shareddata->iters = myProblem.iters;
	shareddata->fevals = myProblem.fevals;
	shareddata->eps = myProblem.eps;
	shareddata->elapse_time = myProblem.elapse_time;
	shareddata->optValue = myProblem.optValue;
	shareddata->tolerance = myProblem.tolerance;
	shareddata->residual = myProblem.residual;
	shareddata->average_lateral_jerk = myProblem.average_lateral_jerk;
	shareddata->average_longitudinal_jerk = myProblem.average_longitudinal_jerk;
	shareddata->success = 1;

	return TRUE;
}

int main()
{
	if (!OpenSharedMemory(SHARED_MEMORY_NAME))
	{
		std::cout << "共有メモリの作成に失敗しました。\n";
	}

	if (!ReadSharedMemory(SHARED_MEMORY_SIZE))
	{
		std::cout << "共有メモリの読み取りに失敗しました。\n";
	}

	Frenet frenet;
	LinearInterporater table;
	MyProblem myProblem(shareddata);
	getconstraint constraint(vsize);

	frenet.frenetlib.LoadPath(shareddata->course[0], shareddata->course[1], csize, true);
	table.GetCourse(shareddata->course);
	constraint.InitCourse(table);

	myProblem.InitState(shareddata);
	myProblem.SetFront_u();
	constraint.GetConstraint(myProblem.u, myProblem.u_front_l, myProblem.u_front_r, myProblem.u_center_l, myProblem.u_center_r, myProblem.u_rear_l, myProblem.u_rear_r);
	myProblem.SetConstraints(constraint.v_max, constraint.v_min, constraint.v_ref, constraint.vel_max, constraint.rho, constraint.v_front_max, constraint.v_front_min, constraint.v_rear_max, constraint.v_rear_min);
	myProblem.SetV(shareddata->vel_ref);
	myProblem.SetAllState();
	myProblem.Solve();

	//軌跡をGlobalに変換
	for (int i = 0; i < vsize; i++)
	{
		frenet.Cache_g = frenet.frenetlib.GetGlobal(myProblem.u[i], myProblem.v[i], myProblem.theta[i], myProblem.x[i], myProblem.y[i], myProblem.yaw[i], frenet.Cache_g);
	}

	if (!SetSharedData(myProblem))
	{
		std::cout << "共有メモリの書き込みに失敗しました。\n";
	}
	
	if (shareddata != NULL)
	{
		UnmapViewOfFile(shareddata);
	}

	if (hSharedMemory != NULL)
	{
		CloseHandle(hSharedMemory);
	}

	return 0;
}