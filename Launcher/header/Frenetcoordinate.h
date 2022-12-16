#pragma once

#define _USE_MATH_DEFINES

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <Eigen/Core>

struct FrenetPoint
{
	typedef Eigen::Vector2d Vector;
	Vector pos;
	FrenetPoint* next;
	FrenetPoint* prev;

	// u 
	double du;	// distance to next
	double u;	// distance from start along path
			// vector to next
	Vector to_next;
	double to_next_theta;

	// normal vector to next
	Vector t_to_next;
	Vector n_to_next;

	// tangent vector
	Vector t;
	double t_theta;

	// normal vector
	Vector n;
	double n_theta;

	// curvature
	double kappa;


	FrenetPoint() : to_next(0, 0), t_to_next(0, 0), n_to_next(0, 0),
		t(0, 0), n(0, 0)
	{
		prev = nullptr;
		next = nullptr;
		u = 0; du = std::numeric_limits<double>::infinity();
		to_next_theta = 0;
		t_theta = 0;
		n_theta = 0;
		kappa = 0;
	}

	// Compute curvature Kappa from (*_from) until (*_to)
	void ComputeKappa(FrenetPoint *_from, FrenetPoint *_to)
	{
		double len = 0;
		FrenetPoint* cur = _from;
		while (cur != nullptr && cur != _to)
		{
			len += cur->du;
			cur = cur->next;
		}
		double angle_diff = _to->to_next_theta - _from->to_next_theta;
		if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
		if (angle_diff < -M_PI) angle_diff += 2 * M_PI;
		kappa = angle_diff / (len);
	}

	void CalculateU(FrenetPoint &_prev)
	{
		_prev.du = (pos - _prev.pos).norm();
		u = _prev.u + _prev.du;
		prev = &_prev;
	}

	void ConnectToNext(FrenetPoint &_next)
	{
		next = &_next;
		to_next = _next.pos - pos;
		t_to_next = to_next.normalized();
		n_to_next.x() = -t_to_next.y();
		n_to_next.y() = t_to_next.x();
		to_next_theta = atan2(t_to_next.y(), t_to_next.x());

		// Compute tangent
		if (prev != nullptr)
		{
			double angle_diff = to_next_theta - prev->to_next_theta;
			if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
			if (angle_diff < -M_PI) angle_diff += 2 * M_PI;
			prev->kappa = angle_diff * 2 / (prev->du + du);
		}
	}

	void OutputLaneTagsForDebug(std::ofstream &ofs)
	{
		ofs << "x, y, du, u,"
			<< "e_to_next_x, e_to_next_y, to_next_theta,";

		ofs << "tx, ty, t_theta,";
		ofs << "nx, ny, n_theta,";
		ofs << "kappa" << std::endl;
	}

	void OutputLaneForDebug(std::ofstream& ofs)
	{
		ofs << pos.x() << "," << pos.y() << ","
			<< du << "," << u << ","
			<< t_to_next.x() << "," << t_to_next.y() << ","
			<< to_next_theta << ",";

		ofs << t.x() << "," << t.y() << "," << t_theta << ",";
		ofs << n.x() << "," << n.y() << "," << n_theta << ",";
		ofs << kappa << std::endl;
		//ofs << kappa << "," << std::endl;
	}

	inline Vector GetLocalPos(Vector target)
	{
		Vector diff = target - pos;
		double u = t_to_next.x() * diff.x()
			+ t_to_next.y() * diff.y();
		double v = n_to_next.x() * diff.x()
			+ n_to_next.y() * diff.y();
		return Vector(u, v);
	}
};

struct CachetoGlobal {
	bool initialized;
	size_t prev_index;
	int lap;
	CachetoGlobal()
		: prev_index(0), lap(0), initialized(0) {};
};

struct CachetoFrenet
{
	bool initialized;
	size_t prev_index;
	int lap;
	CachetoFrenet()
		: prev_index(0), lap(0), initialized(0) {};

};

class FrenetLib
{
public:
	bool isLooping = false;
	std::vector<FrenetPoint> points;
	std::vector<double> u_cache;
	std::vector<size_t> anchors;
	double length;
	size_t size;

	void LoadPath(std::vector<double> ref_x, std::vector<double> ref_y, bool bLoop)
	{
		isLooping = bLoop;
		setXY(ref_x, ref_y);
		compute_aux();
		WritePath(std::ofstream("Path/resampled_path.csv"));
		OutputLaneForDebug("Path/detailed_path.csv");
	}

	// ---------------------------------  coordinate conversion
	// get global position from Frenet coordinate
	CachetoGlobal GetGlobal(double u, double v,
		double &x, double &y, CachetoGlobal prev)
	{
		CachetoGlobal cache = search_u(u, prev);
		size_t index = cache.prev_index;

		u = normalize_u(u, cache.lap);
		double diffu = u - points[index].u;
		double diffv = v;

		Eigen::Vector2d gpos =
			points[index].pos
			+ points[index].t_to_next * diffu
			+ points[index].n_to_next * diffv;

		x = gpos.x();
		y = gpos.y();

		return cache;
	}

	// get global position from Frenet coordinate together with yaw angle
	CachetoGlobal GetGlobal(double u, double v, double theta_e,
		double &x, double &y, double& yaw, CachetoGlobal prev)
	{
		CachetoGlobal cache = GetGlobal(u, v, x, y, prev);
		size_t index = cache.prev_index;

		yaw = points[index].to_next_theta + theta_e;
		if (theta_e > M_PI) theta_e -= 2 * M_PI;
		if (theta_e < -M_PI) theta_e += 2 * M_PI;

		return cache;
	}

	CachetoFrenet GetFrenet(double x, double y, double yaw, double& u, double& v, double& theta_e, CachetoFrenet prev)
	{
		CachetoFrenet cache = GetFrenet(x, y, u, v, prev);
		size_t index = cache.prev_index;
		//theta_e = yaw - points[index].to_next_theta;

		double y_x = cos(yaw);
		double y_y = sin(yaw);
		const Eigen::Vector2d &t = points[index].t_to_next;
		double t_x = t.x();
		double t_y = t.y();
		// get angle by outer prod
		double angle_diff_sin = y_y * t_x - y_x * t_y;
		double angle_diff = asin(angle_diff_sin);

		theta_e = angle_diff;

		return cache;
	}

	CachetoFrenet GetFrenet(double x, double y, double& u, double& v, CachetoFrenet cache)
	{
		int index;
		if (cache.initialized)
		{
			index = cache.prev_index;
		}
		else
		{
			index = find_basepoint(x, y); //Å‚à‹ß‚¢“_‚ð’Tõ
		}

		// Get local position from base point
		Eigen::Vector2d target(x, y);
		Eigen::Vector2d uv = points[index].GetLocalPos(target);

		// backward search
		while (uv.x() < 0)
		{	// compute inner dot with tangent
			if (isLooping)
			{
				index = index - 1;
				if (index < 0) index = size - 1;
			}
			else
				index = index > 0 ? index=index-1 : 0;

			uv = points[index].GetLocalPos(target);
		}
		// forward search
			// if local u > 0 from next point, go next
		while (uv.x() > 0)
		{
			if (uv.x() <= points[index].du) break;
			// vector from base point to current point
			size_t next = index;
			if (isLooping)
				next = (index + 1) % size;
			else
				next = (index + 1) > (size - 1) ? (size - 1) : index + 1;

			Eigen::Vector2d uv_from_next
				= points[next].GetLocalPos(target);

			if (uv.x() > 0 && uv_from_next.x() < 0)
			{
				break;
			}
			index = next;
			uv = uv_from_next;
		}
		u = uv.x() + points[index].u;
		v = uv.y();
		CachetoFrenet ret;
		ret.prev_index = index;
		ret.lap = cache.lap;
		ret.initialized = true;
		return ret;
	}

	void WritePath(std::ofstream ofs)
	{
		ofs << "x, y, z, nx, ny, nz, " << std::endl;
		for (size_t i = 0; i < size; i++)
		{
			ofs << points[i].pos.x() << ", "
				<< points[i].pos.y() << ", 0, "
				<< "0, 0, 1, " << std::endl;
		}
	}

	// just for debugging
	void OutputLaneForDebug(std::string testfn)
	{
		std::ofstream ofs(testfn);
		points[0].OutputLaneTagsForDebug(ofs);

		for (size_t i = 0; i < size; i++)
		{
			points[i].OutputLaneForDebug(ofs);
		}

	}

	void OutputRho(std::vector<std::vector<double>>& course)
	{
		for (size_t i = 0; i < course[0].size(); i++)
		{
			course[10][i] = points[i].kappa;
		}
		for (size_t i = 0; i < course[0].size() - 1; i++)
		{
			course[11][i] = (course[10][i + 1] - course[10][i]) / (course[2][i + 1] - course[2][i]);
		}
	}


private:
	inline void setXY(std::vector<double> ref_x, std::vector<double> ref_y)
	{
		points.resize(ref_x.size());
		for (size_t i = 0; i < ref_x.size(); i++)
		{

			points[i].pos.x() = ref_x[i];
			points[i].pos.y() = ref_y[i];
		}
		size = points.size();
	}

	inline void compute_aux()
	{
		computeU();
		connect();
		make_anchors();
		compute_kappa();
	}

	// this must be called after SetXY
	inline void computeU()
	{
		u_cache.resize(points.size());
		points[0].u = 0;
		points[0].du = 0;
		u_cache[0] = 0;
		for (size_t i = 1; i < points.size(); i++)
		{
			points[i].CalculateU(points[i - 1]);
			u_cache[i] = points[i].u;
		}
		if (isLooping)
		{	// connect last point to first
			points[0].CalculateU(points[size - 1]);
			points[0].u = 0;
		}
		else 
		{
			points[size - 1].du = 10e10;
		}
		// if loop course, add line length from last point to start point
		if (isLooping) 
		{
			length = points.back().u;// +points.back().du;
		}
		else
		{
			length = points.back().u;
		}
	}

	inline void connect()
	{
		for (size_t i = 0; i < points.size() - 1; i++)
		{
			points[i].ConnectToNext(points[i + 1]);
		}
		if (isLooping)
		{	// connect last point to first
			points[size - 1].ConnectToNext(points[0]);
		}
		else {
			FrenetPoint &last = points[size - 1];
			last.next = nullptr;
			last.to_next = points[size - 2].to_next;
			last.t_to_next = points[size - 2].t_to_next;
			last.n_to_next = points[size - 2].n_to_next;
			last.to_next_theta = points[size - 2].to_next_theta;
			last.t = points[size - 2].t;
			last.t_theta = points[size - 2].t_theta;
			last.n = points[size - 2].n;
			last.n_theta = points[size - 2].n_theta;
			last.kappa = points[size - 2].kappa;
			points[0].kappa = points[1].kappa;
		}
	}

	CachetoGlobal search_u(double u, CachetoGlobal cache)
	{
		int lap = 0;
		// un-looping 
		u = normalize_u(u, lap);

		size_t index = 0;
		size_t n = u_cache.size();
		if (cache.initialized == false)
		{	// find from all : binary search
			size_t front = 0;
			size_t back = n - 1;
			while (back - front > 3)
			{
				size_t mid = (front + back) / 2;
				if (u_cache[mid] < u)
				{
					front = mid;
				}
				else {
					back = mid;
				}
				index = mid;
			}
		}
		else {
			index = cache.prev_index;
		}

		if (index < 0)index = 0;
		if (index >= n) index = n - 1;
		// search around mid

		// backward search
		while (index > 0 && u < u_cache[index])
		{
			index--;
		}
		// forward search
		while (index + 1 < size && u > u_cache[index + 1])
		{
			index++;
		}
		CachetoGlobal ret;
		ret.initialized = true;
		ret.prev_index = index;
		ret.lap = lap;
		return ret;
	}

	// make anchor points for quick search of Frenet point
	void make_anchors()
	{
		double len = length;
		const int N = 90;

		double u = 0;
		anchors.push_back(0);
		for (size_t i = 1; i < N; i++)
		{
			u += length / N;
			auto itr = upper_bound(u_cache.begin(), u_cache.end(), u);
			size_t index = itr - u_cache.begin();
			if (anchors.back() != index)
			{
				anchors.push_back(index);
			}
		}
		if (anchors.back() != size - 1)
		{
			anchors.push_back(size - 1);
		}
	}

	inline void compute_kappa()
	{
		const int N_kappa = 2;
		for (int c = N_kappa; c < size - N_kappa - 1; c++)
		{
			points[c].ComputeKappa(&points[c - N_kappa], &points[c + N_kappa]);
		}
	}

	//find nearest base point
	size_t find_basepoint(double x, double y)
	{
		double min_dist = 10e10;
		size_t min_ind = 0;
		for (size_t i = 0; i < anchors.size(); i++)
		{
			double diffx = points[anchors[i]].pos.x() - x;
			double diffy = points[anchors[i]].pos.y() - y;
			double sq_dist = x * x + y * y;
			if (sq_dist < min_dist)
			{
				min_dist = sq_dist;
				min_ind = anchors[i];
			}
		}
		return min_ind;
	}

	// normalize u by unwrapping in looped course.
	// for normal course, return u.
	// for loop course, adjust in range of 0-length
	double normalize_u(double u, int& lap)
	{
		int _lap = lap;
		// un-looping 
		if (isLooping)
		{
			while (u < 0)
			{
				u += length;
				lap--;
			}
			while (u > length)
			{
				u -= length;
				lap++;
			}
		}
		return u;
	}
};

class Frenet
{
public:
	FrenetLib frenetlib;
	CachetoFrenet Cache_f;
	CachetoGlobal Cache_g;
};