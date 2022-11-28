#pragma once
#define _USE_MATH_DEFINES
#include <string>
#include <vector>
#include <math.h>
#include <CSVLoader/CSVLoader.h>

struct CourseSetting
{
	double start_x = 0.0;
	double start_y = 0.0;
	double delta = 2.0; //Change of x

//Obstacle avoidance
#define twoobstacle //One obstacle -> off，Two ->on
	//Change of x near obstacles
	double delta_ob = 0.1;
	//Sigmoid parameter
	double a;
	//First obstacle
	double width_1;
	std::string pos1; //upper side of lower side
	//Second obstacle
	double width_2;
	std::string pos2;
	double dist;

//Sine curve
	double ampl;
	double cycle;
	double delta_sine = 0.3;

//Path of csvfile
	std::string Path_coursecsv;

//Setting of constraints
	double v_min_const = -1.5;
	double v_max_const = 1.5;
};

class GenCourse
{
public:
	GenCourse();
	void GetSetting(CourseSetting setting);
	std::vector<std::vector<double>> Gen_SOA(); //static obstacle avoidance
	std:: vector<std::vector<double>> Gen_SINE();
	std::vector<std::vector<double>> Gen_Course_csv(std::string path);

private:
	int CPrmNum = 12; //Number of parameters 0x(road center), 1y(road center), 2u(reference path), 3v(reference path), 4v_min, 5v_max, 6x_min, 7y_min, 8x_max, 9y_max, 10rho

	CourseSetting csetting;
	//Two dimensional array for course
	std::vector<std::vector<double>> course;
	std::vector<std::vector<double>> ret; //Return array

	void sine_curve();
	void sigmoid_obstacle();
};

inline GenCourse::GenCourse()
{
	ret.resize(2);
	course.resize(CPrmNum);
	for (size_t i = 0; i < ret.size(); i++)
	{
		ret[i].resize(400);
	}
	for (size_t i = 0; i < course.size(); i++)
	{
		course[i].resize(400);
	}
}

inline void GenCourse::GetSetting(CourseSetting setting)
{
	this->csetting = setting;
}

//Course of static obstacles avoidance
//1.直線を作成（障害物区間は細かくx, yの点列をサンプリング） 2.障害物区間の点列を作成 3.元の点列を修正 4.コースの配列を返す
inline std::vector<std::vector<double>> GenCourse::Gen_SOA()
{
	course[0][0] = csetting.start_x;
	course[1][0] = csetting.start_y;

	for (size_t i = 1; i < course[0].size(); i++)
	{
		//Obstacle section (x = 45m - 60m, detailed sampling)
		if (course[0][i - 1] >= 45 && course[0][i - 1] <= 60)
		{
			course[0][i] = course[0][i - 1] + csetting.delta_ob; //x
		}
#ifdef twoobstacle
		//Obstacle section (detailed sampling)
		else if (course[0][i - 1] >= 45 + csetting.dist && course[0][i - 1] <= 60 + csetting.dist)
		{
			course[0][i] = course[0][i - 1] + csetting.delta_ob;
		}
#endif // twoobstacle
		else
		{
			course[0][i] = course[0][i - 1] + csetting.delta;
		}
		course[1][i] = course[1][i - 1]; //y
		course[2][i] = course[0][i]; //u
	}

	sigmoid_obstacle();

	for (size_t i = 0; i < course[0].size(); i++)
	{
		course[3][i] = (course[4][i] + course[5][i]) / 2; //modify reference path
		course[6][i] = course[2][i];  //global_x_min
		course[7][i] = course[4][i];  //global_y_min
		course[8][i] = course[2][i];  //global_x_max
		course[9][i] = course[5][i];  //global_y_max
	}
	return course;
}

inline void GenCourse::sigmoid_obstacle()
{
	for (size_t i = 0; i < course[0].size(); i++)
	{
		//First obstacle
		if (course[0][i] >= 46 && course[0][i] <= 60)
		{
			if (csetting.pos1 == "lower") //Lower side
			{
				//Bilateral symmetry at 53m
				if (course[0][i] <= 53)
				{
					course[4][i] = csetting.v_min_const + csetting.width_1 / (1 + exp(-csetting.a * (course[0][i] - 49.5))); //vmin
				}
				else
				{
					course[4][i] = csetting.v_min_const + csetting.width_1 / (1 + exp(-csetting.a * (-course[0][i] + 56.5)));
				}
				course[5][i] = csetting.v_max_const;
			}
			else //Upper side
			{
				if (course[0][i] <= 53)
				{
					course[5][i] = csetting.v_max_const - csetting.width_1 / (1 + exp(-csetting.a * (course[0][i] - 49.5))); //vmax
				}
				else
				{
					course[5][i] = csetting.v_max_const - csetting.width_1 / (1 + exp(-csetting.a * (-course[0][i] + 56.5)));
				}
				course[4][i] = csetting.v_min_const;
			}
		}
		else
		{
			course[4][i] = csetting.v_min_const;
			course[5][i] = csetting.v_max_const;
		}
	}

#ifdef twoobstacle
	for (size_t i = 0; i < course[0].size(); i++)
	{
		if (course[0][i] >= 46 + csetting.dist && course[0][i] <= 60 + csetting.dist)
		{
			if (csetting.pos2 == "lower")
			{
				if (course[0][i] <= 53 + csetting.dist)
				{
					course[4][i] = csetting.v_min_const + csetting.width_1 / (1 + exp(-csetting.a * (course[0][i] - 49.5 - csetting.dist)));
				}
				else
				{
					course[4][i] = csetting.v_min_const + csetting.width_1 / (1 + exp(-csetting.a * (-course[0][i] + 56.5 + csetting.dist)));
				}
			}
			else
			{
				if (course[0][i] <= 53 + csetting.dist)
				{
					course[5][i] = csetting.v_max_const - csetting.width_1 / (1 + exp(-csetting.a * (course[0][i] - 49.5 - csetting.dist)));
				}
				else
				{
					course[5][i] = csetting.v_max_const - csetting.width_1 / (1 + exp(-csetting.a * (-course[0][i] + 56.5 + csetting.dist)));
				}
			}
		}
	}
#endif // twoobstacle
}

//Course of sine curve
inline std::vector<std::vector<double>> GenCourse::Gen_SINE()
{
	sine_curve();
	course[0] = ret[0];
	course[1] = ret[1];

	for (size_t i = 0; i < course[0].size(); i++)
	{
		course[4][i] = csetting.v_min_const;
		course[5][i] = csetting.v_max_const;
	}
	return course;
}

inline void GenCourse::sine_curve()
{
	ret[0][0] = csetting.start_x;
	ret[1][0] = csetting.start_y;
	int i = 1;
	double delta_x = 0.0001;
	double temp_y;

	//Sampling every delta_sine[m] on the sine curve
	for (double temp_x = ret[0][0]; i < ret[0].size(); temp_x = temp_x + delta_x)
	{
		temp_y = csetting.ampl * sin(2 * M_PI * temp_x / csetting.cycle);
		if (pow(ret[0][i - 1] - temp_x, 2.0) + pow(ret[1][i - 1] - temp_y, 2.0) >= pow(csetting.delta_sine, 2.0))
		{
			ret[0][i] = temp_x;
			ret[1][i] = temp_y;
			i++;
		}
	}
}

inline std::vector<std::vector<double>> GenCourse::Gen_Course_csv(std::string path)
{
	RTCLib::CSVLoader csv(path, 1);
	for (size_t i = 0; i < course[0].size(); i++)
	{
		course[0][i] = csv[i][0];
		course[1][i] = csv[i][1];
		course[4][i] = csetting.v_min_const;
		course[5][i] = csetting.v_max_const;
	}
	return course;
}