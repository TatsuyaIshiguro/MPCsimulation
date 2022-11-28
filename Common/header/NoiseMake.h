#pragma once
#include <random>
#include <stdlib.h>
#include <time.h>

class Noise
{
public:
	// ノイズ作成
	void Make();
	double noise_u, noise_v, noise_theta;

private:
	//obseravation noise
	double observationVariance_u = 0.0;
	double observationVariance_v = 0.0;
	double observationVariance_theta = 0.0;

	//double observationVariance_u = 0.2;
	//double observationVariance_v = 0.2;
	//double observationVariance_theta = 0.05;
};

inline void Noise::Make()
{
	srand(time(NULL));
	double num1, num2, num3;
	// 0~1の一様乱数生成
	num1 = (double)rand() / (double)RAND_MAX;
	num2 = (double)rand() / (double)RAND_MAX;
	num3 = (double)rand() / (double)RAND_MAX;

	while (num1 == 0 || num2 == 0 || num3 == 0)
	{
		num1 = (double)rand() / (double)RAND_MAX;
		num2 = (double)rand() / (double)RAND_MAX;
		num3 = (double)rand() / (double)RAND_MAX;
	}

	// ボックスミュラー法
	noise_u = observationVariance_u * sqrt(-2.0 * log(num1)) * cos(2 * std::_Pi * num2);
	noise_v = observationVariance_v * sqrt(-2.0 * log(num2)) * cos(2 * std::_Pi * num3);
	noise_theta = observationVariance_theta * sqrt(-2.0 * log(num3)) * sin(2 * std::_Pi * num1);

	if (std::isnan(noise_u) || std::isnan(noise_v) || std::isnan(noise_theta))
	{
		std::cout << "nan" << std::endl;
	}
}