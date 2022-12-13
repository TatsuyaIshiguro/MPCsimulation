#pragma once
#include <random>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>

class random_num
{
public:
	int Make_num();
    double num_range(double min, double max);
private:

};

inline int random_num::Make_num()
{
	std::srand(time(NULL));
	return rand();
}

inline double random_num::num_range(double min, double max)
{
    boost::random::mt19937 gen;
    boost::random::uniform_real_distribution<> dist(min, max);
    
    return dist(gen);
}


