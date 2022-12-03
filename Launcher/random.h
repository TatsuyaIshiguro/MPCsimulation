#pragma once
#include <random>
#include <stdlib.h>
#include <time.h>

class random_num
{
public:
	int Make_num();

private:

};

inline int random_num::Make_num()
{
	std::srand(time(NULL));
	return rand();
}