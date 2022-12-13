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
<<<<<<< HEAD
}

<<<<<<< HEAD
inline double random_num::num_range(double min, double max)
{
    boost::random::mt19937 gen;
    boost::random::uniform_real_distribution<> dist(min, max);
    
    return dist(gen);
}

=======

>>>>>>> 2942353e150a707a5a840466743353603b86a711

=======
}
>>>>>>> parent of 434d816 (commit)
