#include "MiscFunctions.h"

std::default_random_engine & RandEngine()
{
	static std::default_random_engine e;

	return e;
}

float RandFloat()
{
	static std::uniform_real_distribution<float> d(0, 1);

	return d(RandEngine());
}

//void SeedRand() {RandEngine().seed(500);}

//void SeedRand() {RandEngine().seed(time(NULL));}

void SeedRand(unsigned long seed) {RandEngine().seed(seed);}