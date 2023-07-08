#ifndef MICSFUNCTIONS_H
#define MISCFUNCTIONS_H

#include <stdlib.h>
#include <random>
#include <time.h>
#include <map>

std::default_random_engine & RandEngine();

float RandFloat();

void SeedRand(unsigned long seed);

#endif



