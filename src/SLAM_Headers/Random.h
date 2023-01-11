#pragma once

double random_seed = 21.0;
std::uniform_real_distribution<double> distribution(0.0, 5.0);
static std::default_random_engine generator(random_seed);