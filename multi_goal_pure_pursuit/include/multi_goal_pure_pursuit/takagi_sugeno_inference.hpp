/*
 * takagi_sugeno_inference.hpp
 *
 *  Created on: Nov 12, 2019
 *      Author: kyberszittya
 */

#ifndef INCLUDE_MULTI_GOAL_PURE_PURSUIT_TAKAGI_SUGENO_INFERENCE_HPP_
#define INCLUDE_MULTI_GOAL_PURE_PURSUIT_TAKAGI_SUGENO_INFERENCE_HPP_

#include <cmath>
#include <vector>

class BellMembershipFunction
{
private:
	double center;
	double a;
	double b;
public:
	BellMembershipFunction(double center, double a, double b):
		center(center), a(a), b(b){}

	double eval(double x)
	{
		return 1/std::pow(1+std::abs((x-center)/a), 2*b);
	}
};

class FuzzyInferenceTakagiMacVicar
{
private:
	std::vector<BellMembershipFunction> ruleset;
public:
	FuzzyInferenceTakagiMacVicar()
	{

	}

	void initRuleset()
	{


	}
};


#endif /* INCLUDE_MULTI_GOAL_PURE_PURSUIT_TAKAGI_SUGENO_INFERENCE_HPP_ */
