// SphTest.cpp: 定义控制台应用程序的入口点。
//


#include <Plane3.h>
#include <SphSystemSolver3.h>
using namespace CalfFluidEngine;
int main()
{
	SphSystemSolver3 solver;
	solver.SetViscosityCoefficient(0.1);
	solver.SetPseudoViscosityCoefficient(10);
	return 0;
}

