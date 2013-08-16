#pragma once

#ifndef CentralDifferenceMethod_H
#define CentralDifferenceMethod_H

#include "StdAfx.h"

using namespace std;

class CentralDifferenceMethod
{
	private:

	protected:

	public:
		static const int ID = 1;

		CentralDifferenceMethod(void);

		~CentralDifferenceMethod(void);

		double GetInitialAcceleration(double Force,
									  double MassFactor,
									  double DampingFactor,
									  double StiffnessFactor,
									  double InitialVelocity,
									  double InitialDisplacement);

		double GetInitialDisplacement(double InitialDisplacement,
									  double InitialVelocity,
									  double InitialAcceleration,
									  double TimeStep);

		double GetEffectiveStiffness(double MassFactor,
									 double DampingFactor,
									 double StiffnessFactor,
									 double TimeStep);


		double GetEffectiveIncrementalForce(double MassFactor,
											double DampingFactor,
											double StiffnessFactor,
											double Force,
											double PreviousDisplacement,
											double CurrentDisplacement,
											double TimeStep);

		double GetEffectiveIncrementalForce(double MassFactor,
											double DampingFactor,
											double StiffnessFactor,
											double RestoringForce,
											double Force,
											double PreviousDisplacement,
											double CurrentDisplacement,
											double TimeStep);

		vector <double> Solve_Linear(double MassFactor,
									 double DampingFactor,
									 double StiffnessFactor,
									 double Force,
									 double IncrementalForce,
									 double PreviousDisplacement,
									 double CurrentDisplacement,
									 double Acceleration,
									 double Velocity,
									 double TimeStep);

		vector <double> Solve_NonLinear(double MassFactor,
										double DampingFactor,
										double StiffnessFactor,
										double RestoringForce,
										double Force,
										double IncrementalForce,
										double PreviousDisplacement,
										double CurrentDisplacement,
										double Acceleration,
										double Velocity,
										double TimeStep);
};
#endif