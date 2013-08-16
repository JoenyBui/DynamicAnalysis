#include "StdAfx.h"
#include "CentralDifferenceMethod.h"

CentralDifferenceMethod::CentralDifferenceMethod(void)
{
}

CentralDifferenceMethod::~CentralDifferenceMethod(void)
{
}

double CentralDifferenceMethod::GetInitialAcceleration(double Force, 
													   double MassFactor, 
													   double DampingFactor, 
													   double StiffnessFactor, 
													   double InitialVelocity, 
													   double InitialDisplacement)
{
	// Numerator = Po - c * dU - k * U
	double Numerator = Force - DampingFactor * InitialVelocity - StiffnessFactor * InitialDisplacement;

	// Denominator = m
	double Denominator = MassFactor;

	// Return value.
	return Numerator / Denominator;
};

double CentralDifferenceMethod::GetInitialDisplacement(double InitialDisplacement,
													   double InitialVelocity,
													   double InitialAcceleration,
													   double TimeStep)
{
	double A = -TimeStep * InitialVelocity;

	double B = std::pow(TimeStep, 2) * InitialAcceleration / 2;

	return InitialDisplacement + A + B;
};

double CentralDifferenceMethod::GetEffectiveStiffness(double MassFactor, 
													  double DampingFactor, 
													  double StiffnessFactor, 
													  double TimeStep)
{
	// m / (b * dT^2)
	double A = MassFactor / (std::pow(TimeStep, 2));

	// gam / (beta * dT) * c
	double B = DampingFactor / (2 * TimeStep);

	return A + B;
};

double CentralDifferenceMethod::GetEffectiveIncrementalForce(double MassFactor, 
															 double DampingFactor, 
															 double StiffnessFactor,
															 double Force, 
															 double PreviousDisplacement,
															 double CurrentDisplacement,
															 double TimeStep)
{
	double A = MassFactor / (std::pow(TimeStep, 2)) - DampingFactor / (2 * TimeStep);

	double B =  StiffnessFactor - (2 * MassFactor) / (std::pow(TimeStep, 2));

	return Force - A * PreviousDisplacement - B * CurrentDisplacement;
};

double CentralDifferenceMethod::GetEffectiveIncrementalForce(double MassFactor, 
															 double DampingFactor, 
															 double StiffnessFactor,
															 double RestoringForce,
															 double Force, 
															 double PreviousDisplacement,
															 double CurrentDisplacement,
															 double TimeStep)
{
	double A = MassFactor / (std::pow(TimeStep, 2)) - DampingFactor / (2 * TimeStep);

	double B =  - (2 * MassFactor) / (std::pow(TimeStep, 2));

	return Force - A * PreviousDisplacement - B * CurrentDisplacement - RestoringForce;
};

vector <double> CentralDifferenceMethod::Solve_Linear(double MassFactor, 
													  double DampingFactor, 
													  double StiffnessFactor, 
													  double Force,
													  double IncrementalForce, 
													  double PreviousDisplacement,
													  double CurrentDisplacement,
													  double Acceleration, 
													  double Velocity, 
													  double TimeStep)
{
	double EffectiveForce = this -> GetEffectiveIncrementalForce(MassFactor, 
																 DampingFactor, 
																 StiffnessFactor, 
																 Force,
																 PreviousDisplacement,
																 CurrentDisplacement,
																 TimeStep);

	double EffectiveStiffness = this -> GetEffectiveStiffness(MassFactor, 
															  DampingFactor, 
															  StiffnessFactor, 
															  TimeStep);

	double NextDisplacement = EffectiveForce / EffectiveStiffness;

	double NextVelocity = (NextDisplacement - PreviousDisplacement) / (2 * TimeStep);

	double NextAcceleration = (NextDisplacement - 2 * CurrentDisplacement + PreviousDisplacement) / std::pow(TimeStep, 2);

	vector <double> Results;

	// Store the Results in a Vector.
	// 0 - Force
	// 1 - IncrementForce
	// 2 - RestoringForce
	// 3 - Tangent Stiffness Factor
	// 4 - Displacement
	// 5 - Velocity
	// 6 - Acceleration
	// 7 - Increment Displacement
	// 8 - Increment Velocity
	// 9 - Increment Acceleration
	// 10 - Effective Load.
	// 11 - Effective Stiffness
	Results.push_back(Force);
	Results.push_back(IncrementalForce);
	Results.push_back(-1.0);
	Results.push_back(StiffnessFactor);
	Results.push_back(CurrentDisplacement);
	Results.push_back(Velocity);
	Results.push_back(Acceleration);
	Results.push_back(NextDisplacement - CurrentDisplacement);
	Results.push_back(NextVelocity - Velocity);
	Results.push_back(NextAcceleration - Acceleration);
	Results.push_back(EffectiveForce);
	Results.push_back(EffectiveStiffness);

	return Results;
};

vector <double> CentralDifferenceMethod::Solve_NonLinear(double MassFactor, 
													  double DampingFactor, 
													  double StiffnessFactor, 
													  double RestoringForce,
													  double Force,
													  double IncrementalForce, 
													  double PreviousDisplacement,
													  double CurrentDisplacement,
													  double Acceleration, 
													  double Velocity, 
													  double TimeStep)
{
	double EffectiveForce = this -> GetEffectiveIncrementalForce(MassFactor, 
																 DampingFactor, 
																 StiffnessFactor, 
																 RestoringForce,
																 Force,
																 PreviousDisplacement,
																 CurrentDisplacement,
																 TimeStep);

	double EffectiveStiffness = this -> GetEffectiveStiffness(MassFactor, 
															  DampingFactor, 
															  StiffnessFactor, 
															  TimeStep);

	double NextDisplacement = EffectiveForce / EffectiveStiffness;

	double NextVelocity = (NextDisplacement - PreviousDisplacement) / (2 * TimeStep);

	double NextAcceleration = (NextDisplacement - 2 * CurrentDisplacement + PreviousDisplacement) / std::pow(TimeStep, 2);

	vector <double> Results;

	// Store the Results in a Vector.
	// 0 - Force
	// 1 - IncrementForce
	// 2 - RestoringForce
	// 3 - Tangent Stiffness Factor
	// 4 - Displacement
	// 5 - Velocity
	// 6 - Acceleration
	// 7 - Increment Displacement
	// 8 - Increment Velocity
	// 9 - Increment Acceleration
	// 10 - Effective Load.
	// 11 - Effective Stiffness
	Results.push_back(Force);
	Results.push_back(IncrementalForce);
	Results.push_back(RestoringForce);
	Results.push_back(StiffnessFactor);
	Results.push_back(CurrentDisplacement);
	Results.push_back(Velocity);
	Results.push_back(Acceleration);
	Results.push_back(NextDisplacement - CurrentDisplacement);
	Results.push_back(NextVelocity - Velocity);
	Results.push_back(NextAcceleration - Acceleration);
	Results.push_back(EffectiveForce);
	Results.push_back(EffectiveStiffness);

	return Results;
};