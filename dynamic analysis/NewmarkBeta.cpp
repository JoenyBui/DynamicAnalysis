#include "StdAfx.h"
#include "NewmarkBeta.h"

using namespace std;

///<summary>Initialize the Numerical Method Scheme.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NewmarkBeta::Initialize(double Gamma, double Beta)
{
	MyGamma = Gamma;
	MyBeta = Beta;
};

NewmarkBeta::NewmarkBeta(double Gamma, double Beta)
{
	this -> Initialize(Gamma, Beta);
};

NewmarkBeta::NewmarkBeta(void)
{
	// Standard Average Acceleration Constants.
	this -> Initialize(0.5, 0.25);
}

NewmarkBeta::~NewmarkBeta(void)
{
}

///<summary></summary>
///<param name="Force">Force at time = 0.</param>
///<param name="MassFactor">Total Mass of system.</param>
///<param name="DampingFactor">Damping of system.</param>
///<param name="StiffnessFactor">Stiffness of system.</param>
///<param name="InitialVelocity">Velocity at time = 0.</param>
///<param name="InitialDisplacement">Displacement at time =0.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double NewmarkBeta::GetInitialAcceleration(double Force, 
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

///<summary></summary>
///<param name="Force">Force at time = 0.</param>
///<param name="MassFactor">Total Mass of system.</param>
///<param name="DampingFactor">Damping of system.</param>
///<param name="ResistanceForce">Resistance force of the system.</param>
///<param name="InitialVelocity">Velocity at time = 0.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double NewmarkBeta::GetInitialAcceleration(double Force, 
										   double MassFactor, 
										   double DampingFactor, 
										   double ResistanceForce, 
										   double InitialVelocity)
{
	// Numerator = Po - c * dU - R_0
	double Numerator = Force - DampingFactor * InitialVelocity - ResistanceForce;

	// Denominator = m
	double Denominator = MassFactor;

	// Return value.
	return Numerator / Denominator;
};

///<summary></summary>
///<param name="MassFactor"></param>
///<param name="DampingFactor"></param>
///<param name="StiffnessFactor"></param>
///<param name="TimeStep"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double NewmarkBeta::GetEffectiveStiffness(double MassFactor, 
										  double DampingFactor, 
										  double StiffnessFactor, 
										  double TimeStep)
{
	// A + B + C

	// A = k
	double A = StiffnessFactor;

	// gam / (beta * dT) * c
	double B = MyGamma / (MyBeta * TimeStep) * DampingFactor;

	// m / (b * dT^2)
	double C = MassFactor / (MyBeta * std::pow(TimeStep, 2));

	return A + B + C;
};

///<summary></summary>
///<param name="MassFactor"></param>
///<param name="DampingFactor"></param>
///<param name="StiffnessFactor"></param>
///<param name="IncrementalForce"></param>
///<param name="Acceleration"></param>
///<param name="Velocity"></param>
///<param name="TimeStep"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double NewmarkBeta::GetEffectiveIncrementalForce(double MassFactor, 
												 double DampingFactor, 
												 double IncrementalForce, 
												 double Acceleration, 
												 double Velocity, 
												 double TimeStep)
{
	// A = m / (beta * dT) + gamma / beta * c
	double A = 1 * MassFactor / (MyBeta * TimeStep) + (MyGamma / MyBeta) * DampingFactor;

	// B = 1 / (2 * beta) * m + dT *(gamma / (2 * beta) - 1) * c
	double B = MassFactor / (2 * MyBeta) + TimeStep * (MyGamma / (2*MyBeta) - 1) * DampingFactor;

	// dP_eff = dP + A * d(u) + B * d^2(u)
	return IncrementalForce + A * Velocity + B * Acceleration;
};

///<summary></summary>
///<param name="MassFactor"></param>
///<param name="DampingFactor"></param>
///<param name="StiffnessFactor"></param>
///<param name="IncrementalForce"></param>
///<param name="Acceleration"></param>
///<param name="Velocity"></param>
///<param name="TimeStep"></param>
///<returns>0-Displacement Increment, 1-Velocity Increment, 2-Acceleration Increment</returns>
///<remarks>description</remarks>
vector <double> NewmarkBeta::Solve_Linear(double MassFactor, 
										  double DampingFactor, 
										  double StiffnessFactor, 
										  double IncrementalForce, 
										  double Acceleration, 
										  double Velocity, 
										  double TimeStep)
{
	// dP_eff = dP + A * d(U) + B * d^2(U)
	double DeltaLoadEffective = this -> GetEffectiveIncrementalForce(MassFactor, 
																	 DampingFactor, 
																	 IncrementalForce, 
																	 Acceleration, 
																	 Velocity, 
																	 TimeStep); 

	// K_eff
	double EffectiveStiffness = this -> GetEffectiveStiffness(MassFactor, 
															  DampingFactor, 
															  StiffnessFactor, 
															  TimeStep);

	// Delta Displacement = dP_eff / K_eff
	double dDisplacement = DeltaLoadEffective / EffectiveStiffness;

	// Delta Velocity = A + B + C
	double A = MyGamma / (MyBeta * TimeStep) * dDisplacement;

	double B = -MyGamma / MyBeta * Velocity;

	double C = TimeStep * (1 - MyGamma / (2 * MyBeta)) * Acceleration;

	
	double dVelocity = A + B + C;

	// Delta Acceleration.
	double D = 1 / (MyBeta * std::pow(TimeStep, 2)) * dDisplacement;

	double E = - 1 / (MyBeta * TimeStep) * Velocity;

	double F = - 1 / (2 * MyBeta) * Acceleration;

	double dAcceleration = D + E + F;

	vector <double> Results;

	// Store the Results in a Vector.
	// 0 - Displacement
	// 1 - Velocity
	// 2 - Acceleration
	Results.push_back(dDisplacement);
	Results.push_back(dVelocity);
	Results.push_back(dAcceleration);

	return Results;
};

///<summary></summary>
///<param name="MassFactor"></param>
///<param name="DampingFactor"></param>
///<param name="StiffnessFactor"></param>
///<param name="IncrementalForce"></param>
///<param name="RestoringForce"></param>
///<param name="Acceleration"></param>
///<param name="Velocity"></param>
///<param name="TimeStep"></param>
///<returns>0-Displacement Increment, 1-Velocity Increment, 2-Acceleration Increment</returns>
///<remarks>description</remarks>
vector <double> NewmarkBeta::Solve_NonLinear(double MassFactor, 
											 double DampingFactor, 
											 double TangentStiffnessFactor, 
											 double Force,
											 double IncrementalForce, 
											 double RestoringForce, 
											 double Displacement,
											 double Acceleration, 
											 double Velocity, 
											 double TimeStep)
{
	// Next Force
	double NextForce = Force + IncrementalForce;

	// dP_eff = dP + A * d(U) + B * d^2(U)
	double DeltaLoadEffective = this -> GetEffectiveIncrementalForce(MassFactor, 
																	 DampingFactor, 
																	 IncrementalForce, 
																	 Acceleration, 
																	 Velocity, 
																	 TimeStep); 

	// K_eff
	double EffectiveStiffness = this -> GetEffectiveStiffness(MassFactor, 
															  DampingFactor, 
															  TangentStiffnessFactor, 
															  TimeStep);

	// Delta Displacement = dP_eff / K_eff
	double dDisplacement = DeltaLoadEffective / EffectiveStiffness;

	// Delta Velocity = A + B + C
	double A = MyGamma / (MyBeta * TimeStep) * dDisplacement;

	double B = -MyGamma / MyBeta * Velocity;

	double C = TimeStep * (1 - MyGamma / (2 * MyBeta)) * Acceleration;

	double dVelocity = A + B + C;

	// Delta Acceleration.
	double NextTimeStepRestoringForce = RestoringForce + TangentStiffnessFactor * dDisplacement;

	if (NextTimeStepRestoringForce < -388623)
	{
		NextTimeStepRestoringForce = -388623;
	}
	else if(NextTimeStepRestoringForce > 76773.6)
	{
		NextTimeStepRestoringForce = 76773.6;
	};

	double NextTimeStepAcceleration = ((Force + IncrementalForce) - DampingFactor * (Velocity + dVelocity) - NextTimeStepRestoringForce) / MassFactor;

	double dAcceleration = NextTimeStepAcceleration - Acceleration;

	// Store the Results
	vector <double> Results;

	// Store the Results in a Vector.
	// 0 - Force
	// 1 - IncrementForce
	// 2 - ResotringForce
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
	Results.push_back(TangentStiffnessFactor);
	Results.push_back(Displacement);
	Results.push_back(Velocity);
	Results.push_back(Acceleration);
	Results.push_back(dDisplacement);
	Results.push_back(dVelocity);
	Results.push_back(dAcceleration);
	Results.push_back(DeltaLoadEffective);
	Results.push_back(EffectiveStiffness);

	return Results;
};

///<summary></summary>
///<param name="MassFactor"></param>
///<param name="DampingFactor"></param>
///<param name="StiffnessFactor"></param>
///<param name="IncrementalForce"></param>
///<param name="RestoringForce"></param>
///<param name="Acceleration"></param>
///<param name="Velocity"></param>
///<param name="TimeStep"></param>
///<returns>0-Displacement Increment, 1-Velocity Increment, 2-Acceleration Increment</returns>
///<remarks>description</remarks>
vector <double> NewmarkBeta::Solve_NonLinear(GeneralizedSDoF2D &SDOF,
											 double MassFactor, 
											 double DampingFactor, 
											 double TangentStiffnessFactor, 
											 double Force,
											 double IncrementalForce, 
											 double RestoringForce, 
											 double Displacement,
											 double Acceleration, 
											 double Velocity, 
											 double TimeStep)
{
	// Next Force
	double NextForce = Force + IncrementalForce;

	// dP_eff = dP + A * d(U) + B * d^2(U)
	double DeltaLoadEffective = this -> GetEffectiveIncrementalForce(MassFactor, 
																	 DampingFactor, 
																	 IncrementalForce, 
																	 Acceleration, 
																	 Velocity, 
																	 TimeStep); 

	// K_eff
	double EffectiveStiffness = this -> GetEffectiveStiffness(MassFactor, 
															  DampingFactor, 
															  TangentStiffnessFactor, 
															  TimeStep);

	// A = m / (beta * dT) + gamma / beta * c
	double TangentConstant = (1 * MassFactor / (MyBeta * TimeStep) + (MyGamma / MyBeta) * DampingFactor) / TimeStep;

	// Delta Displacement = dP_eff / K_eff
	double dDisplacement = this -> GetModifiedNewtonRhapsonDisplacement(SDOF,
																		Displacement, 
																		Velocity,
																		RestoringForce, 
																		DeltaLoadEffective, 
																		EffectiveStiffness, 
																		TangentConstant,
																		0.1);
	//double dDisplacement = DeltaLoadEffective / EffectiveStiffness;

	// Delta Velocity = A + B + C
	double A = MyGamma / (MyBeta * TimeStep) * dDisplacement;

	double B = -MyGamma / MyBeta * Velocity;

	double C = TimeStep * (1 - MyGamma / (2 * MyBeta)) * Acceleration;

	double dVelocity = A + B + C;

	// Delta Acceleration.
	double D = 1 / (MyBeta * std::pow(TimeStep, 2)) * dDisplacement;

	double E = - 1 / (MyBeta * TimeStep) * Velocity;

	double F = - 1 / (2 * MyBeta) * Acceleration;

	double dAcceleration = D + E + F;

	// Store the Results
	vector <double> Results;

	// Store the Results in a Vector.
	// 0 - Force
	// 1 - IncrementForce
	// 2 - ResotringForce
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
	Results.push_back(TangentStiffnessFactor);
	Results.push_back(Displacement);
	Results.push_back(Velocity);
	Results.push_back(Acceleration);
	Results.push_back(dDisplacement);
	Results.push_back(dVelocity);
	Results.push_back(dAcceleration);
	Results.push_back(DeltaLoadEffective);
	Results.push_back(EffectiveStiffness);

	return Results;
};

double NewmarkBeta::GetModifiedNewtonRhapsonDisplacement(GeneralizedSDoF2D &SDOF,
														 double Displacement, 
														 double Velocity, 
														 double InitialRestoringForce,
														 double EffectiveForce, 
														 double EffectiveStffness,
														 double TangentConstant,
														 double Tolerance)
{
	// Declare the return variable.
	double NextDisplacement = Displacement;

	double DeltaDisplacement = 0;
	double DeltaRestoringForce = 0;

	double ResidualForce = EffectiveForce;

	while (abs(ResidualForce) > Tolerance)
	{
		// Solve for the Delta Displacement.
		DeltaDisplacement = ResidualForce / EffectiveStffness;

		// Get the Next Force.
		//double Force = SDOF.
		// Get Mapping Values;
		vector <double> MapValues = SDOF.GetMappingValues(NextDisplacement, 
														  DeltaDisplacement,
														  Velocity);

		// Set the Values.
		double RestoringForce = MapValues[0];

		// Set the Next Displacement;
		NextDisplacement += DeltaDisplacement;

		// Find the Delta Restoring Force.
		DeltaRestoringForce = RestoringForce - InitialRestoringForce + TangentConstant * NextDisplacement;

		// Change the Restoing Force.
		InitialRestoringForce = RestoringForce;

		// Find the Residual Force.
		ResidualForce = ResidualForce - DeltaRestoringForce;
	};

	return NextDisplacement - DeltaDisplacement;
};
