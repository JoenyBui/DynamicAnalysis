#pragma once

#ifndef NewmarkBeta_H
#define NewmarkBeta_H

#include "StdAfx.h"
#include "GeneralizedSDoF2D.h"

/*
	"Dynamics of Structures Theory and Applications to Earthquake Engineering"
	3rd Edition
	Anil K. Chopra
	page 210-213
*/

using namespace std;

class GeneralizedSDoF2D;

class NewmarkBeta
{
	private:
		double MyGamma;
		double MyBeta;

	protected:
		void Initialize(double Gamma, double Beta);

		void Clone(double Gamma, double Beta);

	public:
		static const int ID = 0;

		NewmarkBeta(double Gamma, double Beta);

		NewmarkBeta(void);
		
		~NewmarkBeta(void);

		double GetInitialAcceleration(double Force,
									  double MassFactor,
									  double DampingFactor,
									  double StiffnessFactor,
									  double InitialVelocity,
									  double InitialDisplacement);

		double GetInitialAcceleration(double Force,
									  double MassFactor,
									  double DampingFactor,
									  double ResistanceForce,
									  double InitialVelocity);

		double GetEffectiveStiffness(double MassFactor,
									 double DampingFactor,
									 double StiffnessFactor,
									 double TimeStep);

		double GetEffectiveIncrementalForce(double MassFactor,
											double DampingFactor,
											double IncrementalForce,
											double Acceleration,
											double Velocity,
											double TimeStep);

		double GetModifiedNewtonRhapsonDisplacement(GeneralizedSDoF2D &SDOF,
													double Displacement,
													double Velocity,
													double InitialRestoringForce,
													double EffectiveForce,
													double EffectiveStffness,
													double TangentConstant,
													double Tolerance);

		///<summary>Solve for the Linear System.</summary>
		///<param name="MassFactor"></param>
		///<param name="DampingFactor"></param>
		///<param name="StiffnessFactor"></param>
		///<param name="IncrementalForce"></param>
		///<param name="Acceleration"></param>
		///<param name="Velocity"></param>
		///<param name="TimeStep"></param>
		///<returns>description</returns>
		///<remarks>description</remarks>
		vector <double> Solve_Linear(double MassFactor,
							  double DampingFactor,
							  double StiffnessFactor,
							  double IncrementalForce,
							  double Acceleration,
							  double Velocity,
							  double TimeStep);

		///<summary>Solve for the Non-Linear System.</summary>
		///<param name="MassFactor"></param>
		///<param name="DampingFactor"></param>
		///<param name="TangentStiffnessFactor"></param>
		///<param name="IncrementalForce"></param>
		///<param name="RestoringForce"></param>
		///<param name="Acceleration"></param>
		///<param name="Velocity"></param>
		///<param name="TimeStep"></param>
		///<returns>description</returns>
		///<remarks>description</remarks>
		vector <double> Solve_NonLinear(GeneralizedSDoF2D &SDOF,
										double MassFactor,
										double DampingFactor,
										double TangentStiffnessFactor,
										double Force,
										double IncrementalForce,
										double RestoringForce,
										double Displacement,
										double Acceleration,
										double Velocity,
										double TimeStep);

		vector <double> Solve_NonLinear(double MassFactor,
										double DampingFactor,
										double TangentStiffnessFactor,
										double Force,
										double IncrementalForce,
										double RestoringForce,
										double Displacement,
										double Acceleration,
										double Velocity,
										double TimeStep);
};

#endif
