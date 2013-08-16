#pragma once

#ifndef GeneralizedSDoF2D_H
#define GeneralizedSDoF2D_H

#include "StdAfx.h"
#include "NonLinearStatic2D.h"

#include "CentralDifferenceMethod.h"
#include "NewmarkBeta.h"
#include "TextFileIO.h"

class GeneralizedSDoF2D
{
	private:
		
		
	protected:
		string MyPath;

		TextFileIO MyTextFile;

		void Clone(const GeneralizedSDoF2D &SDOF);

		NonLinearStatic2D MyNLStaticAnalysis;
		
		double MyGravity;

		// Store the Response History.
		vector <vector<double>> MyResponseHistory;

		// Neutral Orgin is changed everytime the load goes inelastic.
		double MyNeutralOrigin;

		int MyPositiveDirection;

		// Stored the Positive and Negative Resistance Function
		// Positive/Negative is with respect to the Loading.
		vector <vector<double>> MyPositiveResistanceFunction;
		vector <vector<double>> MyNegativeResistanceFunction;

		// Store the Equilvalent Load Mass Factor.
		vector <vector<double>> MyLoadMassFactorPositive;
		vector <vector<double>> MyLoadMassFactorNegative;

		// The vector is a general force function, if the inherited class does not inherit
		// the GetForceFunction(double time) then the force function will default to the 
		// vector to calculate the force.
		vector <vector<double>> MyGeneralForceFunctionTable;

		// The Sequence of Mapping the Resistance Function is broking up into
		// loading and unloading sequence.  When loading, the algorithm is used to
		// assuming the nonlinear static analysis.  Unloading assumes a linear 
		// rebound according to the initial stiffness.
		/*				LOADING
							|        ----------
							|     -
							|   -
							| /
							|/
		<------------------------------------------->
						   /|
			--------------	|
							|
							|
							|
							|
		*/

		/*				UNLOADING
							|    /
							|   /
							|  /
							| /
							|/
		<------------------------------------------->
						   /|
						  /	|
						 /	|
						/	|
					   /	|
							|
		*/
	public:
		static const int TranslationX = 1;
		static const int TranslationY = 2;
		static const int RotationZ = 3;

		// Default Constructors.
		GeneralizedSDoF2D(void);

		GeneralizedSDoF2D(vector <vector<double>> PositiveResistanceFunction, 
						  vector <vector<double>> NegativeResistanceFunction,
						  vector <vector<double>> PositiveLoadMassFactor, 
						  vector <vector<double>> NegativeLoadMassFactor,
						  vector <vector<double>> GeneralForceFunctionTable, 
						  string Path);

		// Destructor.
		~GeneralizedSDoF2D(void);

		virtual void RunSDoF(int DOF);

		void SetEvent(NonLinearStatic2D Event);
		void SetFrameGeometry(ElasticFrame2D Frame);
		void AddStaticLoad();
		void AddDynamicLoad();

		double GetMassFactor(int DOF, int LoadStep);

		double GetLoadFactor(int DOF, int LoadStep);

		double GetImpulse(const vector<vector<double>> &PT, 
						  double UnitConversion);

		vector <double> GetMappingValues(double Displacement, 
										 double DisplacementIncrement,
										 double Velocity);

		virtual double GetResistanceInitialStiffness(double Displacement);
		virtual double GetResistanceFunctionStiffness(double Displacement);
		virtual double GetResistanceFunctionLoading(double Displacement);
		virtual double GetResistanceFunctionUnloading(double Displacement);

		virtual double GetLoadMassFactorLoading(double Displacement);
		virtual double GetLoadMassFactorUnloading(double Displacement);

		/// <summary> The forcing function must be override by the inherited class </summary>
		virtual double GetForceFunction(double time);

		void RunCentralDifferenceMethod(double TimeStep,
										double MaxTime,
										double InitialDisplacement,
										double InitialVelocity,
										double MassFactor,
										double DampingFactor);

		void RunNewmarkBeta(double TimeStep, 
						    double MaxTime, 
							double InitialDisplacement = 0.0, 
							double InitialVelocity = 0.0,
							double MassFactor = 1.0,
							double DampingFactor = 0.0);

		void SetGravity(double Gravity);
};

#endif