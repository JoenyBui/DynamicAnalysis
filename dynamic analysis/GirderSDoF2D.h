#pragma once

#ifndef GirderSDoF2D_H
#define GirderSDoF2D_H

#include "stdafx.h"
#include "GeneralizedSDoF2D.h"
#include "PyramidLoad2D.h"
#include "GirderSection2D.h"

using namespace std;
using namespace arma;

class GirderSDoF2D : public GeneralizedSDoF2D
{
	private:
		void Initialize();
		void Clone(const GirderSDoF2D &SDOF);

		// Counters.
		bool IsGeometrySet;
		bool IsLoadSet;

	protected:
		int MyID;

		double MyLength;
		double MyWidth;
		int MyMeshIncrement;
		
		Section2D MySection;
		GirderSection2D MyGirderSection;
		ElasticFrame2D MyGirder;

		vector <vector<double>> MyPressureAtCG;
		vector <vector<double>> MyPressureAtLeft;
		vector <vector<double>> MyPressureAtRight;

		double GetIncrementalReferenceLoad(double ScaleLoad);

		// Store the Specific Impulse At CG.
		double MySpecificImpulseAtCG;
		double MySpecificImpulseAtLeft;
		double MySpecificImpulseAtRight;
	
		// Storage for the Force Vector.
		double MyPeakReflectedPressureAtCG;
		double MyPeakReflectedPressureAtLeft;
		double MyPeakReflectedPressureAtRight;

		double MyExplosiveCGx;
		double MyExplosiveCGz;

		double MyImpulseDirectionFactor;
		double MyImpulseFactorAtLeft;
		double MyImpulseFactorAtRight;

		vector <double> MyNodalSpecificImpulse;
		vector <double> MyNodalPeakPressure;

		double MyEquivalentPeakPressure;
		double MyEquivalentUniformSpecificImpulse;
		double MyEquivalentTimeConstant;
		double MyForceFunctionMaxMagnitude;

		void SetGirderFrame();

		// Get the Pressure Time History from BEL Location.
		void SetBridgeExplosiveLoad(double CGx,
									double CGz,
								    string Path);

		// Dead Load.
		void SetSelfWeight();
		void SetInitialStrandForces(double InitialForce, double InitialMoment);

#pragma region "BLAST EXPLOSIVE LOAD"
		// The Explosive Load.
		void SetSpecificImpulseAndPressureVector();

		void SetIncrementBlastLoad(double ScaleIncrementLoad = 1.0);

		double GetImpulseAtLocation(double X);
		
		vector <double> GetImpulseVector();
		//vector <double> GetSpecificImpulseVector();

		double GetEquivalentUniformSpecificImpulse(int DOF, int LoadStep);
		double GetEquivalentPeakPressure(int DOF, int LoadStep);
		double GetPeakForce();

		double GetPressureAtLocation(double X);

		void SetForcingFunction();

		double GetForceFunction(double time);

		double GetPeakPressure(vector<vector<double>> &PressureTimeHistory);
#pragma endregion

	public:
		// Constructor.
		GirderSDoF2D(int ID, 
					 double Length, 
					 double CGx,
					 double CGz, 
					 int MeshIncrement, 
					 double Gravity,
					 char* Path, 
					 Section2D Section);

		GirderSDoF2D(int ID, 
					 double Length, 
					 double CGx,
					 double CGz, 
					 int MeshIncrement, 
					 double Gravity,
					 char* Path, 
					 GirderSection2D &Section);

		GirderSDoF2D(int ID, 
					 double Length, 
					 double CGx,
					 double CGz, 
					 int MeshIncrement, 
					 double Gravity,
					 char* Path, 
					 GirderSection2D *Section);

		GirderSDoF2D(void);
		
		// Destructor.
		~GirderSDoF2D(void);

		// Assignment Operator.
		GirderSDoF2D& operator= (const GirderSDoF2D &SDOF);

		// Copy Constructor.
		GirderSDoF2D(const GirderSDoF2D &SDOF);

#pragma region "SETTING GEOMETRY, DEAD LOADS, and BEL LOADS"
		void SetUniformDeadLoad(double DL);

		
#pragma endregion

#pragma region "RUN SDOF"
		static const int NewmarkBetaIndex = 0;

		void RunSDoF(double ScaleIncrementLoad, 
					 int NodeDOF, 
					 double TimeStep, 
					 double MaxTime);

		void RunNumericAnalysis(int TypeOfAnalysis, 
								double TimeStep,
								double MaxTime);

		void ModifyResistanceFunction(vector<vector<double>> &ResistanceFunction, 
									  vector<vector<double>> &LoadMassFactor);

#pragma endregion
};

#endif
