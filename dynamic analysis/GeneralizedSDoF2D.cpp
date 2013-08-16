#include "GeneralizedSDoF2D.h"

using namespace std;

GeneralizedSDoF2D::GeneralizedSDoF2D(void)
{
	MyNeutralOrigin = 0.0;
}

GeneralizedSDoF2D::GeneralizedSDoF2D(vector <vector<double>> PositiveResistanceFunction, 
									 vector <vector<double>> NegativeResistanceFunction,
									 vector <vector<double>> PositiveLoadMassFactor, 
									 vector <vector<double>> NegativeLoadMassFactor,
									 vector <vector<double>> GeneralForceFunctionTable, 
									 string Path)
{
	MyTextFile = TextFileIO(Path);

	MyNeutralOrigin = 0.0;

	MyPositiveResistanceFunction = PositiveResistanceFunction;
	MyNegativeResistanceFunction = NegativeResistanceFunction;

	MyLoadMassFactorPositive = PositiveLoadMassFactor;
	MyLoadMassFactorNegative = NegativeLoadMassFactor;

	MyGeneralForceFunctionTable = GeneralForceFunctionTable;
}

GeneralizedSDoF2D::~GeneralizedSDoF2D(void)
{
}

///<summary>Copy the Frame Geometry into the Static Analysis.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void GeneralizedSDoF2D::SetFrameGeometry(ElasticFrame2D Frame)
{
	// Add the Frame Geometry
	MyNLStaticAnalysis.SetFrameGeometry(Frame);
};

///<summary>Copy the Frame Geometry, Static Loads, and Dynamic Loads.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void GeneralizedSDoF2D::SetEvent(NonLinearStatic2D Event)
{
	// Set the Load
	MyNLStaticAnalysis = NonLinearStatic2D(Event);
};

///<summary></summary>
///<param name="DOF"></param>
///<remarks>description</remarks>
void GeneralizedSDoF2D::RunSDoF(int DOF)
{
	// Run the Static Analysis.
	MyNLStaticAnalysis.RunAnalysis(NonLinearStatic2D::Euler);

	// Load Factor.
	double K_Mass;
	double K_Load;

	switch (DOF)
	{
		case GeneralizedSDoF2D::TranslationX:
			// Get Mass Factor.
			K_Mass = this -> GetMassFactor(DOF, 1);

			// Get Load Factor.
			K_Load = this -> GetLoadFactor(DOF, 1);

			break;
		case GeneralizedSDoF2D::TranslationY:
			// Get Mass Factor.
			K_Mass = this -> GetMassFactor(DOF, 1);

			// Get Load Factor.
			K_Load = this -> GetLoadFactor(DOF, 1);

			break;
		case GeneralizedSDoF2D::RotationZ:
			// Get Mass Factor.
			K_Mass = this -> GetMassFactor(DOF, 1);

			// Get Load Factor.
			K_Load = this -> GetLoadFactor(DOF, 1);
			break;
	}
	
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
void GeneralizedSDoF2D::AddStaticLoad()
{
	
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
void GeneralizedSDoF2D::AddDynamicLoad()
{

};

///<summary></summary>
///<param name="DOF"></param>
///<param name="LoadStep"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double GeneralizedSDoF2D::GetMassFactor(int DOF, int LoadStep)
{
	// Mass Factor 
	// Use to transform the mass factor based off of what is given
	// Km = Sum(M_r * Phi_r^2) / sum(M_r)

	// Retreive the Shape Vector at an early time increment.
	vector <double> ShapeVector = MyNLStaticAnalysis.GetShapeVector(DOF, LoadStep, true);

	vector <double> MassVector = MyNLStaticAnalysis.GetMassVector(DOF, LoadStep, true);

	double Numerator = 0.0;
	double Denominator = 0.0;

	for (size_t i = 0; i < ShapeVector.size(); i++)
	{
		// Summation of the Equivalent Mass = Mass * Phi^2.
		Numerator += MassVector[i] * pow(ShapeVector[i], 2);

		// Summatin of the Total Mass
		Denominator += MassVector[i];
	};

	return Numerator / Denominator;
};

///<summary></summary>
///<param name="DOF"></param>
///<param name="LoadStep"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double GeneralizedSDoF2D::GetLoadFactor(int DOF, int LoadStep)
{
	// Load Factor 
	// Use to transform the Load into a single degree of freedom.
	// Kl = Sum(L_r * Phi_r) / sum(L_r)

	// Retreive the Shape Vector at an early time increment.
	vector <double> ShapeVector = MyNLStaticAnalysis.GetShapeVector(DOF, LoadStep, true);

	vector <double> LoadVector = MyNLStaticAnalysis.GetBeamLoadVector(DOF, LoadStep, true);

	double Numerator = 0.0;
	double Denominator = 0.0;

	for (size_t i = 0; i < ShapeVector.size(); i++)
	{
		// Summation of the Equivalent Force = Load * Phi.
		Numerator += LoadVector[i] * ShapeVector[i];

		// Summatin of the Total Force = Load
		Denominator += LoadVector[i];
	};

	return Numerator / Denominator;
};

///<summary>Return the Peak Pressure Impulse.  The area of the curve is taken until the pressure turns negative.</summary>
///<param name="PT">Pressure Time history.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double GeneralizedSDoF2D::GetImpulse(const vector <vector<double>> &PT, double UnitConversion)
{
	// Declare the Return Total Impulse.
	double TotalImpulse = 0;

	/*
		P_i
		|\
		| \
		|  \
		|   \
		|    \ P_i+1
		|    |
		|	 |
		|	 |
		|	 |
		|____|
		t_i   t_i+1

		Impulse@i = [(P_i + P_i+1) / 2] * [t_i+1 - t_i]
	*/

	int i = 0;
	bool PositivePressure = true;

	while (i < PT.size() && PositivePressure)
	{
		// Time Increment.
		double Ti = PT[i][0];
		double Ti_1 = PT[i+1][0];

		// Pressure Increment.
		double Pi = PT[i][1];
		double Pi_1 = PT[i+1][1];

		if (Pi_1 >= 0.0)
		{
			// Add to the Total Impulse.
			TotalImpulse += ((Pi + Pi_1) / 2) * (Ti_1 - Ti);
		}
		else
		{
			// Exit Early.
			PositivePressure = false;
		};
		
		// Increment the Syste.
		i++;
	};

	return TotalImpulse * UnitConversion;
};

///<summary>Return the parameters that is needed for the Newmark-Beta.  The function
/// returns the restoring force, the stiffness at the displacement, and the load-mass 
/// factor.</summary>
///<param name="Displacement"></param>
///<param name="DisplacementIncrement"></param>
///<param name="Velocity"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> GeneralizedSDoF2D::GetMappingValues(double Displacement, 
										            double DisplacementIncrement,
												    double Velocity)
{
	vector <double> Results;
	
	double InitialDisplacement = MyPositiveResistanceFunction[0][0];

	// Check to see if the displacement is positive or negative relative to the neutral orgin.
	double NextDisplacement = Displacement + DisplacementIncrement;
	
	double InstantaneousStiffness = 0.0;
	double RestoringForce = 0.0;
	double LoadMassFactor = 1.0;

	// See the relative displacement.
	if(NextDisplacement > MyNeutralOrigin)
	{
		// Activity is done on the positive side of the loading.
		if (Velocity > 0)
		{
			// *************** LOADING **************************
			// Loading is done on the positive side of the element.

			// Set the Restoring force based off of the new displacement.
			RestoringForce = this -> GetResistanceFunctionLoading(Displacement - MyNeutralOrigin + InitialDisplacement);

			// Set the Initial Stiffness.
			InstantaneousStiffness = this -> GetResistanceFunctionStiffness(Displacement - MyNeutralOrigin + InitialDisplacement);

			// Find the Load Mass Factor.
			LoadMassFactor = this -> GetLoadMassFactorLoading(Displacement - MyNeutralOrigin + InitialDisplacement);
		}
		else
		{
			// *************** UNLOADING **************************
			// Unloading is done on the positive side of the element.
			// Change the location of the Neutral Orgin.
			double CurrentForce = this -> GetResistanceFunctionLoading(Displacement - MyNeutralOrigin + InitialDisplacement);

			// Reset the new neutral orgin.
			double InitialStiffness = this -> GetResistanceInitialStiffness(Displacement - MyNeutralOrigin + InitialDisplacement);

			// Set the New Orgin.
			MyNeutralOrigin = Displacement - CurrentForce / InitialStiffness;

			// Set the Restoring force based off of the new displacement.
			RestoringForce = this -> GetResistanceFunctionUnloading(Displacement - MyNeutralOrigin);

			// Set the Initial Stiffness.
			InstantaneousStiffness = InitialStiffness;

			// Find the Load Mass Factor.
			LoadMassFactor = this -> GetLoadMassFactorUnloading(Displacement - MyNeutralOrigin + InitialDisplacement);
		};
	}
	else if(NextDisplacement < MyNeutralOrigin)
	{
		// Activity is done on the negative side of the loading.
		if (Velocity < 0)
		{
			// *************** LOADING **************************
			// Unloading is done on the negative side of the element.

			// Set the Restoring force based off of the new displacement.
			RestoringForce = this -> GetResistanceFunctionLoading(Displacement - MyNeutralOrigin + InitialDisplacement);

			// Set the Initial Stiffness.
			InstantaneousStiffness = this -> GetResistanceFunctionStiffness(Displacement - MyNeutralOrigin + InitialDisplacement);

			// Find the Load-Mass Factor.
			LoadMassFactor = this -> GetLoadMassFactorLoading(Displacement - MyNeutralOrigin + InitialDisplacement);
		}
		else
		{
			// *************** UNLOADING **************************
			// Unloading is done on the negative side of the element.
			// Change the location of the Neutral Orgin.
			double CurrentForce = this -> GetResistanceFunctionLoading(Displacement - MyNeutralOrigin + InitialDisplacement);

			// Reset the new neutral orgin.
			double InitialStiffness = this -> GetResistanceInitialStiffness(Displacement - MyNeutralOrigin + InitialDisplacement);

			// Set the New Orgin.
			MyNeutralOrigin = Displacement - CurrentForce / InitialStiffness;

			// Set the Restoring force based off of the new displacement.
			RestoringForce = this -> GetResistanceFunctionUnloading(Displacement - MyNeutralOrigin);

			// Set the Initial Stiffness.
			InstantaneousStiffness = InitialStiffness;

			// Find the Load-Mass Factor.
			LoadMassFactor = this -> GetLoadMassFactorUnloading(Displacement - MyNeutralOrigin);
		};
	}
	else
	{
		// If NextDisplacement equals MyNeutralOrigin, then there is no loading activity.
		
		// Set the Restoring force based off of the new displacement.
		RestoringForce = this -> GetResistanceFunctionLoading(Displacement - MyNeutralOrigin + InitialDisplacement);

		// Set the Initial Stiffness.
		InstantaneousStiffness = this -> GetResistanceFunctionStiffness(Displacement  - MyNeutralOrigin + InitialDisplacement);

		// Find the Load-Mass Factor.
		LoadMassFactor = this -> GetLoadMassFactorUnloading(Displacement - MyNeutralOrigin + InitialDisplacement);
	};

	Results.push_back(RestoringForce);
	Results.push_back(InstantaneousStiffness);
	Results.push_back(LoadMassFactor);

	// Reset the Value of the Neutral Orgin when we are unloading.
	return Results;
};

///<summary>Return the Initial Stiffness.</summary>
///<param name="Displacement"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double GeneralizedSDoF2D::GetResistanceInitialStiffness(double Displacement)
{
	// Initial Stiffness
	double Stiffness = 0.0;

	double InitialDisplacement = MyPositiveResistanceFunction[0][0];

	if (Displacement > InitialDisplacement)
	{
		// Unloading on the Positive Branch.
		Stiffness = (MyPositiveResistanceFunction[1][1] - MyPositiveResistanceFunction[0][1]) /
						(MyPositiveResistanceFunction[1][0] - MyPositiveResistanceFunction[0][0]);
	}
	else
	{
		// Unloading on the Negative Branch.
		Stiffness = (MyNegativeResistanceFunction[1][1] - MyNegativeResistanceFunction[0][1]) /
						(MyNegativeResistanceFunction[1][0] - MyNegativeResistanceFunction[0][0]);
	};

	return Stiffness;
};

///<summary></summary>
///<param name="Displacement"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double GeneralizedSDoF2D::GetResistanceFunctionStiffness(double Displacement)
{
	// Declare the Return Variable.
	double Stiffness = 0.0;

	if (Displacement > 0.0)
	{
		// Loading Positive. Go through a time march to find the displacement.
		bool StayInLoop = true;

		// Start at a time step 1 over.
		int i = 1;

		// Loop
		while (StayInLoop && i < MyPositiveResistanceFunction.size())
		{
			// Check to see if the cell is greater than the displacement.
			if(Displacement < MyPositiveResistanceFunction[i][0])
			{
				// Do a Linear Interpolation between two data point.
				// P = (Pi_1 - Pi)/(Ui_1 - Ui)

				double Pi = MyPositiveResistanceFunction[i-1][1];		// Load at previous time step.
				double Ui = MyPositiveResistanceFunction[i-1][0];		// Displacement at previous time step.

				double Pi_1 = MyPositiveResistanceFunction[i][1];		// Load at time step.
				double Ui_1 = MyPositiveResistanceFunction[i][0];		// Displacement at previous time step.

				Stiffness = (Pi_1 - Pi)/(Ui_1 - Ui);

				// Exit Loop
				StayInLoop = false;
			}

			// Increment
			i++;
		};
	}
	else
	{
		// Loading Positive. Go through a time march to find the displacement.
		bool StayInLoop = true;

		// Start at a time step 1 over.
		int i = 1;

		// Loop
		while (StayInLoop && i < MyNegativeResistanceFunction.size())
		{
			// Check to see if the cell is greater than the displacement.
			if(Displacement > MyNegativeResistanceFunction[i][0])
			{
				// Do a Linear Interpolation between two data point.
				// P = (Pi_1 - Pi)/(Ui_1 - Ui)

				double Pi = MyNegativeResistanceFunction[i-1][1];		// Load at previous time step.
				double Ui = MyNegativeResistanceFunction[i-1][0];		// Displacement at previous time step.

				double Pi_1 = MyNegativeResistanceFunction[i][1];		// Load at time step.
				double Ui_1 = MyNegativeResistanceFunction[i][0];		// Displacement at previous time step.

				Stiffness = (Pi_1 - Pi)/(Ui_1 - Ui);

				// Exit Loop
				StayInLoop = false;
			}

			// Increment
			i++;
		};
	}

	// Return variable.
	return Stiffness;
};

///<summary>Return the resistance function</summary>
///<param name="Displacement">Displacement of the SDOF.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double GeneralizedSDoF2D::GetResistanceFunctionLoading(double Displacement)
{
	// Declare the Return Variable.
	double RestoringForce = 0.0;

	// Find the Initial Displacement.
	double InitialDisplacement = MyPositiveResistanceFunction[0][0];

	if (Displacement > InitialDisplacement)
	{
		// Loading Positive. Go through a time march to find the displacement.
		bool StayInLoop = true;

		// Start at a time step 1 over.
		int i = 1;

		// Loop
		while (StayInLoop && i < MyPositiveResistanceFunction.size())
		{
			// Check to see if the cell is greater than the displacement.
			if(Displacement < MyPositiveResistanceFunction[i][0])
			{
				// Do a Linear Interpolation between two data point.
				// P = Pi + (Pi_1 - Pi)(U - Ui) / (Ui_1 - Ui)

				double Pi = MyPositiveResistanceFunction[i-1][1];		// Load at previous time step.
				double Ui = MyPositiveResistanceFunction[i-1][0];		// Displacement at previous time step.

				double Pi_1 = MyPositiveResistanceFunction[i][1];		// Load at time step.
				double Ui_1 = MyPositiveResistanceFunction[i][0];		// Displacement at previous time step.

				// Find the restoring force in the system.
				RestoringForce = Pi + (Pi_1 - Pi)*(Displacement - Ui)/(Ui_1 - Ui);

				// Exit Loop
				StayInLoop = false;
			}

			// Increment
			i++;
		};
	}
	else
	{
		// Loading Negative. Go through a time march to find the displacement.
		bool StayInLoop = true;

		// Start at a time step 1 over.
		int i = 1;

		// Loop
		while (StayInLoop && i < MyNegativeResistanceFunction.size())
		{
			// Check to see if the cell is greater than the displacement.
			if(Displacement > MyNegativeResistanceFunction[i][0])
			{
				// Do a Linear Interpolation between two data point.
				// P = Pi + (Pi_1 - Pi)(U - Ui) / (Ui_1 - Ui)

				double Pi = MyNegativeResistanceFunction[i - 1][1];		// Load at previous time step.
				double Ui = MyNegativeResistanceFunction[i - 1][0];		// Displacement at previous time step.

				double Pi_1 = MyNegativeResistanceFunction[i][1];		// Load at time step.
				double Ui_1 = MyNegativeResistanceFunction[i][0];		// Displacement at previous time step.

				RestoringForce = Pi  + (Pi_1 - Pi)*(Displacement - Ui)/(Ui_1 - Ui);

				// Exit Loop
				StayInLoop = false;
			}

			// Increment
			i++;
		};
	};

	return RestoringForce;
};


///<summary>Return the restoring force for the unloading branch of the resistance function.</summary>
///<param name="Displacement"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double GeneralizedSDoF2D::GetResistanceFunctionUnloading(double Displacement)
{
	// Declare the Return Variable.
	double RestoringForce = 0.0;

	// Initial Stiffness
	double Stiffness = this -> GetResistanceInitialStiffness(Displacement);

	// Calculate the Restoring Force.
	RestoringForce = Stiffness * Displacement;

	// Return Variable.
	return RestoringForce;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double GeneralizedSDoF2D::GetLoadMassFactorLoading(double Displacement)
{
	// Declare the Return Variable.
	double Factor = 0.0;

	// Find the Initial Displacement.
	double InitialDisplacement = MyLoadMassFactorPositive[0][0];

	if (Displacement > InitialDisplacement)
	{
		// Loading Positive. Go through a time march to find the displacement.
		bool StayInLoop = true;

		// Start at a time step 1 over.
		int i = 1;

		// Loop
		while (StayInLoop && i < MyLoadMassFactorPositive.size())
		{
			// Check to see if the cell is greater than the displacement.
			if(Displacement < MyLoadMassFactorPositive[i][0])
			{
				// Do a Linear Interpolation between two data point.
				// K = Ki + (Ki_1 - Ki)(U - Ui) / (Ui_1 - Ui)

				double Ki = MyLoadMassFactorPositive[i-1][1];		// Load at previous time step.
				double Ui = MyLoadMassFactorPositive[i-1][0];		// Displacement at previous time step.

				double Ki_1 = MyLoadMassFactorPositive[i][1];		// Load at time step.
				double Ui_1 = MyLoadMassFactorPositive[i][0];		// Displacement at previous time step.

				// Find the restoring force in the system.
				//Factor = Ki + (Ki_1 - Ki)*(Displacement - Ui)/(Ui_1 - Ui);
				Factor = Ki_1;

				// Exit Loop
				StayInLoop = false;
			}

			// Increment
			i++;
		};
	}
	else
	{
		// Loading Negative. Go through a time march to find the displacement.
		bool StayInLoop = true;

		// Start at a time step 1 over.
		int i = 1;

		// Loop
		while (StayInLoop && i < MyLoadMassFactorNegative.size())
		{
			// Check to see if the cell is greater than the displacement.
			if(Displacement > MyLoadMassFactorNegative[i][0])
			{
				// Do a Linear Interpolation between two data point.
				// K = Ki + (Ki_1 - Ki)(U - Ui) / (Ui_1 - Ui)

				double Ki = MyLoadMassFactorNegative[i - 1][1];		// Load at previous time step.
				double Ui = MyLoadMassFactorNegative[i - 1][0];		// Displacement at previous time step.

				double Ki_1 = MyLoadMassFactorNegative[i][1];		// Load at time step.
				double Ui_1 = MyLoadMassFactorNegative[i][0];		// Displacement at previous time step.

				Factor = Ki_1;
				//Factor = Ki  + (Ki_1 - Ki)*(Displacement - Ui)/(Ui_1 - Ui);

				// Exit Loop
				StayInLoop = false;
			}

			// Increment
			i++;
		};
	};

	return Factor;
};

///<summary>Return the load mass factor at the 0 increment.</summary>
///<param name="Displacement"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double GeneralizedSDoF2D::GetLoadMassFactorUnloading(double Displacement)
{
	// Declare the Return Variable.
	double Factor = 0.0;

	// Find the Initial Displacement.
	double InitialDisplacement = MyLoadMassFactorPositive[0][0];

	if (Displacement > InitialDisplacement)
	{
		Factor = MyLoadMassFactorPositive[0][1];
	}
	else
	{
		Factor = MyLoadMassFactorNegative[0][1];
	};

	// Return Variable.
	return Factor;
};

void GeneralizedSDoF2D::RunCentralDifferenceMethod(double TimeStep, 
												   double MaxTime, 
												   double InitialDisplacement, 
												   double InitialVelocity, 
												   double MassFactor, 
												   double DampingFactor)
{
	// Use the Central Difference Method.
	CentralDifferenceMethod CDM = CentralDifferenceMethod();

	// Max Time Increment.
	int MaxTimeIncrement = MaxTime / TimeStep;

	// Set the Initial Displacement
	MyNeutralOrigin = InitialDisplacement;
	double Displacement = InitialDisplacement;
	double DisplacementIncrement = 0.0;
	double Velocity = InitialVelocity;

	// Initial the Restoring Force, Stiffness Factor, and the Load-Mass Factor.
	vector <double> MapValues = this -> GetMappingValues(Displacement, 
			                                             DisplacementIncrement, 
														 Velocity);

	// Set the Values.
	double RestoringForce = MapValues[0];
	double StiffnessFactor = MapValues[1];
	double LoadMassFactor = MapValues[2];

	
	double InitialForce = this -> GetForceFunction(0.0);

	// Set the Initial Velocity and Acceleration.
	double Acceleration = CDM.GetInitialAcceleration(InitialForce,
													 LoadMassFactor * MassFactor, 
													 DampingFactor,
													 StiffnessFactor,
													 InitialVelocity,
													 InitialDisplacement);

	double PreviousDisplacement = CDM.GetInitialDisplacement(InitialDisplacement,
															 InitialVelocity,
															 Acceleration, 
															 TimeStep);
	// Set the Initial Loads
	double Load = 0.0;
	double LoadIncrement = 0.0;

	// Loop through the time step.
	for (int i = 0; i <= MaxTimeIncrement; i++)
	{
		// Get New Load.
		Load = this -> GetForceFunction(TimeStep * (i));

		// Get the Load Increment.
		LoadIncrement = this -> GetForceFunction(TimeStep * (i + 1)) - Load;

		// Get Mapping Values;
		vector <double> MapValues = this -> GetMappingValues(Displacement, 
			                                                 DisplacementIncrement, 
															 Velocity);

		// Set the Values.
		RestoringForce = MapValues[0];
		StiffnessFactor = MapValues[1];
		LoadMassFactor = MapValues[2];

		// Solve the Increment.
		vector <double> Results = CDM.Solve_NonLinear(LoadMassFactor * MassFactor,
												   DampingFactor,
												   StiffnessFactor,
												   RestoringForce,
												   Load,
												   LoadIncrement,
												   PreviousDisplacement,
												   Displacement,
												   Acceleration,
												   Velocity,
												   TimeStep);

		Results.push_back(LoadMassFactor);

		// Modify the Variables.
		DisplacementIncrement = Results[7];

		PreviousDisplacement = Results[4];

		Displacement = PreviousDisplacement + DisplacementIncrement;

		Velocity = Results[5] + Results[8];

		Acceleration = Results[6] + Results[9];

		// Store the Response History.
		MyResponseHistory.push_back(Results);
	};

	//string FilePath = MyTextFile.GetFilePath(MyPath, "NL.txt");
	MyTextFile.CreateTextFile(&MyTextFile.GetFilePath("RF_Positive.txt")[0], MyPositiveResistanceFunction);
	MyTextFile.CreateTextFile(&MyTextFile.GetFilePath("RF_Negative.txt")[0], MyNegativeResistanceFunction);
	MyTextFile.CreateTextFile(&MyTextFile.GetFilePath("LM_Positive.txt")[0], MyLoadMassFactorPositive);
	MyTextFile.CreateTextFile(&MyTextFile.GetFilePath("LM_Negative.txt")[0], MyLoadMassFactorNegative);
	MyTextFile.CreateTextFile(&MyTextFile.GetFilePath("NL.txt")[0], MyResponseHistory);
};

///<summary>Run the Newmark Beta.</summary>
///<param name="TimeStep">The time step duration for each step.</param>
///<param name="MaxTime">The maximum time to run the system.</param>
///<param name="InitialDisplacement">The initial displacement of the system.</param>
///<param name="InitialVelocity">The initial velocity of the system.</param>
///<param name="MassFactor">The Mass Factor used on the system.</param>
///<param name="DampingFactor">The Damping factor used on the system.</param>
///<remarks>description</remarks>
void GeneralizedSDoF2D::RunNewmarkBeta(double TimeStep, 
									   double MaxTime, 
									   double InitialDisplacement, 
									   double InitialVelocity,
									   double MassFactor,
									   double DampingFactor)
{
	// Use the Average Acceleration.
	NewmarkBeta NB = NewmarkBeta(0.5, 0.25);

	// Max Time Increment.
	int MaxTimeIncrement = MaxTime / TimeStep;

	// Set the Initial Displacement
	MyNeutralOrigin = InitialDisplacement;
	double Displacement = InitialDisplacement;
	double DisplacementIncrement = 0.0;
	
	// Initial the Restoring Force, Stiffness Factor, and the Load-Mass Factor.
	double RestoringForce = this -> GetResistanceFunctionLoading(Displacement);
	double StiffnessFactor = 0.0;
	double LoadMassFactor = this -> GetLoadMassFactorLoading(Displacement);
	double InitialForce = this -> GetForceFunction(0.0);

	// Set the Initial Velocity and Acceleration.
	double Velocity = InitialVelocity;
	double Acceleration = NB.GetInitialAcceleration(InitialForce,
													LoadMassFactor * MassFactor, 
													DampingFactor,
													RestoringForce,
													InitialVelocity);

	// Set the Initial Loads
	double Load = 0.0;
	double LoadIncrement = 0.0;
	
	// Loop through the time step.
	for (int i = 0; i <= MaxTimeIncrement; i++)
	{
		// Get New Load.
		Load = this -> GetForceFunction(TimeStep * (i));

		// Get the Load Increment.
		LoadIncrement = this -> GetForceFunction(TimeStep * (i + 1)) - Load;

		// Get Mapping Values;
		vector <double> MapValues = this -> GetMappingValues(Displacement, 
			                                                 DisplacementIncrement, 
															 Velocity);

		// Set the Values.
		RestoringForce = MapValues[0];
		StiffnessFactor = MapValues[1];
		LoadMassFactor = MapValues[2];

		// Solve the Increment.
		vector <double> Results = NB.Solve_NonLinear(LoadMassFactor * MassFactor,
													 DampingFactor,
													 StiffnessFactor,
													 Load,
													 LoadIncrement,
													 RestoringForce,
													 Displacement,
													 Acceleration,
													 Velocity,
													 TimeStep);

		Results.push_back(LoadMassFactor);

		// Modify the Variables.
		DisplacementIncrement = Results[7];

		Displacement = Results[4] + DisplacementIncrement;

		Velocity = Results[5] + Results[8];

		Acceleration = Results[6] + Results[9];

		// Store the Response History.
		MyResponseHistory.push_back(Results);
	};

	//string FilePath = MyTextFile.GetFilePath(MyPath, "NL.txt");
	MyTextFile.CreateTextFile(&MyTextFile.GetFilePath("RF_Positive.txt")[0], MyPositiveResistanceFunction);
	MyTextFile.CreateTextFile(&MyTextFile.GetFilePath("RF_Negative.txt")[0], MyNegativeResistanceFunction);
	MyTextFile.CreateTextFile(&MyTextFile.GetFilePath("LM_Positive.txt")[0], MyLoadMassFactorPositive);
	MyTextFile.CreateTextFile(&MyTextFile.GetFilePath("LM_Negative.txt")[0], MyLoadMassFactorNegative);
	MyTextFile.CreateTextFile(&MyTextFile.GetFilePath("NL.txt")[0], MyResponseHistory);
};

///<summary>Return the Force Function.  If this function is not inherted, then
/// the force function will interpolate through the table.</summary>
///<param name="time"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double GeneralizedSDoF2D::GetForceFunction(double time)
{
	// Declare the return values.
	double Force = 0.0;
	
	// Counter Index.
	size_t i = 0;

	// True/False Counter
	bool StayInLoop = true;

	// Loop through to find the load.
	while (StayInLoop && i < MyGeneralForceFunctionTable.size())
	{

		if (time < MyGeneralForceFunctionTable[i][0])
		{
			// Interpolate the value.
			if (i == 0)
			{
				// Store the first value.
				Force = MyGeneralForceFunctionTable[i][1];
			}
			else
			{
				// Interpolate.
				double Ti = MyGeneralForceFunctionTable[i-1][0];
				double Pi = MyGeneralForceFunctionTable[i-1][1];

				double Ti_1 = MyGeneralForceFunctionTable[i][0];
				double Pi_1 = MyGeneralForceFunctionTable[i][1];

				Force = Pi + (Pi_1 - Pi)*(time - Ti)/(Ti_1 - Ti);
			};

			// Exit Loop
			StayInLoop = false;
		}

		// Increment Counter
		i++;
	};

	return Force;
};

///<summary>Set the Gravity.</summary>
///<param name="Gravity"></param>
///<remarks>description</remarks>
void GeneralizedSDoF2D::SetGravity(double Gravity)
{
	MyGravity = Gravity;
};