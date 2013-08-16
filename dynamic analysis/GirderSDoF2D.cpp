#include "GirderSDoF2D.h"

///<summary>Declare the SDOF model parameter for the girder.</summary>
///<param name="ID">Girder ID</param>
///<param name="Length">Length of the girder.</param>
///<param name="CGx">Center of gravity from the left edge of the girder.</param>
///<param name="CGz">Center of gravity from the bottom flange of the girder.</param>
///<param name="MeshIncrement">Fidelity of the Girder Model.</param>
///<param name="Path">Path of where the Pressure Time Histories are saved.</param>
///<param name="Section">Section of the Girder.</param>
///<remarks>description</remarks>
GirderSDoF2D::GirderSDoF2D(int ID, 
						   double Length,
						   double CGx,
						   double CGz, 
						   int MeshIncrement, 
						   double Gravity,
						   char* Path, 
						   Section2D Section)
{
	//Determine if the Geoemtry is set. 
	IsGeometrySet = false;
	IsLoadSet = false;

	// Store the ID.
	MyID = ID;
	MyTextFile = TextFileIO(Path);

	MyLength = Length;
	MyMeshIncrement = MeshIncrement;
	MyGravity = Gravity;

	MySection = Section;

	// Set the Frame Geometry.
	this -> SetGirderFrame();

	// Download the PT Time-History.
	this -> SetBridgeExplosiveLoad(CGx, CGz, Path);

	// Set the Impulse along the Beam.
	this -> SetSpecificImpulseAndPressureVector();

	// Set the Incremental Pressure along the Girder.
	this -> SetIncrementBlastLoad();
};

GirderSDoF2D::GirderSDoF2D(int ID, 
						   double Length, 
						   double CGx,
						   double CGz, 
						   int MeshIncrement, 
						   double Gravity,
						   char *Path, 
						   GirderSection2D &Section)
{
	//Determine if the Geoemtry is set. 
	IsGeometrySet = false;
	IsLoadSet = false;

	// Store the ID.
	MyID = ID;
	MyTextFile = TextFileIO(Path);

	// Set the Geometry of the Overall Girder.
	MyLength = Length;

	if (CGz >= 0.0)
	{
		MyWidth = Section.GetTopFlangeWidth(); 
	}
	else
	{
		MyWidth = Section.GetBottomFlangeWidth();
	};
	
	MyMeshIncrement = MeshIncrement;
	
	MyGravity = Gravity;

	// Store the Girder Section.
	MyGirderSection = Section;
	MySection = MyGirderSection.GetBiLinearSection();

	// Set the Frame Geometry.
	this -> SetGirderFrame();

	// Set the Initial Force.
	this -> SetInitialStrandForces(MyGirderSection.GetInitialForce(), 
								   MyGirderSection.GetInitialMoment());

	// Download the PT Time-History.
	this -> SetBridgeExplosiveLoad(CGx, CGz, Path);

	// Set the Impulse along the Beam.
	this -> SetSpecificImpulseAndPressureVector();

	// Set the Incremental Pressure along the Girder.
	this -> SetIncrementBlastLoad(0.10);
};

GirderSDoF2D::GirderSDoF2D(int ID, 
					 double Length, 
					 double CGx,
					 double CGz, 
					 int MeshIncrement, 
					 double Gravity,
					 char* Path, 
					 GirderSection2D *Section)
{
	// Call the other constructor.
	GirderSDoF2D(ID,
				 Length,
				 CGx,
				 CGz,
				 MeshIncrement,
				 Gravity,
				 Path,
				 *Section);
				 
};

///<summary>Clone the girder single-degree of freedom.</summary>
///<param name="SDOF"></param>
///<remarks>description</remarks>
void GirderSDoF2D::Clone(const GirderSDoF2D &SDOF)
{
	MyID = SDOF.MyID;

	MyLength = SDOF.MyLength;
	MyWidth = SDOF.MyWidth;
	MyMeshIncrement = SDOF.MyMeshIncrement;

	MyGravity = SDOF.MyGravity;

	MySection = SDOF.MySection;
	MyPath = SDOF.MyPath;

	MyPressureAtCG = SDOF.MyPressureAtCG;
	MyPressureAtLeft = SDOF.MyPressureAtLeft;
	MyPressureAtRight = SDOF.MyPressureAtRight;

	MyExplosiveCGx = SDOF.MyExplosiveCGx;
	MyExplosiveCGz = SDOF.MyExplosiveCGz;

	MySpecificImpulseAtLeft = SDOF.MySpecificImpulseAtLeft;
	MySpecificImpulseAtRight = SDOF.MySpecificImpulseAtRight;
	MyImpulseDirectionFactor = SDOF.MyImpulseDirectionFactor;

	MyExplosiveCGx = SDOF.MyExplosiveCGx;
	MyExplosiveCGz = SDOF.MyExplosiveCGz;

	MyImpulseFactorAtLeft = SDOF.MyImpulseFactorAtLeft;
	MyImpulseFactorAtRight = SDOF.MyImpulseFactorAtRight;

	MyNodalSpecificImpulse = SDOF.MyNodalSpecificImpulse;

	MyGirder = SDOF.MyGirder;

	MyNLStaticAnalysis = SDOF.MyNLStaticAnalysis;
};

///<summary>Assignment Operator.</summary>
///<param name="SDOF"></param>
///<remarks>description</remarks>
GirderSDoF2D& GirderSDoF2D::operator= (const GirderSDoF2D &SDOF)
{
	// Call the Clone Subroutine.
	this -> Clone(SDOF);

	// Return a pointer to this Instance.
	return *this;
};

///<summary>Copy Constructor.</summary>
///<param name="SDOF"></param>
///<remarks>description</remarks>
GirderSDoF2D::GirderSDoF2D(const GirderSDoF2D &SDOF)
{
	this -> Clone(SDOF);
};

GirderSDoF2D::GirderSDoF2D(void)
{

}

GirderSDoF2D::~GirderSDoF2D(void)
{
	
};

/// ===========================================================================
///							SETTING GEOMETRY AND DEAD LOADS
/// ===========================================================================

#pragma region "SETTING GEOMETRY, DEAD LOADS, and BEL LOADS"
///<summary>Create the girder frame use.</summary>
///<param name="Section">Add the section to all the beam elements.</param>
///<remarks>description</remarks>
void GirderSDoF2D::SetGirderFrame()
{
	// Vector of Nodes and Beams.
	vector <Node2D> Nodes;
	vector <Beam2D> Beams;

	// Set the Length.
	double Increment = MyLength / (MyMeshIncrement);

	// Define Nodes.
	for(int i = 0; i < MyMeshIncrement + 1; i++)
	{
		Nodes.push_back(Node2D(i + 1, i * Increment, 0.0, true, true, true));
	};
	
	// Fix the Support Conditions
	Nodes[0].SetDOF(false, false, true);
	Nodes[MyMeshIncrement].SetDOF(true, false, true);

	// Constrain the Boolean
	bool Constrain[] = {false, false, false};

	// Define Beams.
	for(int j = 0; j < MyMeshIncrement; j++)
	{
		Beams.push_back(Beam2D(j + 1, 
						Nodes[j], 
						Nodes[j + 1], 
						MySection));
	};

	// Add Nodes, Section, and Beam.
	MyGirder.AddNode(Nodes);
	MyGirder.AddSection(MySection);
	MyGirder.AddBeam(Beams);

	// Set the Static Analysis
	MyNLStaticAnalysis.SetFrameGeometry(MyGirder);

	// Set the Self Weight.
	this -> SetSelfWeight();

	// Geometry is establish.
	IsGeometrySet = true;
};

///<summary>Add the Self Weight of the Beam into the Static Load Case for the Beam.</summary>
///<remarks>description</remarks>
void GirderSDoF2D::SetSelfWeight()
{
	// Add the Self Weight of Each Beam.

	for (size_t i = 0; i < MyGirder.GetNumberOfBeams(); i++)
	{
		Beam2D& Beam = MyGirder.GetBeamAddress(i + 1);

		// Add the Self Weight.
		MyNLStaticAnalysis.AddStaticLoad(UniformLoad2D(Beam, 
													   -Beam.GetSelfWeightFactor() * MyGravity));
	};
};

///<summary>Set the Initial Strand Forces.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
void GirderSDoF2D::SetInitialStrandForces(double InitialForce, double InitialMoment)
{
	// Load the first node into the static analysis.  
	// The initial moment is applied at the edge.
	MyNLStaticAnalysis.AddStaticLoad(NodeLoad2D(MyGirder.GetNodeAddress(1), 
												0.0,
												0.0,
												-InitialMoment));

	// Load the last node into the non-linear static analysis.  
	// The initial force and initial moment is applied at the edge.
	MyNLStaticAnalysis.AddStaticLoad(NodeLoad2D(MyGirder.GetNodeAddress(MyMeshIncrement + 1), 
												InitialForce,
												0.0,
												InitialMoment));
};

#pragma endregion

/// ===========================================================================
///							BLAST EXPLOSIVE LOAD
/// ===========================================================================

#pragma region "BLAST EXPLOSIVE LOAD"

///<summary>Establish the Bridge Explosive Load.</summary>
///<param name="CGx">Center of Gravity of the load with respect to the left edge of the girder.</param>
///<param name="CGz">Center of Gravity of the load with respect to the bottom edge of the girder.</param>
///<param name="Path">Path of the load.</param>
///<remarks>description</remarks>
void GirderSDoF2D::SetBridgeExplosiveLoad(double CGx, 
										  double CGz,
										  string Path)
{
	// Store the Path.
	MyPath = Path;

	// Store the CG Location along the X-Axis.
	MyExplosiveCGx = CGx;
	MyExplosiveCGz = CGz;
	
	//Path 
	string PathCG = Path + "\Center.csv";
	string PathLeft = Path + "\Left.csv";
	string PathRight = Path + "\Right.csv";

	// Store the Pressure Time History from the chart.
	//MyPressureAtCG = MyTextFile.GetPressureTimeHistory(&PathCG[0]);
	//MyPressureAtLeft = MyTextFile.GetPressureTimeHistory(&PathLeft[0]);
	//MyPressureAtRight = MyTextFile.GetPressureTimeHistory(&PathRight[0]);

	MyPressureAtCG = MyTextFile.GetPressureTimeHistoryTabDelimited(&PathCG[0], 3, 16, 16);
	MyPressureAtLeft = MyTextFile.GetPressureTimeHistoryTabDelimited(&PathLeft[0], 3, 16, 16);
	MyPressureAtRight = MyTextFile.GetPressureTimeHistoryTabDelimited(&PathRight[0], 3, 16, 16);

	// Find the Impulse at CG, Left, and Right Edge.
	MySpecificImpulseAtCG = this -> GetImpulse(MyPressureAtCG, 
										1.0 / 1000.0);
	MySpecificImpulseAtLeft = this -> GetImpulse(MyPressureAtLeft, 
										 1.0 / 1000.0);
	MySpecificImpulseAtRight =  this -> GetImpulse(MyPressureAtRight, 
										   1.0 / 1000.0);

	// Peak Pressure
	MyPeakReflectedPressureAtCG = this -> GetPeakPressure(MyPressureAtCG);
	MyPeakReflectedPressureAtLeft = this -> GetPeakPressure(MyPressureAtLeft);
	MyPeakReflectedPressureAtRight = this -> GetPeakPressure(MyPressureAtRight);

	// Set the Side Impulse Factors.
	if (CGz > 0.0)
	{
		MyImpulseDirectionFactor = -1.0;
	}
	else
	{
		MyImpulseDirectionFactor = 1.0;
	};

	MyImpulseFactorAtLeft = MySpecificImpulseAtLeft / MySpecificImpulseAtCG;
	MyImpulseFactorAtRight = MySpecificImpulseAtRight / MySpecificImpulseAtCG;
};

///<summary>Store the Nodal Impulse along the Length of the Cross Section.</summary>
///<remarks>description</remarks>
void GirderSDoF2D::SetSpecificImpulseAndPressureVector()
{
	/*					I_cg

					  ------
				------		---------
			----					----------
		 --									--------|
		|											|
	   \|/										   \|/
		============================================
	AlphaLeft * I_cg								AlphaRight * I_cg
		
	*/
	vector <double> ImpulseVector;
	vector <double> PressureVector;

	// Set teh Length.
	double Increment = MyLength / (MyMeshIncrement);

	// Define Nodes.
	for(int i = 0; i < MyMeshIncrement + 1; i++)
	{
		// Find the Impulse given the location along the length.
		ImpulseVector.push_back(this -> GetImpulseAtLocation(i * Increment));

		PressureVector.push_back(this -> GetPressureAtLocation(i * Increment));
	};
	
	// Save the Impulse and Peak Pressure at Vector.
	MyNodalSpecificImpulse = ImpulseVector;
	MyNodalPeakPressure = PressureVector;
};

///<summary>Return the incremental reference load.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
double GirderSDoF2D::GetIncrementalReferenceLoad(double ScaleLoad)
{
	// Incremental Reference Load is the Peak Pressure * Width of the Flange.
	double ReferenceLoad = 0.0;

	// Find the Peak Pressure.
	double PeakPressure = this -> GetPeakPressure(MyPressureAtCG);

	if (MyExplosiveCGz >= 0)
	{
		// Above Deck Scenario.
		ReferenceLoad = PeakPressure * MyGirderSection.GetTopFlangeWidth();
	}
	else
	{
		// Below Deck Scenario.
		ReferenceLoad = PeakPressure * MyGirderSection.GetBottomFlangeWidth();
	};

	return ReferenceLoad;
};

///<summary>Set the increment blast loading along the beam.</summary>
///<param name="ScaleIncrementLoad">Scale the increment loading.</param>
///<remarks>description</remarks>
void GirderSDoF2D::SetIncrementBlastLoad(double ScaleIncrementLoad)
{
	/*					1.0

					  ------
				------		---------
			----					----------
		 --									--------|
		|											|
	   \|/										   \|/
		============================================
	AlphaLeft * 1.0								AlphaRight * 1.0
		
	*/

	vector <double> NodalPressure;

	// Normalize the Load.
	for (size_t i = 0; i < MyNodalSpecificImpulse.size(); i++)
	{
		NodalPressure.push_back(MyNodalSpecificImpulse[i] / MySpecificImpulseAtCG);
	};

	// Find the Increment Length.
	double Increment = MyLength / MyMeshIncrement;

	// Declare the Explosive Distance from the Left End.
	double ExpDistance = MyExplosiveCGx;

	// Peak
	double MaxMagnitude = this -> GetIncrementalReferenceLoad(ScaleIncrementLoad);

	// Store the Beams
	for (size_t j = 0; j < MyMeshIncrement; j++)
	{
		Beam2D& Beam = MyGirder.GetBeamAddress(j + 1);

		if (j * Increment > ExpDistance && (j + 1) * Increment < ExpDistance)
		{
			// Add Pyramid Load
			/*
							1   
					   _______		
				_______		  ______
				|					|
				=====================
			*/

			

			// Length on Beam to CG.
			double Length = MyExplosiveCGx - j * Increment;
			
			// Add the Pyramid Load
			PyramidLoad2D Load = PyramidLoad2D(MyGirder.GetBeamAddress(j + 1), 
											   MyImpulseDirectionFactor * MaxMagnitude, 
											   MyImpulseDirectionFactor * MaxMagnitude * NodalPressure[j], 
											   MyImpulseDirectionFactor * MaxMagnitude * NodalPressure[j + 1], 
											   Length);
		
			// Add to Non-Linear Static Analysis the Pyramid Load.
			MyNLStaticAnalysis.AddIncrementalLoad(Load * ScaleIncrementLoad);
		}
		else
		{
			/*
							  ______
					   _______		|
				_______				|
				|					|
				=====================
			*/
			
			// Declare the Triangle Load.
			TriangularLoad2D Load = TriangularLoad2D(MyGirder.GetBeamAddress(j + 1), 
													 MyImpulseDirectionFactor * MaxMagnitude * NodalPressure[j], 
													 MyImpulseDirectionFactor * MaxMagnitude * NodalPressure[j + 1]);

			// Add to Non-Linear Static Analysis the Triangle Load.
			MyNLStaticAnalysis.AddIncrementalLoad(Load * ScaleIncrementLoad);
		}
	};
};

///<summary>Return the Impulse at the Location.</summary>
///<param name="X">X location of the impulse.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double GirderSDoF2D::GetImpulseAtLocation(double X)
{
	double Impulse;

	if(X < MyExplosiveCGx)
	{
		// I(x) = AlphaLeft * Icg + (1 - AlphaLeft) * Icg * (x / L_left)
		double Icg = MySpecificImpulseAtCG;
		double L_left = MyExplosiveCGx;
		double AlphaLeft = MyImpulseFactorAtLeft;

		Impulse = AlphaLeft * Icg + (1 - AlphaLeft) * Icg * (X / L_left);
	}
	else 
	{
		// I(x) = Icg - (1 - AlphaRight) * Icg * ((X - L_CG) / L_right)
		double Icg = MySpecificImpulseAtCG;
		double L_CG = MyExplosiveCGx;
		double L_Right =  MyLength - MyExplosiveCGx;
		double AlphaRight = MyImpulseFactorAtRight;

		Impulse = Icg - (1 - AlphaRight) * Icg * ((X - L_CG) / L_Right);
	}

	return Impulse;
};

///<summary>Return the Max at the time of first incident. </summary>
///<param name="X">Length from the Left end of the Girder.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double GirderSDoF2D::GetPressureAtLocation(double X)
{
	double Pressure;

	double AlphaLeft = MySpecificImpulseAtLeft / MySpecificImpulseAtCG;
	double AlphaRight = MySpecificImpulseAtRight / MySpecificImpulseAtCG;

	// Find the Maximum Pressure for the give Element.
	double MaxPressureAtCG = MyPeakReflectedPressureAtCG;
	double MaxPressureAtLeft = MyPeakReflectedPressureAtLeft;
	double MaxPressureAtRight = MyPeakReflectedPressureAtRight;

	if (X < MyExplosiveCGx)
	{
		double L_left = MyExplosiveCGx;

		Pressure = MaxPressureAtLeft + (MaxPressureAtCG - MaxPressureAtLeft) * (X / L_left);
	}
	else
	{
		double L_CG = MyExplosiveCGx;
		double L_Right = MyLength - MyExplosiveCGx;

		Pressure = MaxPressureAtCG - (MaxPressureAtCG - MaxPressureAtRight) * (X - L_CG) / (L_Right);
	};

	return Pressure;
};

///<summary>Add Super-Imposed Dead Load from the decking, rails, etc.</summary>
///<param name="DL">Magnitude of the Dead Load.</param>
///<remarks>description</remarks>
void GirderSDoF2D::SetUniformDeadLoad(double DL)
{
	// Add the Self Weight of Each Beam.
	for (size_t i = 0; i < MyGirder.GetNumberOfBeams(); i++)
	{
		Beam2D& Beam = MyGirder.GetBeamAddress(i + 1);

		// Add the Super-Impose Dead Load.
		MyNLStaticAnalysis.AddStaticLoad(UniformLoad2D(Beam, DL));
	};
};


///<summary>Find the equivalent uniform specific impulse of the girder section.</summary>
///<param name="DOF">X axis, Y axis, or Rotation</param>
///<param name="LoadStep">Determine which load step.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double GirderSDoF2D::GetEquivalentUniformSpecificImpulse(int DOF, 
														 int LoadStep)
{
	// Equivalent Uniform Impulse
	// Use the Transform Impulse.
	// ImpulseEUI = sum(I * phi) / sum(I)

	// Retreive the Shape Vector at an early time increment.
	vector <double> ShapeVector = MyNLStaticAnalysis.GetShapeVector(DOF, LoadStep, true);

	double Numerator = 0.0;
	double Denominator = 0.0;

	// Loop find Sum.
	for (int i = 0; i < ShapeVector.size(); i++)
	{ 
		// Sum of the Vector.
		Numerator += ShapeVector[i] * MyNodalSpecificImpulse[i];

		// sum(ShapeVector)
		Denominator += ShapeVector[i];
	};

	return Numerator / Denominator;
};

///<summary></summary>
///<param name="DOF"></param>
///<param name="LoadStep"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double GirderSDoF2D::GetEquivalentPeakPressure(int DOF, 
											   int LoadStep)
{
	// Equivalent Peak Pressure

	// Retreive the Shape Vector at an early time increment.
	vector <double> ShapeVector = MyNLStaticAnalysis.GetShapeVector(DOF, LoadStep, true);

	double Numerator = 0.0;
	double Denominator = 0.0;

	// Loop find Sum.
	for (int i = 0; i < ShapeVector.size(); i++)
	{ 
		// sum(Impulse * ShapeVector)
		Numerator += ShapeVector[i] * MyNodalPeakPressure[i];

		// sum(ShapeVector)
		Denominator += ShapeVector[i];
	};

	return Numerator / Denominator;
};

///<summary>Returns a vector that is an equivalent uniform impulse vector.  
///Meaning, that the displacement is counted twice at the end of a connecting node.  
///Therefore the impulse at the middle node is split and counted twice.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> GirderSDoF2D::GetImpulseVector()
{
	vector <double> Impulse;

	/*					1.0

					  -------
				------|		|--------
			----|	  |		|		|-----------
		 --|	|	  |		|		|		   |----|
		|  | 	|	  |		|		|		   |	|		
	   \|/ |	|	  |		|		|		   |   \|/
		============================================
	AlphaLeft * 1.0								AlphaRight * 1.0
		
	*/
	// Find the Number of Nodes.
	size_t NumOfNodes = MyNodalSpecificImpulse.size();

	// Find the Mesh Length.
	double MeshLength = MyLength / MyMeshIncrement;
	
	for (size_t i = 0; i < NumOfNodes; i++)
	{
		double SpecificImpulseLeft = 0;
		double SpecificImpulseCenter = 0;
		double SpecificImpulseRight = 0;

		double NodalImpulse = 0;

		if (i == 0)
		{
			// Beam[0] First Node
			SpecificImpulseLeft = 0;
			SpecificImpulseCenter = MyNodalSpecificImpulse[0];
			SpecificImpulseRight = MyNodalSpecificImpulse[1];

			// Add the Specific Impulse to the Right.
			NodalImpulse += ((SpecificImpulseCenter + SpecificImpulseRight) / 2) * (MeshLength / 2) * MyWidth;
		}
		else if (i == NumOfNodes - 1)
		{
			// Beam[0] Last Node
			SpecificImpulseLeft = MyNodalSpecificImpulse[i - 1];
			SpecificImpulseCenter = MyNodalSpecificImpulse[i];
			SpecificImpulseRight = MyNodalSpecificImpulse[0];

			// Add the Specific Impulse to the Left.
			NodalImpulse += ((SpecificImpulseCenter + SpecificImpulseLeft) / 2) * (MeshLength / 2) * MyWidth;
		}
		else
		{
			// Add to Beam [i-1] Node B.
			SpecificImpulseLeft = MyNodalSpecificImpulse[i - 1];
			SpecificImpulseCenter = MyNodalSpecificImpulse[i];
			SpecificImpulseRight = MyNodalSpecificImpulse[i + 1];	

			NodalImpulse += ((SpecificImpulseCenter + SpecificImpulseLeft) / 2) * (MeshLength / 2) * MyWidth;

			NodalImpulse += ((SpecificImpulseCenter + SpecificImpulseRight) / 2) * (MeshLength / 2)* MyWidth;
		};

		// Find the Impulse.
		Impulse.push_back(NodalImpulse);
	};

	// Return the Nodal Impulse
	return Impulse;
};

///<summary>Return the Maximum Pressure from the PT History.</summary>
///<param name="PressureTimeHistory"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double GirderSDoF2D::GetPeakPressure(vector<vector<double>> &PressureTimeHistory)
{
	// We know that the pressure decays after hitting it's peak, therefore the function stops 
	// when the value is decaying at +5 increment.

	double MaxPressure = 0;

	bool StayInLoop = true;
	int DecayingIncrement = 0;
	int MaxDecay = 5;

	size_t MaxLoop = PressureTimeHistory.size();
	int LoopIncrement = 0;
	
	while (LoopIncrement < MaxLoop && StayInLoop)
	{
		// Find the Pressure at this increment.
		double Pressure = PressureTimeHistory[LoopIncrement][1];

		if (MaxPressure < Pressure)
		{
			// Change the Maximum Pressure.
			MaxPressure = Pressure;
			
			// Reset the Decaying Increment.
			DecayingIncrement = 0;
		}
		else
		{
			// Increment the Decaying Increment.
			DecayingIncrement++;
		};

		// If the Decay Increment is greater than Maximum Decay, than
		// we know we have the maximum value.
		if (DecayingIncrement > MaxDecay)
		{
			StayInLoop = false;
		};

		// Increment the Index.
		LoopIncrement++;
	};
	
	return MaxPressure;
};

///<summary>Set the forcing function.</summary>
///<remarks>description</remarks>
void GirderSDoF2D::SetForcingFunction()
{
	// Get the Equivalent Uniform Impulse and Peak Pressure.
	MyEquivalentUniformSpecificImpulse = this -> GetEquivalentUniformSpecificImpulse(GeneralizedSDoF2D::TranslationY, 1);

	MyEquivalentPeakPressure = this -> GetEquivalentPeakPressure(GeneralizedSDoF2D::TranslationY, 1);

	// Find the Shape Factor for the Element.
	// For the Prestressed Girder the Shape Factor is 1.0;
	double ShapeFactor = 1.0;

	// Equivalent Time Constant.
	MyEquivalentTimeConstant = (ShapeFactor * MyEquivalentUniformSpecificImpulse) / MyPeakReflectedPressureAtCG;

	// Set the Forcing Function.
	MyForceFunctionMaxMagnitude = this -> GetPeakForce();
};

double GirderSDoF2D::GetPeakForce()
{
	double MaxForce = 0.0;

	size_t NumOfGirder = MyGirder.GetNumberOfBeams();

	// Loop through the Sequence to find the Equivalent Force on the System.
	for (size_t i = 0; i < NumOfGirder; i++)
	{
		// Width of the Element: 
		Beam2D Beam = MyGirder.GetBeam(i + 1);

		Node2D NodeA = Beam.GetNodeA();
		Node2D NodeB = Beam.GetNodeB();

		double Length = Beam.GetLength();

		// Get the Pressure at A.
		double PressureAtA = this-> GetPressureAtLocation(NodeA.GetX());
		double PressureAtB = this-> GetPressureAtLocation(NodeB.GetX());

		// Find the Max Force.
		MaxForce += (PressureAtA + PressureAtB) / 2 * MyWidth * Length;
	};

	return MaxForce;
};

///<summary>Return the force from the decaying force calculated.</summary>
///<param name="time">Time variable used in the decarying force equation.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double GirderSDoF2D::GetForceFunction(double time)
{
	// Fmax * e^(-t/to)
	return MyImpulseDirectionFactor * MyForceFunctionMaxMagnitude * std::exp(-time / MyEquivalentTimeConstant);
};

#pragma endregion

/// ===========================================================================
///					RUN SINGLE-DEGREE-OF-FREEDOM ANALYSIS
/// ===========================================================================
#pragma region "Run SDOF"

///<summary>Run Equivalent Uniform Single-Degree-of-Freedom design by Eric Sammarco.</summary>
///<param name="ScaleIncrementLoad">Scale the Incremental Load.</param>
///<param name="NodeDOF">Degree-of-freedom direction to determine.</param>
///<remarks>description</remarks>
void GirderSDoF2D::RunSDoF(double ScaleIncrementLoad, 
						   int NodeDOF, 
						   double TimeStep,
						   double MaxTime)
{
	// Run the Non-Linear Static Analysis.
	MyNLStaticAnalysis.RunAnalysis(NonLinearStatic2D::ArcLength, 10, 0.01, 20, 0.01);

	// Set the Parameters of the Forcing Function.
	this -> SetForcingFunction();

	// Find the Node ID.
	int NodeID = MyNLStaticAnalysis.GetNodeIDWithLargestDeformation(NodeDOF,
																	GeneralizedSDoF2D::TranslationY,
																	true);

	// Return the Resistance Function for the positive loading.
	vector <vector<double>> PositiveLoadingResistanceFunction = MyNLStaticAnalysis.GetResistanceFunction(NodeID, 
																										 NodeDOF, 
																										 NodeDOF, 
																										 true);

	// Return the Resistance function for the negative loading.
	vector <vector<double>> NegativeLoadingResistanceFunction = MyNLStaticAnalysis.GetResistanceFunction(NodeID, 
																										 NodeDOF, 
																										 NodeDOF, 
																										 false);

	vector <vector<double>> PositiveLoadingLoadMassFactor = MyNLStaticAnalysis.GetLoadMassFactor(NodeID,
																								 NodeDOF,
																								 NodeDOF,
																								 true);
	
	vector <vector<double>> NegativeLoadingLoadMassFactor = MyNLStaticAnalysis.GetLoadMassFactor(NodeID,
																								 NodeDOF,
																								 NodeDOF,
																								 false);

	// Change the Resistance Function.
	this -> ModifyResistanceFunction(PositiveLoadingResistanceFunction, PositiveLoadingLoadMassFactor);
	this -> ModifyResistanceFunction(NegativeLoadingResistanceFunction, NegativeLoadingLoadMassFactor);

	// Find the Direction of Loading.
	if (MyExplosiveCGz >= 0.0)
	{
		// Store the Resistace function for positive direction in displacements.
		MyPositiveResistanceFunction = NegativeLoadingResistanceFunction;
		MyNegativeResistanceFunction = PositiveLoadingResistanceFunction;

		MyLoadMassFactorPositive = NegativeLoadingLoadMassFactor;
		MyLoadMassFactorNegative = PositiveLoadingLoadMassFactor;
	}
	else
	{
		// Store the Resistace function for positive direction in displacements.
		MyPositiveResistanceFunction = PositiveLoadingResistanceFunction;
		MyNegativeResistanceFunction = NegativeLoadingResistanceFunction;

		// Get Positive Load-Mass Factor.
		MyLoadMassFactorPositive = PositiveLoadingLoadMassFactor;
		MyLoadMassFactorNegative = NegativeLoadingLoadMassFactor;
	};

	// Run the Numeric Analysis.
	this -> RunNumericAnalysis(CentralDifferenceMethod::ID, 
							   TimeStep,
							   MaxTime);
};

void GirderSDoF2D::ModifyResistanceFunction(vector<vector<double>> &ResistanceFunction, 
											vector<vector<double>> &LoadMassFactor)
{
	// Find the Difference.
	double Difference = ResistanceFunction[1][0] - ResistanceFunction[0][0];
	
	bool PositiveLoadingDirection;

	// Find the Direction Factor.
	if(Difference/abs(Difference) > 0.0)
	{
		PositiveLoadingDirection = true;	
	}
	else
	{
		PositiveLoadingDirection = false;
	};
	
	for (size_t i = 1; i < ResistanceFunction.size(); i++)
	{
		if (PositiveLoadingDirection)
		{
			if(ResistanceFunction[i][0] < ResistanceFunction[i-1][0])
			{
				// Change the Displacement and Modifiy the Load.
				ResistanceFunction[i][0] = 5 * ResistanceFunction[i-1][0];
				ResistanceFunction[i][1] = ResistanceFunction[i-1][1];

				LoadMassFactor[i][0] = 5 * LoadMassFactor[i-1][0];

				// TODO: Temporarily save the Load Mass Factor of the Last Increment.
				LoadMassFactor[i][1] = LoadMassFactor.back()[1];
			};
		}
		else
		{
			if(ResistanceFunction[i][0] > ResistanceFunction[i-1][0])
			{
				// Change the Displacement and Modifiy the Load.
				ResistanceFunction[i][0] = 5 * ResistanceFunction[i-1][0];
				ResistanceFunction[i][1] = ResistanceFunction[i-1][1];

				LoadMassFactor[i][0] = 5 * LoadMassFactor[i-1][0];
				// TODO: Temporarily save the Load Mass Factor of the Last Increment.
				LoadMassFactor[i][1] = LoadMassFactor.back()[1];
			};
		};
		
	};
};

///<summary>Run the Numeical Analysis.</summary>
///<param name="TypeOfAnalysis">Set the Type of Analysis.</param>
///<remarks>description</remarks>
void GirderSDoF2D::RunNumericAnalysis(int TypeOfAnalysis, 
									  double TimeStep, 
									  double MaxTime)
{
	// Mass Factor
	double MassFactor = MyNLStaticAnalysis.GetTotalMass(ElasticFrame2D::TranslationY, 1, true);

	// Find the initial displacements.
	double InitialDisplacement = MyPositiveResistanceFunction[0][0];

	switch (TypeOfAnalysis)
	{
		case (NewmarkBeta::ID):
			this -> RunNewmarkBeta(TimeStep, 
								   MaxTime, 
								   InitialDisplacement, 
								   0.0, 
								   MassFactor, 
								   0.0);
			
			break; 
		case (CentralDifferenceMethod::ID):
			this -> RunCentralDifferenceMethod(TimeStep, 
											   MaxTime, 
											   InitialDisplacement, 
											   0.0, 
											   MassFactor, 
											   0.0);
			
			break;
	};
};
#pragma endregion
