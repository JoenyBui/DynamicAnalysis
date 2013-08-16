#include "StdAfx.h"
#include "TextFileIO.h"

	/* Cited from (http://www.cplusplus.com/doc/tutorial/files/)
	Options Mode for Opening File
	Where filename is a null-terminated character sequence of type const char * (the same type that string literals have) 
	representing the name of the file to be opened, and mode is an optional parameter with a combination of the following flags:

	ios::in			Open for input operations.
	ios::out		Open for output operations.
	ios::binary		Open in binary mode.
	ios::ate		Set the initial position at the end of the file.
					If this flag is not set to any value, the initial position is the beginning of the file.
	ios::app		All output operations are performed at the end of the file, appending the content to the current content of the file. 
					This flag can only be used in streams open for output-only operations.
	ios::trunc		If the file opened for output operations already existed before, its previous content is deleted and replaced 
					by the new one.

	All these flags can be combined using the bitwise operator OR (|). For example, if we want to open the file example.bin in 
	binary mode to add data we could do it by the following call to member function open():
	*/
using namespace std;

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void TextFileIO::Initialize(string Path)
{
	MyPath = Path;
};

TextFileIO::TextFileIO(string Path)
{
	this -> Initialize(Path);
};

TextFileIO::TextFileIO(void)
{
	this -> Initialize("");
}

TextFileIO::~TextFileIO(void)
{
}

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void TextFileIO::CreateTextFile(char* Path, vector <vector<double>> Data)
{
	ofstream myfile;

	myfile.open (Path);

	for (size_t i = 0; i < Data.size(); i++)
	{
		vector <double> Line = Data[i];

		for (size_t j = 0; j < Line.size(); j++)
		{
			myfile << Line[j] << "\t";
		};
		
		myfile << endl;
	};
	
	myfile.close();
};

///<summary></summary>
///<param name="Path"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector<vector<double>> TextFileIO::GetPressureTimeHistory(char* Path)
{
	vector<vector<double>>  PressureTime;

	vector <double> Time;
	vector <double> Pressure;

	ifstream myfile (Path);

	if (myfile.is_open())
	{
		int i = 0;

		while (myfile.good())
		{
			// Declare the String.
			string line;

			// Return the Next Line in the String.
			getline (myfile, line);
			
			if (i >= 2)
			{
				// Locate the Comma Position.
				int Comma = line.find(",");
			
				// Declare the Time String.
				string TimeString = line.substr(0, Comma);
				
				// Declare the Pressure String.
				string PressureString = line.substr(Comma + 1, line.length());

				// Remove white spaces.
				boost::algorithm::erase_all(TimeString, " ");
				boost::algorithm::erase_all(PressureString, " ");

				// Check to see if the Values are empty.
				if (PressureString != "" && TimeString != "")
				{
					// Vector that will store one pair of Pressure-Time
					// 0 - Time, 1 - Pressure
					vector <double> PT;

					PT.push_back(boost::lexical_cast<double>(TimeString));
				
					PT.push_back(boost::lexical_cast<double>(PressureString));

					// Store the Vector of the Pressure Time into the History Vector..
					PressureTime.push_back(PT);
				}
			};

			// Increment the Counter
			i++;
		};
	};

	myfile.close();

	return PressureTime;
};

///<summary></summary>
///<param name="Path"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector<vector<double>> TextFileIO::GetPressureTimeHistoryTabDelimited(char* Path, 
																	  int StartIndex,
																	  int TimeLength,
																	  int PressureLength)
{
	vector<vector<double>>  PressureTime;

	vector <double> Time;
	vector <double> Pressure;

	ifstream myfile (Path);

	if (myfile.is_open())
	{
		int i = 0;

		while (myfile.good())
		{
			// Declare the String.
			string line;

			// Return the Next Line in the String.
			getline (myfile, line);
			
			// Check first to see if there is anythng in this line.
			if (line.size() != 0)
			{
				// Start to look for index.
				if (i >= StartIndex)
				{			
					// Declare the Time String.
					string TimeString = line.substr(0, TimeLength);
					
					// Declare the Pressure String.
					string PressureString = line.substr(TimeLength, PressureLength);

					// Remove white spaces.
					boost::algorithm::erase_all(TimeString, " ");
					boost::algorithm::erase_all(PressureString, " ");

					// Check to see if the Values are empty.
					if (PressureString != "" && TimeString != "")
					{
						// Vector that will store one pair of Pressure-Time
						// 0 - Time, 1 - Pressure
						vector <double> PT;

						PT.push_back(boost::lexical_cast<double>(TimeString));
					
						PT.push_back(boost::lexical_cast<double>(PressureString));

						// Store the Vector of the Pressure Time into the History Vector..
						PressureTime.push_back(PT);
					}
				};
			};

			// Increment the Counter
			i++;
		};
	};

	myfile.close();

	return PressureTime;
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void TextFileIO::PrintPressureTimeHistory(const std::vector<vector<double>> &PT)
{
	cout << "Time    " << "Pressure" << endl;
	cout << PT.size() << endl;

	for (size_t i = 0; i < PT.size(); i++)
	{
		cout << PT[i][0] << " " << PT[i][1] << endl;
	};
};


void TextFileIO::SetPath(string Path)
{
	MyPath = Path;
};

string TextFileIO::GetFilePath(string FileName)
{
	return  MyPath + FileName;
};