#pragma once

#ifndef TextFileIO_H
#define TextFileIO_H

#include "stdafx.h"

using namespace std;

class TextFileIO
{
	private:

	protected:
		void Initialize(string Path);
		void Clone();

		string MyPath;


	public:
		TextFileIO(string Path);

		TextFileIO(void);
		~TextFileIO(void);

		void SetPath(string Path);

		void CreateTextFile(char* Path, 
						    vector <vector<double>> Data);

		string GetFilePath(string FileName);

		vector<vector<double>> GetPressureTimeHistory(char* Path); 
		vector<vector<double>> GetPressureTimeHistoryTabDelimited(char* Path,
																  int StartIndex,
																  int TimeLength,
																  int PressureLength);

		void PrintPressureTimeHistory(vector<vector<double>> const &PT);
};

#endif