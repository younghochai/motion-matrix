#pragma once
#include "quaternion.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

class Comparision {
	
	public:static float getAngularDistance(TVector3 q1, TVector3 q2);
	public:static float getDiffBtwTrajectory(char* usf1File, char* lsf1File, char* usf2File, char* lsf2File);
	public:static void readCSV(istream &input, vector< vector<string> > &output, quaternion(&trajectory)[1024], int &count);
};
