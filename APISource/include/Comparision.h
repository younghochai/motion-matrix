#pragma once
#include "quaternion.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
struct CurveProperty
{
	float upperArmLength, LowerArmLength;
	float speed;
	float initialOrientationDeviation;


};
class Comparision {
	
	public:static float getAngularDistance(TVector3 q1, TVector3 q2);
	public:static float getDiffBtwTrajectory(char* usf1File, char* lsf1File, char* usf2File, char* lsf2File, int &percent, struct CurveProperty &Curveproperty);
	public:static void readCSV(istream &input, vector< vector<string> > &output, quaternion(&trajectory)[1024], int &count);
public:static void curveDiagnosis(quaternion usf1[], quaternion lsf1[], int noOfPoints,
	struct CurveProperty &Curveproperty);
public:static   void resetDiagnosis();
};
