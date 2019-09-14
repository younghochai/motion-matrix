#include <windows.h>

#include <stdlib.h> 
#include "quaternion.h"
#include "Comparision.h"
#include <stdio.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <iterator>
#include <sstream>


#define SAMPLESIZE 100.0f
#define PI 3.14159265359
typedef vector< vector<string> > csvVector;

csvVector csvData;
ofstream myfile;
string a;

ostream& operator<<(ostream& os, quaternion& q)
{
	cout << q.mData[3] << "," << q.mData[0] << "," << q.mData[1] << "," << q.mData[2];
	return os;
}

ostream& operator<<(ostream& os, TVector3& q)
{
	cout << q._x << "," << q._y << "," << q._z;
	return os;
}
void Comparision:: readCSV(istream &input, vector< vector<string> > &output, quaternion (&trajectory)[1024], int &count)
{
	csvData.clear();
	count = 0;
	string csvLine;
	quaternion prevQuatInverse, currentQuat, axisQuat;
	/*std::ofstream rpyFile;
	rpyFile.open("rypFile.csv");*/
	// read every line from the stream
	while (getline(input, csvLine))
	{
		istringstream csvStream(csvLine);
		vector<string> csvColumn;
		string csvElement;
		// read every element from the line that is seperated by commas
		// and put it into the vector or strings
		while (getline(csvStream, csvElement, ','))
		{
			csvColumn.push_back(csvElement);
		}
		output.push_back(csvColumn);
	}

	for (csvVector::iterator i = csvData.begin(); i != csvData.end(); ++i)
	{
		quaternion currentQuat(stod(i->at(1)), stod(i->at(2)), stod(i->at(3)), stod(i->at(0)));
		trajectory[count] = currentQuat;
		count++;
	}
	//rpyFile.close();
}

float Comparision::getDiffBtwTrajectory(char* usf1File, char* lsf1File, char* usf2File, char* lsf2File, int &percent)
{
	int count1, count2;
	quaternion usf1[1024], lsf1[1024], usf2[1024], lsf2[1024];
	percent = 0;
	// Read First Trajectory (UpperArm and Lower Arm) into usf1 and lsf1
	fstream file1(usf1File, ios::in);
	if (!file1.is_open())
	{
		cout << "File not found!\n";
		return 1;
	}
	readCSV(file1, csvData, usf1, count1);

	fstream file2(lsf1File, ios::in);
	if (!file2.is_open())
	{
		cout << "File not found!\n";
		return 1;
	}
	readCSV(file2, csvData, lsf1, count1);

	// Read Second Trajectory (UpperArm and Lower Arm) into usf2 and lsf2
	fstream file3(usf2File, ios::in);
	if (!file3.is_open())
	{
		cout << "File not found!\n";
		return 1;
	}
	readCSV(file3, csvData, usf2, count2);

	fstream file4(lsf2File, ios::in);
	if (!file4.is_open())
	{
		cout << "File not found!\n";
		return 1;
	}
	readCSV(file4, csvData, lsf2, count2);
	
	//Linearly Distributed Indexed Sampling 
	float increment1 = (float)count1 / SAMPLESIZE;
	float increment2 = (float)count2 / SAMPLESIZE;
	float  val1 = 0, val2 = 0;
	float sumOfDistance = 0;
	
	// Initializing the initial Vector
	TVector3 tempVec;
	quaternion BodyQuat(1.29947E-16, 0.707106781, -0.707106781, 1.41232E-32);
	tempVec._x = 0;
	tempVec._y = 0;
	tempVec._z = 0;

	float q0 = BodyQuat.mData[3];
	float q1 = BodyQuat.mData[0];
	float q2 = BodyQuat.mData[1];
	float q3 = BodyQuat.mData[2];

	float angle_rad = acos(q0) * 2;
	float angle_deg = angle_rad * 180 / PI;
	float x = q1 / sin(angle_rad / 2);
	float y = q2 / sin(angle_rad / 2);
	float z = q3 / sin(angle_rad / 2);

	float fnorm = sqrt(x*x + y * y + z * z);

	tempVec._x = x / fnorm;
	tempVec._y = y / fnorm;
	tempVec._z = z / fnorm;

	for (int i = 0; i < SAMPLESIZE; i++)
	{
		float minDist = 999;
				

		for (int j = 0; j < SAMPLESIZE; j++)
		{
			/*if (j >= count2)
			{
				cout << "Skip:" << i << endl;
				continue;
			}*/
			quaternion tempQuat = BodyQuat.mutiplication(usf1[(int)round(j)]);
			quaternion tempQuat1 = tempQuat.mutiplication(lsf1[(int)round(j)]);//Case-2 usf_q
																				  //cout << usf1[(int)round(val1)] << "," << lsf1[(int)round(val1)] << endl;
			TVector3 TransfBodyQuat1 = tempQuat1.quternionMatrices(tempQuat1, tempVec);

			tempQuat = BodyQuat.mutiplication(usf2[(int)round(val2)]);
			quaternion tempQuat2 = tempQuat.mutiplication(lsf2[(int)round(val2)]);//Case-2 usf_q
																				  //cout << usf2[(int)round(val2)] << "," << lsf2[(int)round(val2)] << endl;
			TVector3 TransfBodyQuat2 = tempQuat2.quternionMatrices(tempQuat2, tempVec);
			/////////////////////////////////////////////////////////////
			float angularDist = getAngularDistance(TransfBodyQuat1, TransfBodyQuat2);
			if (angularDist < minDist)
				minDist = angularDist;			
		}
		val1 = val1++;
		val2 = increment2 + val2;
		if(minDist!=999)
		sumOfDistance = sumOfDistance + minDist;

		cout << "MinDistance:"<<i<<"->"<< minDist << endl;
		if (minDist < 0.15)
			percent++;
	}
	val1 = 0; val2 = 0;

	cout << "-------------------XXXXXXX-----------------"  << endl;
	//for (int i = 0; i < SAMPLESIZE; i++)
	//{
	//	val1 = increment1 + val1;
	//	val2 = increment2 + val2;
	//	

	//	//////////////////Computing vector difference//////////////////////
	//	
	//	//quaternion tempQuat = BodyQuat.mutiplication(sf_q);

	//	quaternion tempQuat = BodyQuat.mutiplication(usf1[(int)round(val1)]);
	//	quaternion tempQuat1 = tempQuat.mutiplication(lsf1[(int)round(val1)]);//Case-2 usf_q
	//	//cout << usf1[(int)round(val1)] << "," << lsf1[(int)round(val1)] << endl;
	//	TVector3 TransfBodyQuat1 = tempQuat1.quternionMatrices(tempQuat1, tempVec);

	//	tempQuat = BodyQuat.mutiplication(usf2[(int)round(val2)]);
	//	quaternion tempQuat2 = tempQuat.mutiplication(lsf2[(int)round(val2)]);//Case-2 usf_q
	//	//cout << usf2[(int)round(val2)] << "," << lsf2[(int)round(val2)] << endl;
	//	TVector3 TransfBodyQuat2 = tempQuat2.quternionMatrices(tempQuat2, tempVec);
	//	/////////////////////////////////////////////////////////////
	//	float angularDist = getAngularDistance(TransfBodyQuat1, TransfBodyQuat2);
	//	sumOfDistance = sumOfDistance + angularDist;
	//	//cout << TransfBodyQuat1 <<"," << TransfBodyQuat2 <<","<< getAngularDistance(TransfBodyQuat1, TransfBodyQuat2) << endl;
	//}

	file1.close();
	file2.close();
	file3.close();
	file4.close();
	return sumOfDistance / SAMPLESIZE;
}

float Comparision:: getAngularDistance(TVector3 v1, TVector3 v2)
{
	float distance = sqrt((v1._x - v2._x)*(v1._x - v2._x) +
		(v1._y - v2._y)*(v1._y - v2._y) +
		(v1._z - v2._z)*(v1._z - v2._z));
	return distance;
}