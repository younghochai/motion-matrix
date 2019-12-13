#pragma once
#include "quaternion.h"
#include <fstream>
#include <string>
#include <sstream>
#define PI 3.14159265359


struct Avatar {
	quaternion b0, b1, b2, b3, b4, b5, b6, b7, b8, b9;
	quaternion prv_b0, prv_b1, prv_b2, prv_b3, prv_b4,
		prv_b5, prv_b6, prv_b7, prv_b8, prv_b9;
};

//struct TVec3 {
//	float _x;
//	float _y;
//	float _z;
//};

class SphereUtility {
public:
	Avatar avatarData[1024];
	TVec3 vectors[1024][10];
	int noOfFrames;
	int subOption;
public:
	void readAvatarData(std::string fileName);
	void fullBodytoXYZ();
	void upperBodytoXYZ();
	void lowerBodytoXYZ();
	void printData();
	void getAngleAxisBWQuaternions(quaternion q1, quaternion q2, char * boneID);
	double vecDistance(TVec3 v1, TVec3 v2);
	void vectorsToQuat();
	
};

