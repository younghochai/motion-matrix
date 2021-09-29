#pragma once
#include "iaquaternion.h"
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
	TVec3 startingVector = { 0,0,-1 };
	Avatar avatarData[2000];
	TVec3 vectors[2000][10];
	float twistAngles[2000][10];
	int noOfFrames;
	int subOption;
	TVec3 tempVec = {0,0,-1};
public:
	void readAvatarData(std::string fileName);
	void writeAvatarData(std::string fileName);
	void fullBodytoXYZ();
	void upperBodytoXYZ();
	void lowerBodytoXYZ();
	void printData();
	void getAngleAxisBWQuaternions(quaternion q1, quaternion q2, char * boneID);
	double vecDistance(TVec3 v1, TVec3 v2);
	void vectorsToQuat();
	float getTwistAngle(TVec3 w, quaternion q);
	TVec3 vecSLERP(TVec3& a, TVec3& b, const float t);
	TVec3 vecCrossProduct(TVec3 v1, TVec3 v2);
	double vecDotProduct(TVec3 v1, TVec3 v2);
	void vecNormalize(TVec3 &vec);
	void calTraj(quaternion parent, quaternion child, TVec3 &parentVec, TVec3 &childVec, float &tAngleParent, float &tAngleChild);

	void normalizeAvatar(struct Avatar& avatar);
};

