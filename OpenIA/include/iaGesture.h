#pragma once
#include "iaquaternion.h"
#include "iaJointposdata.h"
class GestureRecognition
{
public:
	//skeleton allBoneJoints[2500];
	static bool _StartScan;
	static bool recordData;
	static bool initCalib;
	static bool isCalib;
	static bool readFile;

	static char* fileName;

	static int tFrameIndex;


public:
	void gesturerecog(JointPosition &jpos);



};

