
/*
*This work is dual-licensed under BSD-3 and Apache License 2.0. 

*You can choose between one of them if you use this work.

*SPDX-License-Identifier: BSD-3-Clause OR Apache License 2.0

*/

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

