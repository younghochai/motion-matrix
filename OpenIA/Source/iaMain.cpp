#pragma once
#define WIN32_LEAN_AND_MEAN
/*
*This work is dual-licensed under BSD-3 and Apache License 2.0. 

*You can choose between one of them if you use this work.

*SPDX-License-Identifier: BSD-3-Clause OR Apache License 2.0

*/
#include <thread>

#include "iaMotionSphere.h"
#include "iaPositionTracking.h"
#include "iaAcquireGesture.h"
#include "iaVitruvianAvatar.h"


using namespace std;

iaAcquireGesture iaAcquire;// Object for IMU sensor 
PositionTracking poseTrack;// Object for LiDAR sensor data connectivity
VitruvianAvatar vAvatar;// 

//objecct for MotionSphere
SphereUtility su;
char* MotionSphere::fileName;
char* PositionTracking::fileName;

// Renders tracked sensor data on motionsphere.
void motionSphere()
{
	MotionSphere ms(0, 0, 900, 900);
	ms.setSphereUtility(su);
	ms.sphereMainLoop(ms, "Sphere 1");
}

//This holds LiDAR sensor connectivity and computes object (persons) position using LiDAR-data

//Connect and get lidar-1 data
void getLiDARdata1()
{
	poseTrack.LiDARDataReader1();
}

//Connect and get lidar-2 data
void getLiDARdata2()
{
	poseTrack.LiDARDataReader2();
}

//pose detection by using the pose tracking
void poseTracking()
{
	poseTrack.positionDetection(vAvatar);
}

//To connect and start IMU to get data from the sensors
void XSensDataReader()
{
	iaAcquire.startXsensData();
	poseTrack.saveQautData();
}

//It creates and renders a virtual Vitruvian Avatar. It also simulates the Avatar using sensor data.
void startAvatar()
{
	VitruvianAvatar::humanHeight = 172.0;
	vAvatar.initializeVetruvianVtkAvatar();
	vAvatar.startVetruvianAvatar();
}

// main method
int main()
{	
	thread t1(XSensDataReader);
	thread t2(startAvatar);
	thread t3(getLiDARdata1);
	thread t4(getLiDARdata2);
	thread t5(poseTracking);
	thread t6(motionSphere);
	

	t1.join();
	t2.join();
	t3.join();
	t4.join();
	t5.join();
	t6.join();	
}
