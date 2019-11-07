#pragma once

//XSens Header Include
#include <xsmutex.h>
#include <xsensdeviceapi.h> 
#include <xstypes/xstime.h>
#include "conio.h"			
#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>

//Quaternion processing
#include "quaternion.h"


struct XsensIMU {
	quaternion b0, b1, b2, b3, b4, b5, b6, b7, b8, b9;
};

class XsensConnection
{
public:
	bool isRunning = false;
	bool bxMTdisconnect = true;
	bool stop_and_restart_everything = false;
		
	bool waitForConnections = true;
	bool newDataAvailable = false;
	bool closeMtW_Succes = false;

	XsDevicePtrArray mtwDevices;

	quaternion xsQInit = { 0,0,0,1 };
	struct XsensIMU xsIMU = { xsQInit,xsQInit,xsQInit,xsQInit,xsQInit,xsQInit,xsQInit,xsQInit,xsQInit,xsQInit };


public:
	void Intialize();

	bool xmtConnect();

};