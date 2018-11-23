#pragma once

//XSens Header Include
#include <xsens/xsmutex.h>
#include <xsensdeviceapi.h> 

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

class XsensConnection
{
public:
	bool isRunning = false;
	bool bxMTdisconnect = true;
	bool stop_and_restart_everything = false;
	float ax[4]; 
	float ax2[4]; 
	float ax3[4]; 

	float r_ax[4];
	float r_ax2[4];
	float r_ax3[4];

	
	bool waitForConnections = true;
	bool newDataAvailable = false;
	bool closeMtW_Succes = false;

public:
	void Intialize();

	bool xmtConnect();

};