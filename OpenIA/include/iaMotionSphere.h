#pragma once
#include <math.h>
#include <stdio.h>
#include <glut.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <freeglut.h>
#include <ctime>

#include "tgaload.h"
#include "iaSphereUtility.h"

class MotionSphere {
	public:
		static int sphereID;

		static bool keyPressed;
		static char* fileName;
		int MotionSphere::maxWidth; //1800;
		int MotionSphere::maxHeight; //900;
		int MotionSphere::minWidth;
		int MotionSphere::minHeight;
		SphereUtility* su;
			
	public:
		MotionSphere(int minWidth, int minHeight, int maxWidth, int maxHeight)
		{
			this->minWidth = minWidth;
			this->maxWidth = maxWidth;
			this->minHeight = minHeight;
			this->maxHeight = maxHeight;
		}
		MotionSphere()
		{
			this->minWidth = 0;
			this->maxWidth = 900;
			this->minHeight = 0;
			this->maxHeight = 900;
		}
		int sphereMainLoop(MotionSphere newms, char* windowName);		// Similar to glut main loop (AvatarMotion function)
		//int sphereMainLoop2(MotionSphere newms, char* windowName);
		void setSphereUtility(SphereUtility su)
		{
			this->su = &su;
		}

		static void setFileName(char* f)
		{
		fileName = f;
		}
		

};