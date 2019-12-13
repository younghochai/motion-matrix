/*
*	BSD 3-Clause License
*
*	Copyright (c) 2018, OpenAI, VELab, GSAIM, Chung-Ang University.
*
*	All rights reserved.
*
*	Redistribution and use in source and binary forms, with or without modification, are permitted provided
*  that the following conditions are met:
*
*	1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
*	   following disclaimer.
*
*	2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
*     the following disclaimer in the documentation and/or other materials provided with the distribution.
*
*	3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific prior written permission.
*
*	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
*	WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
*	PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
*	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*	TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
*	NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
*	OF SUCH DAMAGE.
*
*/


#pragma once
#include "quaternion.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>


/** \brief @Motion Trajectory Daignosis 
*  \note This class enables  Comparision between two motion trajectories, Calaculate angular distance.
*  \authors: Adithya Balasubramanyam <adithyakoundinya@gmail.com>
*			 Ashok Kumar Patil <ashokpatil03@hotmail.com>
*
*/

using namespace std;
struct CurveProperty
{
	float upperArmLength, LowerArmLength;
	float speed;
	float initialOrientationDeviation;


};
class Comparision {
	
	public:static float getAngularDistance(TVec3 q1, TVec3 q2);
	public:static float getDiffBtwTrajectory(char* usf1File, char* lsf1File, char* usf2File, char* lsf2File, int &percent, struct CurveProperty &Curveproperty);
	public:static void readCSV(istream &input, vector< vector<string> > &output, quaternion(&trajectory)[1024], int &count);
public:static void curveDiagnosis(quaternion usf1[], quaternion lsf1[], int noOfPoints,
	struct CurveProperty &Curveproperty);
public:static   void resetDiagnosis();
};
