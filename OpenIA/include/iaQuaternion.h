
/*
*This work is dual-licensed under BSD-3 and Apache License 2.0. 

*You can choose between one of them if you use this work.

*SPDX-License-Identifier: BSD-3-Clause OR Apache License 2.0

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
#include <math.h>
#include <iostream>

using namespace std;

struct TVec3 {
	double _x;
	double _y;
	double _z;
};

struct IntVec3 {
	int _x;
	int _y;
	int _z;
};

/** \brief @Quaternion Functions
*  \note This class enables  Quaternion: Multiplication, Normalization, Inverse, conversion to- Eulers, Matrix.
*  \authors: Ashok Kumar Patil <ashokpatil03@hotmail.com>
*			 Adithya Balasubramanyam <adithyakoundinya@gmail.com>
*
*/

class quaternion
{

public:
double mData[4];
double axisangle[4];
public:
	quaternion();
	quaternion(double x, double y, double z, double w);
	quaternion(TVec3 complex, double real);
	~quaternion();

	double  real() const;
	void real(double r);

	TVec3 complex() const;
	void complex(const TVec3& c);

	double norm(quaternion q);
	double norm();
	void normalize();
	//quaternion normalize(quaternion q);
	quaternion Conjugate(void) const;
	quaternion Conjugate(quaternion Q) ;
	quaternion Conjugate();
	quaternion Inverse(quaternion Q) ;
	quaternion Inverse();
	friend ostream & operator << (ostream &out, const quaternion &q);
	quaternion mutiplication(quaternion Q);
	TVec3 quternionMatrices(quaternion Q,  TVec3 vecPoint);
	void quaternionToEulerAngles(quaternion Q, TVec3& vecPoint);
	void setPrecisionOfValues();
	quaternion EulerAngleToQuaternion(double yaw, double pitch, double roll);
	quaternion SLERP(quaternion& a, quaternion& b, const float t);
	void quattoaxisangle()
	{
		double pi = 3.141592653589793238462643383279502884e+0;
		
				double angle, x_axis, y_axis, z_axis;
				double q0, q1, q2, q3;
				q0 = mData[3];
				q1 = mData[0];
				q2 = mData[2];// rotate axis
				q3 = -mData[1];
				double s;
				s = sqrt(1 - q0 * q0);
				angle = 2 * acos(q0);
				angle = angle * 180 / pi;
				//Z-> Y ->  X
				if (s < 0.001)
				{
					x_axis = q1;
					y_axis = q2;
					z_axis = q3;
				}
				else {
					x_axis = q1 / s;
					y_axis = q2 / s;
					z_axis = q3 / s;
				}

				axisangle[0] = angle;
				axisangle[1] = x_axis;
				axisangle[2] = y_axis;
				axisangle[3] = z_axis;
		
	}
};

