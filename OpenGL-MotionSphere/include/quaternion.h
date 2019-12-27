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
#include <math.h>
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

	quaternion mutiplication(quaternion Q);
	TVec3 quternionMatrices(quaternion Q,  TVec3 vecPoint);
	void quaternionToEulerAngles(quaternion Q, TVec3& vecPoint);
	void setPrecisionOfValues();
	quaternion EulerAngleToQuaternion(double yaw, double pitch, double roll);

};

