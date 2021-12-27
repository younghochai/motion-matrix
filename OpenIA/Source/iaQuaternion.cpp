/*

*This work is dual-licensed under BSD-3 and Apache License 2.0. You can choose between one of them if you use this work.

* SPDX-License-Identifier: BSD-3-Clause OR Apache License 2.0

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

#include "iaquaternion.h"


#define M_PI 3.14159;

/** \brief @Quaternion Functions
*  \note This class enables  Quaternion: Multiplication, Normalization, Inverse, conversion to- Eulers, Matrix.
*  \authors: Ashok Kumar Patil <ashokpatil03@hotmail.com>
*			 Adithya Balasubramanyam <adithyakoundinya@gmail.com>
*			
*/

ostream & operator << (ostream &out, const quaternion &q)
{
	out << q.mData[3] << "\t" << q.mData[0] << "\t" << q.mData[1] << "\t" << q.mData[2];
	return out;
}

quaternion::quaternion(){

	mData[0] = mData[1] = mData[2] = 0;
	mData[3] = 1;
}

quaternion::quaternion(double x, double y, double z, double w) {
	mData[0] = x; mData[1] = y; mData[2] = z;
	mData[3] = w;
}

quaternion::quaternion(TVec3 complex, double real) {
	mData[0] = complex._x; mData[1] = complex._y; mData[2] = complex._z;
	mData[3] = real;
}
quaternion::~quaternion(){

}

double quaternion::norm(quaternion q) {

	return sqrt(q.mData[0]* q.mData[0]+ q.mData[1] * q.mData[1] + q.mData[2] * q.mData[2] + q.mData[3] * q.mData[3] );
}

double quaternion::norm() {

	return sqrt(this->mData[0] * this->mData[0] + this->mData[1] * this->mData[1] + this->mData[2] * this->mData[2] + this->mData[3] * this->mData[3]);
}

//quaternion quaternion::normalize(quaternion q) {
//	
//	quaternion qn;
//	qn.mData[0] =q.mData[0] / norm();
//	qn.mData[1] =q.mData[1] / norm();
//	qn.mData[2] =q.mData[2] / norm();
//	qn.mData[3] =q.mData[3] / norm();
//
//	return q;
//}

//slerping between two quaternions
quaternion quaternion::SLERP(quaternion& a, quaternion& b, const float t)
{
	quaternion r;
	float t_ = 1 - t;
	float Wa, Wb;
	float theta = acos(a.mData[0]*b.mData[0] + a.mData[1] *b.mData[1] + a.mData[2] *b.mData[2] + a.mData[3] *b.mData[3]);
	float sn = sin(theta);
	Wa = sin(t_*theta) / sn;
	Wb = sin(t*theta) / sn;
	r.mData[0] = Wa * a.mData[0] + Wb * b.mData[0];
	r.mData[1] = Wa * a.mData[1] + Wb * b.mData[1];
	r.mData[2] = Wa * a.mData[2] + Wb * b.mData[2];
	r.mData[3] = Wa * a.mData[3] + Wb * b.mData[3];
	r.normalize();
	return r;
}

void quaternion::normalize() {
		
	this->mData[0] = this->mData[0] / norm();
	this->mData[1] = this->mData[1] / norm();
	this->mData[2] = this->mData[2] / norm();
	this->mData[3] = this->mData[3] / norm();	
}

double  quaternion::real() const{
	return mData[3];
}

void quaternion::real(double r) {
	mData[3] = r;
}

TVec3 quaternion::complex() const{	
	
	TVec3 data;
	data._x = mData[0]; data._y = mData[1]; data._z = mData[2];	
	return data;
}

void quaternion::complex(const TVec3& c) {
	mData[0] = c._x; mData[1] = c._y;  mData[2] = c._z;
}

quaternion quaternion::Conjugate() const {
		
	return quaternion(complex(),real());
}

quaternion quaternion::Conjugate(quaternion Q)  {
	
	return quaternion(-Q.mData[0], -Q.mData[1], -Q.mData[2], Q.mData[3]);
}

quaternion quaternion::Conjugate() {

	return quaternion(-mData[0], -mData[1], -mData[2], mData[3]);
}

quaternion quaternion::Inverse(quaternion Q)  {
	quaternion Inv;

	double nrm = norm(Q);
	Inv = Conjugate(Q);

	Inv.mData[0] = Inv.mData[0] / nrm;
	Inv.mData[1] = Inv.mData[1] / nrm;
	Inv.mData[2] = Inv.mData[2] / nrm;
	Inv.mData[3] = Inv.mData[3] / nrm;
	
	return Inv;
}

quaternion quaternion::Inverse() {
	quaternion Inv;

	double nrm = norm();
	Inv = Conjugate();

	Inv.mData[0] = Inv.mData[0] / nrm;
	Inv.mData[1] = Inv.mData[1] / nrm;
	Inv.mData[2] = Inv.mData[2] / nrm;
	Inv.mData[3] = Inv.mData[3] / nrm;

	return Inv;
}

quaternion quaternion::mutiplication(quaternion Q) {
	
	quaternion mul;
	double w1 = mData[3], w2 = Q.mData[3];
	double x1 = mData[0], x2 = Q.mData[0];
	double y1 = mData[1], y2 = Q.mData[1];
	double z1 = mData[2], z2 = Q.mData[2];

	//Hari
	mul.mData[3] = mData[3] * Q.mData[3] - mData[0] * Q.mData[0] - mData[1] * Q.mData[1] - mData[2] * Q.mData[2];
	mul.mData[0] = mData[3] * Q.mData[0] + mData[0] * Q.mData[3] + mData[1] * Q.mData[2] - mData[2] * Q.mData[1];
	mul.mData[1] = mData[3] * Q.mData[1] + mData[1] * Q.mData[3] + mData[2] * Q.mData[0] - mData[0] * Q.mData[2];
	mul.mData[2] = mData[3] * Q.mData[2] + mData[2] * Q.mData[3] + mData[0] * Q.mData[1] - mData[1] * Q.mData[0];
	//std::cout << "HariOM : W" << mul.mData[3] << "X:" << mul.mData[0] << "Y:" << mul.mData[1] << "Z:" << mul.mData[2] << std::endl;
	
	//Stackoverflow
	/*mul.mData[3] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
	mul.mData[0] = w1*x2 + w2*x1 - y1*z2 + y2*z1;
	mul.mData[1] = w1*y2 + w2*y1 + x1*z2 - x2*z1;
	mul.mData[2] = w1*z2 + w2*z1 - x1*y2 + x2*y1;*/

	//Quaternion web
	/*mul.mData[3] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
	mul.mData[0] = w1*x2 + x1*w2 - y1*z2 + z1*y2;
	mul.mData[1] = w1*y2 + x1*z2 + y1*w2 - z1*x2;
	mul.mData[2] = w1*z2 - x1*y2 + y1*x2 + z1*w2;*/

	//Wiki
	/*mul.mData[3] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
	mul.mData[0] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
	mul.mData[1] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
	mul.mData[2] = w1*z2 + x1*y2 - y1*x2 + z1*w2;*/
	//std::cout << "Update : W" << mul.mData[3] << "X:" << mul.mData[0] << "Y:" << mul.mData[1] << "Z:" << mul.mData[2] << std::endl;
	return mul;
}

TVec3 quaternion::quternionMatrices(quaternion Q, TVec3 vecPoint)
{
	TVec3 w;
	
	w._x = (2 * (Q.mData[3] * Q.mData[3]) - 1 + 2 * (Q.mData[0] * Q.mData[0])) * vecPoint._x 
		+ (2 * (Q.mData[0] * Q.mData[1]) - 2 * (Q.mData[3] * Q.mData[2])) * vecPoint._y 
		+ (2 * (Q.mData[0] * Q.mData[2]) + 2 * (Q.mData[3] * Q.mData[1])) * vecPoint._z;
	
	w._y = (2 * (Q.mData[0] * Q.mData[1]) + 2 *(Q.mData[3] * Q.mData[2])) * vecPoint._x 
		+ (2 * (Q.mData[3] * Q.mData[3])-1 + 2 * (Q.mData[1]*Q.mData[1])) * vecPoint._y
		+ (2 * (Q.mData[1] * Q.mData[2])-2*(Q.mData[3] *Q.mData[0])) * vecPoint._z;

	w._z = (2 * (Q.mData[0] * Q.mData[2]) - 2 * (Q.mData[3] * Q.mData[1])) * vecPoint._x
		+ (2 * (Q.mData[1] * Q.mData[2])  + 2 * (Q.mData[3] * Q.mData[0])) * vecPoint._y
		+ (2 * (Q.mData[3] * Q.mData[3]) - 1 + 2 * (Q.mData[2] * Q.mData[2])) * vecPoint._z;

	return w;
}

void quaternion::quaternionToEulerAngles(quaternion q, TVec3& RPY)
{
	// roll (x-axis rotation)
	double sinr_cosp = 2.0 * (q.mData[3] * q.mData[0] + q.mData[1] * q.mData[2]);
	double cosr_cosp = 1.0 - 2.0 * (q.mData[0] * q.mData[0] + q.mData[1] * q.mData[1]);
	RPY._x = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2.0 * (q.mData[3] * q.mData[1] - q.mData[3] * q.mData[0]);
	if (fabs(sinp) >= 1)
	//	RPY._y = (double) copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	//else
		RPY._y = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2.0 * (q.mData[3] * q.mData[3] + q.mData[0] * q.mData[1]);
	double cosy_cosp = 1.0 - 2.0 * (q.mData[1] * q.mData[1] + q.mData[3] * q.mData[3]);
	RPY._z = atan2(siny_cosp, cosy_cosp);

}

void quaternion::setPrecisionOfValues()
{
	this->mData[3] = (float) ((int)(100 * this->mData[3]) / 100.0);
	this->mData[0] = (float) ((int)(100 * this->mData[0]) / 100.0);
	this->mData[1] = (float) ((int)(100 * this->mData[1]) / 100.0);
	this->mData[2] = (float) ((int)(100 * this->mData[2]) / 100.0);
}

quaternion quaternion::EulerAngleToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
	// Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);

	quaternion q;
	q.mData[3] = cy * cp * cr + sy * sp * sr;
	q.mData[0] = cy * cp * sr - sy * sp * cr;
	q.mData[1] = sy * cp * sr + cy * sp * cr;
	q.mData[2] = sy * cp * cr - cy * sp * sr;
	return q;
}