#include "quaternion.h"

#include <iostream>

#define M_PI 3.14159;

quaternion::quaternion(){

	mData[0] = mData[1] = mData[2] = 0;
	mData[3] = 1;
}

quaternion::quaternion(double x, double y, double z, double w) {
	mData[0] = x; mData[1] = y; mData[2] = z;
	mData[3] = w;
}

quaternion::quaternion(TVector3 complex, double real) {
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

TVector3 quaternion::complex() const{	
	
	TVector3 data;
	data._x = mData[0]; data._y = mData[1]; data._z = mData[2];	
	return data;
}

void quaternion::complex(const TVector3& c) {
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

TVector3 quaternion::quternionMatrices(quaternion Q, TVector3 vecPoint)
{
	TVector3 w;
	
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

void quaternion::quaternionToEulerAngles(quaternion q, TVector3& RPY)
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