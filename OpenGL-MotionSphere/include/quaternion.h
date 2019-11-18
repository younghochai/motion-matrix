#pragma once
#include <math.h>
struct TVector3 {
	double _x;
	double _y;
	double _z;
};


class quaternion
{

public:
double mData[4];

public:
	quaternion();
	quaternion(double x, double y, double z, double w);
	quaternion(TVector3 complex, double real);
	~quaternion();

	double  real() const;
	void real(double r);

	TVector3 complex() const;
	void complex(const TVector3& c);

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
	TVector3 quternionMatrices(quaternion Q,  TVector3 vecPoint);
	void quaternionToEulerAngles(quaternion Q, TVector3& vecPoint);
	void setPrecisionOfValues();
	quaternion EulerAngleToQuaternion(double yaw, double pitch, double roll);

};

