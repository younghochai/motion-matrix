#include "iaSphereUtility.h"
#include <math.h>
#include <iostream>



void SphereUtility::getAngleAxisBWQuaternions(quaternion q_1, quaternion q_2, char * boneID)
{
	quaternion result = q_2.mutiplication(q_1.Inverse());
	float q0 = result.mData[3];
	float q1 = result.mData[0];
	float q2 = result.mData[1];
	float q3 = result.mData[2];

	float angle_rad = acos(q0) * 2;
	float angle_deg = angle_rad * 180 / PI;
	float x = q1 / sin(angle_rad / 2);
	float y = q2 / sin(angle_rad / 2);
	float z = q3 / sin(angle_rad / 2);

	std::cout << boneID << " rotates by " << angle_deg << " degree around (" << x << "," << y << "," << z << ")\n";
}

double SphereUtility::vecDistance(TVec3 v1, TVec3 v2)
{
	double result = sqrt(pow(v1._x - v2._x, 2) + pow(v1._y - v2._y, 2) + pow(v1._z - v2._z, 2));
	return result;
}
double SphereUtility::vecDotProduct(TVec3 v1, TVec3 v2)
{
	return ((v1._x*v2._x) + (v1._y*v2._y) + (v1._z*v2._z));
}


TVec3 SphereUtility::vecCrossProduct(TVec3 v1, TVec3 v2)
{
	TVec3 result;
	result._x = v1._y * v2._z - v1._z * v2._y;
	result._y = v1._x * v2._z - v1._z * v2._x;
	result._z = v1._x * v2._y - v1._y * v2._x;
	return result;
}

double vecDotProduct(TVec3 v1, TVec3 v2)
{
	return ((v1._x*v2._x) + (v1._y*v2._y) + (v1._z*v2._z));
}

double vecLength(TVec3 v)
{
	return sqrt(pow(v._x, 2) + pow(v._y, 2) + pow(v._z, 2));
}

void SphereUtility::vecNormalize(TVec3 &vec)
{
	float length = vecLength(vec);
	vec._x = vec._x / length;
	vec._y = vec._y / length;
	vec._z = vec._z / length;
}

void SphereUtility::vectorsToQuat()
{
	TVec3 v1 = { 0.0, 0.707106781, -0.707106781 };
	quaternion quatBody(-1.29947E-16, -0.707106781, 0.707106781, 1.41232E-32);
	//for (int index = 0; index < 10; index++)
	//{
	//	//printf("Bone-%d --- ", index);
	//	for (int jndex = 0; jndex < this->noOfFrames; jndex++)
	//	{
	//		TVec3 v2 = { this->vectors[jndex][index]._x,this->vectors[jndex][index]._y,this->vectors[jndex][index]._z };
	//		TVec3 cross = vecCrossProduct(v1, v2);
	//		double w = sqrt(pow(vecLength(v1), 2) * pow(vecLength(v2), 2)) + vecDotProduct(v1, v2);
	//		quaternion q = { cross._x, cross._y, cross._z, w };
	//		q.normalize();
	//		//printf("%f\t%f\t%f\t%f\n",q.mData[3],q.mData[0], q.mData[1], q.mData[2]);
	//	}
	//	//printf("----------------------------------\n");
	//	
	//}

	for (int index = 0; index < noOfFrames; index++)
	{
		//printf("Bone-%d --- ", index);
		for (int jndex = 0; jndex < 10; jndex++)
		{
			TVec3 v2 = { this->vectors[index][jndex]._x,this->vectors[index][jndex]._y,this->vectors[index][jndex]._z };
			TVec3 cross = vecCrossProduct(v1, v2);
			double w = 1 + vecDotProduct(v1, v2);
			quaternion q = { cross._x, cross._z, -cross._y, w };
			q.normalize();
			q = q.Inverse();
			q = quatBody.mutiplication(q);
			switch (jndex)
			{
			case 0: this->avatarData[index].b0 = q; break;
			case 1: this->avatarData[index].b1 = q; break;
			case 2: this->avatarData[index].b2 = q; break;
			case 3: this->avatarData[index].b3 = q; break;
			case 4: this->avatarData[index].b4 = q; break;
			case 5: this->avatarData[index].b5 = q; break;
			case 6: this->avatarData[index].b6 = q; break;
			case 7: this->avatarData[index].b7 = q; break;
			case 8: this->avatarData[index].b8 = q; break;
			case 9: this->avatarData[index].b9 = q; break;
			}

			//printf("%f %f %f %f ", q.mData[3], q.mData[0], q.mData[1], q.mData[2]);
		}
		//printf("\n");

	}
}

float SphereUtility::getTwistAngle(TVec3 w, quaternion q)
{
	TVec3 v = { startingVector._x,startingVector._y,startingVector._z };

	//find the cross product vw
	TVec3 vw = vecCrossProduct(v, w);

	// Normalize vw
	vecNormalize(vw);
	//float vwLength = vecLength(vw);
	//if (vwLength == 0) return 0;
	//vw._x = vw._x / vwLength;
	//vw._y = vw._y / vwLength;
	//vw._z = vw._z / vwLength;

	//find the dot product dotAngle
	float dotAngle = acos(vecDotProduct(v, w));

	//regrenate swing rotation qs
	quaternion qs = { vw._x*sin(dotAngle / 2),vw._y*sin(dotAngle / 2),vw._z*sin(dotAngle / 2),cos(dotAngle / 2) };

	//compute twist rotation qt using inversion
	quaternion qt = qs.Inverse().mutiplication(q);
	//quaternion qt = q.mutiplication(qs.Inverse());

	//***** Debugging ******//
	/*quaternion newQ = qs.mutiplication(qt);
	std::cout << "q = {" << q.mData[3] << "," << q.mData[0] << "," << q.mData[0] << "," << q.mData[0] << "} <- "
		"qs*qt = {" << newQ.mData[3] << "," << newQ.mData[0] << "," << newQ.mData[0] << "," << newQ.mData[0] << "} "<< std::endl;*/
		//***** Debugging ******//

		//return twist angle
	if (isnan(acos(qt.mData[3])))
		return (0.000001);
	else
		return (2*acos(qt.mData[3]));
}

void SphereUtility::printData()
{
	for (int i = 0; i < this->noOfFrames; i++)
	{
		std::cout << "b0 -> "
			<< this->vectors[i][0]._x << ","
			<< this->vectors[i][0]._y << ","
			<< this->vectors[i][0]._z << ","
			<< "b1 -> "
			<< this->vectors[i][1]._x << ","
			<< this->vectors[i][1]._y << ","
			<< this->vectors[i][1]._z << ","
			<< "b2 -> "
			<< this->vectors[i][2]._x << ","
			<< this->vectors[i][2]._y << ","
			<< this->vectors[i][2]._z << ","
			<< "b3 -> "
			<< this->vectors[i][3]._x << ","
			<< this->vectors[i][3]._y << ","
			<< this->vectors[i][3]._z << ","
			<< "b4 -> "
			<< this->vectors[i][4]._x << ","
			<< this->vectors[i][4]._y << ","
			<< this->vectors[i][4]._z << ","
			<< "b5 -> "
			<< this->vectors[i][5]._x << ","
			<< this->vectors[i][5]._y << ","
			<< this->vectors[i][5]._z << ","
			<< "b6 -> "
			<< this->vectors[i][6]._x << ","
			<< this->vectors[i][6]._y << ","
			<< this->vectors[i][6]._z << ","
			<< "b7 -> "
			<< this->vectors[i][7]._x << ","
			<< this->vectors[i][7]._y << ","
			<< this->vectors[i][7]._z << ","
			<< "b8 -> "
			<< this->vectors[i][8]._x << ","
			<< this->vectors[i][8]._y << ","
			<< this->vectors[i][8]._z << ","
			<< "b9 -> "
			<< this->vectors[i][9]._x << ","
			<< this->vectors[i][9]._y << ","
			<< this->vectors[i][9]._z << "," << std::endl;
	}

}

TVec3 SphereUtility::vecSLERP(TVec3& a, TVec3& b, const float t)
{
	TVec3 r;
	float t_ = 1 - t;
	float Wa, Wb;
	float theta = acos(a._x*b._x + a._y*b._y + a._z*b._z + 0);
	float sn = sin(theta);
	Wa = sin(t_*theta) / sn;
	Wb = sin(t*theta) / sn;
	r._x = Wa * a._x + Wb * b._x;
	r._y = Wa * a._y + Wb * b._y;
	r._z = Wa * a._z + Wb * b._z;

	//vecNormalize(r);

	return r;
}

void SphereUtility::readAvatarData(std::string fileName)
{
	int count = 0;
	int tCount = 0;
	//bool skip = true;
	std::ifstream _filestream(fileName);
	std::string _line;
	int _option;
	std::string _dummy;
	int lineCount = 0;

	while (std::getline(_filestream, _line))
	{
		//skip = !skip;
		std::stringstream _linestream;
		_linestream << _line;
		if (count == 0)
		{
			_linestream >> _line >> this->subOption;  count++; continue;
		}

		if (count == 1)
		{
			_linestream >> _line >> this->noOfFrames; count++; continue;
		}

		switch (this->subOption)
		{
		case 1:
			//if (!skip) {
				_linestream
					>> this->avatarData[lineCount].b0.mData[3] >> this->avatarData[lineCount].b0.mData[0] >> this->avatarData[lineCount].b0.mData[1] >> this->avatarData[lineCount].b0.mData[2]
					>> this->avatarData[lineCount].b1.mData[3] >> this->avatarData[lineCount].b1.mData[0] >> this->avatarData[lineCount].b1.mData[1] >> this->avatarData[lineCount].b1.mData[2]
					>> this->avatarData[lineCount].b2.mData[3] >> this->avatarData[lineCount].b2.mData[0] >> this->avatarData[lineCount].b2.mData[1] >> this->avatarData[lineCount].b2.mData[2]
					>> this->avatarData[lineCount].b3.mData[3] >> this->avatarData[lineCount].b3.mData[0] >> this->avatarData[lineCount].b3.mData[1] >> this->avatarData[lineCount].b3.mData[2]
					>> this->avatarData[lineCount].b4.mData[3] >> this->avatarData[lineCount].b4.mData[0] >> this->avatarData[lineCount].b4.mData[1] >> this->avatarData[lineCount].b4.mData[2]
					>> this->avatarData[lineCount].b5.mData[3] >> this->avatarData[lineCount].b5.mData[0] >> this->avatarData[lineCount].b5.mData[1] >> this->avatarData[lineCount].b5.mData[2]
					>> this->avatarData[lineCount].b6.mData[3] >> this->avatarData[lineCount].b6.mData[0] >> this->avatarData[lineCount].b6.mData[1] >> this->avatarData[lineCount].b6.mData[2]
					>> this->avatarData[lineCount].b7.mData[3] >> this->avatarData[lineCount].b7.mData[0] >> this->avatarData[lineCount].b7.mData[1] >> this->avatarData[lineCount].b7.mData[2]
					>> this->avatarData[lineCount].b8.mData[3] >> this->avatarData[lineCount].b8.mData[0] >> this->avatarData[lineCount].b8.mData[1] >> this->avatarData[lineCount].b8.mData[2]
					>> this->avatarData[lineCount].b9.mData[3] >> this->avatarData[lineCount].b9.mData[0] >> this->avatarData[lineCount].b9.mData[1] >> this->avatarData[lineCount].b9.mData[2];

				lineCount++;
			//}
			break;

		case 2:

			_linestream
				>> this->avatarData[lineCount].b0.mData[3] >> this->avatarData[lineCount].b0.mData[0] >> this->avatarData[lineCount].b0.mData[1] >> this->avatarData[lineCount].b0.mData[2]
				>> this->avatarData[lineCount].b2.mData[3] >> this->avatarData[lineCount].b2.mData[0] >> this->avatarData[lineCount].b2.mData[1] >> this->avatarData[lineCount].b2.mData[2]
				>> this->avatarData[lineCount].b3.mData[3] >> this->avatarData[lineCount].b3.mData[0] >> this->avatarData[lineCount].b3.mData[1] >> this->avatarData[lineCount].b3.mData[2]
				>> this->avatarData[lineCount].b4.mData[3] >> this->avatarData[lineCount].b4.mData[0] >> this->avatarData[lineCount].b4.mData[1] >> this->avatarData[lineCount].b4.mData[2]
				>> this->avatarData[lineCount].b5.mData[3] >> this->avatarData[lineCount].b5.mData[0] >> this->avatarData[lineCount].b5.mData[1] >> this->avatarData[lineCount].b5.mData[2];
			lineCount++;
			break;

		case 3:

			_linestream
				>> this->avatarData[lineCount].b0.mData[3] >> this->avatarData[lineCount].b0.mData[0] >> this->avatarData[lineCount].b0.mData[1] >> this->avatarData[lineCount].b0.mData[2]
				>> this->avatarData[lineCount].b6.mData[3] >> this->avatarData[lineCount].b6.mData[0] >> this->avatarData[lineCount].b6.mData[1] >> this->avatarData[lineCount].b6.mData[2]
				>> this->avatarData[lineCount].b7.mData[3] >> this->avatarData[lineCount].b7.mData[0] >> this->avatarData[lineCount].b7.mData[1] >> this->avatarData[lineCount].b7.mData[2]
				>> this->avatarData[lineCount].b8.mData[3] >> this->avatarData[lineCount].b8.mData[0] >> this->avatarData[lineCount].b8.mData[1] >> this->avatarData[lineCount].b8.mData[2]
				>> this->avatarData[lineCount].b9.mData[3] >> this->avatarData[lineCount].b9.mData[0] >> this->avatarData[lineCount].b9.mData[1] >> this->avatarData[lineCount].b9.mData[2];

			lineCount++;
			break;

		default:
			break;

		}
	}
}

void SphereUtility::writeAvatarData(std::string fileName)
{
	std::ofstream editFile;
	editFile.open(fileName);
	editFile << "FULLBODY\t1\n";
	editFile << "Frames:\t"<<this->noOfFrames<<"\n";
	for (int i = 0; i < this->noOfFrames; i++)
	{
		editFile << this->avatarData[i].b0 << "\t" << this->avatarData[i].b1 << "\t" << this->avatarData[i].b2 << "\t" << this->avatarData[i].b3 << "\t" << this->avatarData[i].b4 << "\t"
					<< this->avatarData[i].b5 << "\t" << this->avatarData[i].b6 << "\t" << this->avatarData[i].b7 << "\t" << this->avatarData[i].b8 << "\t" << this->avatarData[i].b9 << "\n";
	}
	editFile.close();
}

void SphereUtility::calTraj(quaternion parent, quaternion child, TVec3 &parentVec, TVec3 &childVec, float &tAngleParent, float &tAngleChild)
{
	quaternion BodyQuat(1.29947E-16, 0.707106781, -0.707106781, 1.41232E-32);

	/*bodyFrameVec._x = 0;
	bodyFrameVec._y = 0;
	bodyFrameVec._z = -1;*/

	//quaternion tempQuat1 = BodyQuat.mutiplication(parent);

	//quaternion tempQuat2 = BodyQuat.mutiplication(child);//Case-2 usf_q

	quaternion tempQuat1 = parent;
	quaternion tempQuat2 = child;//Case-2 usf_q

	quaternion lq = tempQuat2;
	quaternion uq = tempQuat1;
	quaternion vQuat(startingVector._x, startingVector._y, startingVector._z, 0);

	quaternion lw = lq.mutiplication(vQuat.mutiplication(lq.Inverse()));//QVQ-1
	quaternion uw = uq.mutiplication(vQuat.mutiplication(uq.Inverse()));//QVQ-1


	parentVec = { (float)uw.mData[0], (float)uw.mData[1], (float)uw.mData[2] };
	childVec = { (float)lw.mData[0], (float)lw.mData[1], (float)lw.mData[2] };

	SphereUtility su;
	tAngleParent = su.getTwistAngle(parentVec, uq);
	tAngleChild = su.getTwistAngle(childVec, lq);
}

//
//computing quad points in anticlock wise direction
//
void SphereUtility::getQuadPoints(float lattitude, float longitude, TVec3 *points)
{
	quaternion longituteQuat, lattitudeQuat, combinedQuat;
	for (int i = 0; i < 4; i++)
	{
		//i == 0 is bottom right
		if (i == 1) // top right
		{
			longitude = longitude + 1;
		}
		if (i == 2) // top left
		{
			lattitude = lattitude + 1;
		}
		if (i == 3) // bottom left
		{
			longitude = longitude - 1;
		}
		//longitude = longitude / 2;
		// computing quaternion for vertical swing
		longituteQuat.mData[3] = cos(longitude * PI / 180);
		longituteQuat.mData[0] = 1 * (sin(longitude * PI / 180));
		longituteQuat.mData[1] = 0  * (sin(longitude * PI / 180));
		longituteQuat.mData[2] = 0  * (sin(longitude * PI / 180));

		//computing quternion for lateral swing
		lattitudeQuat.mData[3] = cos(lattitude * PI / 180);
		lattitudeQuat.mData[0] = 0 * (sin(lattitude * PI / 180));
		lattitudeQuat.mData[1] = 0 * (sin(lattitude * PI / 180));
		lattitudeQuat.mData[2] = 1 * (sin(lattitude * PI / 180));

		//combining vertical and lateral swings
		combinedQuat = lattitudeQuat.mutiplication(longituteQuat);

		combinedQuat = combinedQuat.mutiplication(quaternion(0, 0, -1, 0).mutiplication(combinedQuat.Inverse()));
		points[i]._x = combinedQuat.mData[0];
		points[i]._y = combinedQuat.mData[1];
		points[i]._z = combinedQuat.mData[2];
	}
}

void SphereUtility::normalizeAvatar(struct Avatar& avatar)
{
	avatar.b0.normalize();
	avatar.b1.normalize();
	avatar.b2.normalize();
	avatar.b3.normalize();
	avatar.b4.normalize();
	avatar.b5.normalize();
	avatar.b6.normalize();
	avatar.b7.normalize();
	avatar.b8.normalize();
	avatar.b9.normalize();	
}

void SphereUtility::fullBodytoXYZ()
{
	TVec3 b0Vec = { 0,0,0 }, b1Vec = { 0,0,0 };
	float b0Angle = 0, b1Angle = 0;

	for (int i = 0; i < this->noOfFrames; i++)
	{
		quaternion vQuat(startingVector._x, startingVector._y, startingVector._z, 0);
		//b0
		quaternion uTransfBodyQuat = this->avatarData[i].b0.mutiplication(vQuat.mutiplication(this->avatarData[i].b0.Inverse()));


		this->vectors[i][0]._x = uTransfBodyQuat.mData[0];
		this->vectors[i][0]._y = uTransfBodyQuat.mData[1];
		this->vectors[i][0]._z = uTransfBodyQuat.mData[2];
		this->twistAngles[i][0] = this->getTwistAngle({ uTransfBodyQuat.mData[0] ,uTransfBodyQuat.mData[1] ,uTransfBodyQuat.mData[2] }, uTransfBodyQuat);
		/*quaternion parent = this->avatarData[i].b1;
		quaternion child = this->avatarData[i].b1;*/
		//b1
		calTraj(this->avatarData[i].b1, this->avatarData[i].b1, b0Vec, b1Vec, b0Angle, b1Angle);


		this->vectors[i][1]._x = b0Vec._x;
		this->vectors[i][1]._y = b0Vec._y;
		this->vectors[i][1]._z = b0Vec._z;
		this->twistAngles[i][1] = b1Angle;

		//b2 and b3
		calTraj(avatarData[i].b2, avatarData[i].b3, b0Vec, b1Vec, b0Angle, b1Angle);

		this->vectors[i][2]._x = b0Vec._x;
		this->vectors[i][2]._y = b0Vec._y;
		this->vectors[i][2]._z = b0Vec._z;
		this->twistAngles[i][2] = b0Angle;

		this->vectors[i][3]._x = b1Vec._x;
		this->vectors[i][3]._y = b1Vec._y;
		this->vectors[i][3]._z = b1Vec._z;
		this->twistAngles[i][3] = b1Angle;

		// b4 and b5
		calTraj(this->avatarData[i].b4, this->avatarData[i].b5, b0Vec, b1Vec, b0Angle, b1Angle);

		this->vectors[i][4]._x = b0Vec._x;
		this->vectors[i][4]._y = b0Vec._y;
		this->vectors[i][4]._z = b0Vec._z;
		this->twistAngles[i][4] = b0Angle;

		this->vectors[i][5]._x = b1Vec._x;
		this->vectors[i][5]._y = b1Vec._y;
		this->vectors[i][5]._z = b1Vec._z;
		this->twistAngles[i][5] = b1Angle;

		// b6 and b7

		calTraj(this->avatarData[i].b6, this->avatarData[i].b7, b0Vec, b1Vec, b0Angle, b1Angle);

		this->vectors[i][6]._x = b0Vec._x;
		this->vectors[i][6]._y = b0Vec._y;
		this->vectors[i][6]._z = b0Vec._z;
		this->twistAngles[i][6] = b0Angle;

		this->vectors[i][7]._x = b1Vec._x;
		this->vectors[i][7]._y = b1Vec._y;
		this->vectors[i][7]._z = b1Vec._z;
		this->twistAngles[i][7] = b1Angle;

		// b8 and b9
		calTraj(this->avatarData[i].b8, this->avatarData[i].b9, b0Vec, b1Vec, b0Angle, b1Angle);

		this->vectors[i][8]._x = b0Vec._x;
		this->vectors[i][8]._y = b0Vec._y;
		this->vectors[i][8]._z = b0Vec._z;
		this->twistAngles[i][8] = b0Angle;

		this->vectors[i][9]._x = b1Vec._x;
		this->vectors[i][9]._y = b1Vec._y;
		this->vectors[i][9]._z = b1Vec._z;
		this->twistAngles[i][9] = b1Angle;
	}
}
void SphereUtility::upperBodytoXYZ()
{
	TVec3 b0Vec, b1Vec;
	float b0Angle, b1Angle;

	for (int i = 0; i < noOfFrames; i++)
	{
		quaternion vQuat(startingVector._x, startingVector._y, startingVector._z, 0);
		//b0
		quaternion uTransfBodyQuat = this->avatarData[i].b0.mutiplication(vQuat.mutiplication(this->avatarData[i].b0.Inverse()));


		this->vectors[i][0]._x = uTransfBodyQuat.mData[0];
		this->vectors[i][0]._y = uTransfBodyQuat.mData[1];
		this->vectors[i][0]._z = uTransfBodyQuat.mData[2];



		//b2 and b3
		calTraj(this->avatarData[i].b2, this->avatarData[i].b3, b0Vec, b1Vec, b0Angle, b1Angle);

		this->vectors[i][2]._x = b0Vec._x;
		this->vectors[i][2]._y = b0Vec._y;
		this->vectors[i][2]._z = b0Vec._z;
		this->twistAngles[i][2] = b0Angle;

		this->vectors[i][3]._x = b1Vec._x;
		this->vectors[i][3]._y = b1Vec._y;
		this->vectors[i][3]._z = b1Vec._z;
		this->twistAngles[i][3] = b1Angle;

		// b4 and b5
		calTraj(this->avatarData[i].b4, this->avatarData[i].b5, b0Vec, b1Vec, b0Angle, b1Angle);

		this->vectors[i][4]._x = b0Vec._x;
		this->vectors[i][4]._y = b0Vec._y;
		this->vectors[i][4]._z = b0Vec._z;
		this->twistAngles[i][4] = b0Angle;

		this->vectors[i][5]._x = b1Vec._x;
		this->vectors[i][5]._y = b1Vec._y;
		this->vectors[i][5]._z = b1Vec._z;
		this->twistAngles[i][5] = b1Angle;

	}
}

void SphereUtility::lowerBodytoXYZ()
{
	TVec3 b0Vec, b1Vec;
	float b0Angle, b1Angle;

	for (int i = 0; i < noOfFrames; i++)
	{
		quaternion vQuat(startingVector._x, startingVector._y, startingVector._z, 0);
		//b0
		quaternion uTransfBodyQuat = this->avatarData[i].b0.mutiplication(vQuat.mutiplication(this->avatarData[i].b0.Inverse()));


		this->vectors[i][0]._x = uTransfBodyQuat.mData[0];
		this->vectors[i][0]._y = uTransfBodyQuat.mData[1];
		this->vectors[i][0]._z = uTransfBodyQuat.mData[2];


		/// b6 and b7

		calTraj(this->avatarData[i].b6, this->avatarData[i].b7, b0Vec, b1Vec, b0Angle, b1Angle);

		this->vectors[i][6]._x = b0Vec._x;
		this->vectors[i][6]._y = b0Vec._y;
		this->vectors[i][6]._z = b0Vec._z;
		this->twistAngles[i][6] = b0Angle;

		this->vectors[i][7]._x = b1Vec._x;
		this->vectors[i][7]._y = b1Vec._y;
		this->vectors[i][7]._z = b1Vec._z;
		this->twistAngles[i][7] = b1Angle;

		// b8 and b9
		calTraj(this->avatarData[i].b8, this->avatarData[i].b9, b0Vec, b1Vec, b0Angle, b1Angle);

		this->vectors[i][8]._x = b0Vec._x;
		this->vectors[i][8]._y = b0Vec._y;
		this->vectors[i][8]._z = b0Vec._z;
		this->twistAngles[i][8] = b0Angle;

		this->vectors[i][9]._x = b1Vec._x;
		this->vectors[i][9]._y = b1Vec._y;
		this->vectors[i][9]._z = b1Vec._z;
		this->twistAngles[i][9] = b1Angle;
	}
}


