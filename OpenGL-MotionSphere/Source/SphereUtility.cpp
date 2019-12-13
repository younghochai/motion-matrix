#include "SphereUtility.h"
#include <math.h>
#include <iostream>

TVec3 bodyFrameVec;


void SphereUtility:: getAngleAxisBWQuaternions(quaternion q_1, quaternion q_2, char * boneID)
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

		std::cout <<boneID <<" rotates by "<< angle_deg << " degree around (" << x << "," << y << "," << z << ")\n";
}

double SphereUtility::vecDistance(TVec3 v1, TVec3 v2)
{
	double result = sqrt(pow(v1._x-v2._x,2) + pow(v1._y-v2._y,2) + pow(v1._z-v2._z,2));
	return result;
}

TVec3 vecCrossProduct(TVec3 v1, TVec3 v2)
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
	return sqrt(pow(v._x, 2)+pow(v._y, 2)+pow(v._z, 2));
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
void SphereUtility::readAvatarData(std::string fileName)
{
	int count = 0;
	int tCount = 0;

	std::ifstream _filestream(fileName);
	std::string _line;
	int _option;
	std::string _dummy;
	int lineCount = 0;

	while (std::getline(_filestream, _line))
	{
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

void calTraj(quaternion parent, quaternion child, TVec3 &parentVec, TVec3 &childVec)
{
	quaternion BodyQuat(1.29947E-16, 0.707106781, -0.707106781, 1.41232E-32);
	
	bodyFrameVec._x = 1.29947E-16;
	bodyFrameVec._y = 0.707106781;
	bodyFrameVec._z = -0.707106781;

	quaternion tempQuat2 = BodyQuat.mutiplication(parent);

	quaternion tempQuat1 = tempQuat2.mutiplication(child);//Case-2 usf_q


	quaternion lTransfBodyQuat = tempQuat1;
	quaternion uTransfBodyQuat = tempQuat2;
	quaternion vQuat(bodyFrameVec._x, bodyFrameVec._y, bodyFrameVec._z, 0);

	lTransfBodyQuat = lTransfBodyQuat.mutiplication(vQuat.mutiplication(lTransfBodyQuat.Inverse()));
	uTransfBodyQuat = uTransfBodyQuat.mutiplication(vQuat.mutiplication(uTransfBodyQuat.Inverse()));


	parentVec = { (float)uTransfBodyQuat.mData[0], (float)uTransfBodyQuat.mData[1], (float)uTransfBodyQuat.mData[2] };
	childVec = { (float)lTransfBodyQuat.mData[0], (float)lTransfBodyQuat.mData[1], (float)lTransfBodyQuat.mData[2] };
}

void SphereUtility::fullBodytoXYZ()
{
	TVec3 b0Vec, b1Vec;

	for (int i = 0; i < noOfFrames; i++) 
	{
		quaternion vQuat(bodyFrameVec._x, bodyFrameVec._y, bodyFrameVec._z, 0);
		//b0
		quaternion uTransfBodyQuat = this->avatarData[i].b0.mutiplication(vQuat.mutiplication(this->avatarData[i].b0.Inverse()));


		this->vectors[i][0]._x = uTransfBodyQuat.mData[0];
		this->vectors[i][0]._y = uTransfBodyQuat.mData[1];
		this->vectors[i][0]._z = uTransfBodyQuat.mData[2];
		
		//b1
		calTraj(this->avatarData[i].b1, this->avatarData[i].b1, b0Vec, b1Vec);


		this->vectors[i][1]._x = b0Vec._x;
		this->vectors[i][1]._y = b0Vec._y;
		this->vectors[i][1]._z = b0Vec._z;


		//b2 and b3
		calTraj(this->avatarData[i].b2, this->avatarData[i].b3, b0Vec, b1Vec);

		this->vectors[i][2]._x = b0Vec._x;
		this->vectors[i][2]._y = b0Vec._y;
		this->vectors[i][2]._z = b0Vec._z;

		this->vectors[i][3]._x = b1Vec._x;
		this->vectors[i][3]._y = b1Vec._y;
		this->vectors[i][3]._z = b1Vec._z;

		// b4 and b5
		calTraj(this->avatarData[i].b4, this->avatarData[i].b5, b0Vec, b1Vec);

		this->vectors[i][4]._x = b0Vec._x;
		this->vectors[i][4]._y = b0Vec._y;
		this->vectors[i][4]._z = b0Vec._z;

		this->vectors[i][5]._x = b1Vec._x;
		this->vectors[i][5]._y = b1Vec._y;
		this->vectors[i][5]._z = b1Vec._z;

		// b6 and b7
	
		calTraj(this->avatarData[i].b6, this->avatarData[i].b7, b0Vec, b1Vec);

		this->vectors[i][6]._x = b0Vec._x;
		this->vectors[i][6]._y = b0Vec._y;
		this->vectors[i][6]._z = b0Vec._z;

		this->vectors[i][7]._x = b1Vec._x;
		this->vectors[i][7]._y = b1Vec._y;
		this->vectors[i][7]._z = b1Vec._z;

		// b8 and b9
		calTraj(this->avatarData[i].b8, this->avatarData[i].b9, b0Vec, b1Vec);

		this->vectors[i][8]._x = b0Vec._x;
		this->vectors[i][8]._y = b0Vec._y;
		this->vectors[i][8]._z = b0Vec._z;

		this->vectors[i][9]._x = b1Vec._x;
		this->vectors[i][9]._y = b1Vec._y;
		this->vectors[i][9]._z = b1Vec._z;
	}
}
void SphereUtility::upperBodytoXYZ()
{
	TVec3 b0Vec, b1Vec;

	for (int i = 0; i < noOfFrames; i++) 
	{
		quaternion vQuat(bodyFrameVec._x, bodyFrameVec._y, bodyFrameVec._z, 0);
		//b0
		quaternion uTransfBodyQuat = this->avatarData[i].b0.mutiplication(vQuat.mutiplication(this->avatarData[i].b0.Inverse()));


		this->vectors[i][0]._x = uTransfBodyQuat.mData[0];
		this->vectors[i][0]._y = uTransfBodyQuat.mData[1];
		this->vectors[i][0]._z = uTransfBodyQuat.mData[2];
		


		//b2 and b3
		calTraj(this->avatarData[i].b2, this->avatarData[i].b3, b0Vec, b1Vec);

		this->vectors[i][2]._x = b0Vec._x;
		this->vectors[i][2]._y = b0Vec._y;
		this->vectors[i][2]._z = b0Vec._z;

		this->vectors[i][3]._x = b1Vec._x;
		this->vectors[i][3]._y = b1Vec._y;
		this->vectors[i][3]._z = b1Vec._z;

		// b4 and b5
		calTraj(this->avatarData[i].b4, this->avatarData[i].b5, b0Vec, b1Vec);

		this->vectors[i][4]._x = b0Vec._x;
		this->vectors[i][4]._y = b0Vec._y;
		this->vectors[i][4]._z = b0Vec._z;

		this->vectors[i][5]._x = b1Vec._x;
		this->vectors[i][5]._y = b1Vec._y;
		this->vectors[i][5]._z = b1Vec._z;

	}
}

void SphereUtility::lowerBodytoXYZ()
{
	TVec3 b0Vec, b1Vec;

	for (int i = 0; i < noOfFrames; i++) 
	{
		quaternion vQuat(bodyFrameVec._x, bodyFrameVec._y, bodyFrameVec._z, 0);
		//b0
		quaternion uTransfBodyQuat = this->avatarData[i].b0.mutiplication(vQuat.mutiplication(this->avatarData[i].b0.Inverse()));


		this->vectors[i][0]._x = uTransfBodyQuat.mData[0];
		this->vectors[i][0]._y = uTransfBodyQuat.mData[1];
		this->vectors[i][0]._z = uTransfBodyQuat.mData[2];
		

		// b6 and b7
	
		calTraj(this->avatarData[i].b6, this->avatarData[i].b7, b0Vec, b1Vec);

		this->vectors[i][6]._x = b0Vec._x;
		this->vectors[i][6]._y = b0Vec._y;
		this->vectors[i][6]._z = b0Vec._z;

		this->vectors[i][7]._x = b1Vec._x;
		this->vectors[i][7]._y = b1Vec._y;
		this->vectors[i][7]._z = b1Vec._z;

		// b8 and b9
		calTraj(this->avatarData[i].b8, this->avatarData[i].b9, b0Vec, b1Vec);

		this->vectors[i][8]._x = b0Vec._x;
		this->vectors[i][8]._y = b0Vec._y;
		this->vectors[i][8]._z = b0Vec._z;

		this->vectors[i][9]._x = b1Vec._x;
		this->vectors[i][9]._y = b1Vec._y;
		this->vectors[i][9]._z = b1Vec._z;
	}
}


