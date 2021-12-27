#include "iaAcquireGesture.h"

//Quaternion raw data
quaternion QuatData_RightUpperArm, QuatData_head, QuatData_RightLowerArm, QuatData_Pelvis, QuatData_LeftUpperArm, QuatData_LeftLowerArm;
quaternion QuatData_RightUpperLeg, QuatData_RightLowerLeg, QuatData_LeftUpperLeg, QuatData_LeftLowerLeg;

//Quaternion first inverse
quaternion firstInvQuat_RightUpperArm, firstInvQuat_RightLowerArm;
quaternion firstInvQuat_RightUpperLeg, firstInvQuat_RightLowerLeg;
quaternion firstInvQuat_LeftUpperArm, firstInvQuat_LeftLowerArm;
quaternion firstInvQuat_LeftUpperLeg, firstInvQuat_LeftLowerLeg;
quaternion firstPlvCalib, firstHeadCalib;

quaternion qInit = { 0,0,0,1 };

XsensConnection connectXS;

bool iaAcquireGesture::calibIMU;

ofstream rawIMUDataFile;
// finding the values of quaternion height
double quatHeight(quaternion data)
{

	double pi = 3.141592653589793238462643383279502884e+0;

	double s;

	//double vectorM[3];
	double vectorM[3] = { 0,	0,	-1 };

	double Rotaxis_X, Rotaxis_Y, Rotaxis_Z;
	double q0, q1, q2, q3;

	q0 = data.mData[3];
	q1 = data.mData[0];
	q2 = data.mData[1];
	q3 = data.mData[2];

	double height;

	Rotaxis_X = vectorM[0] * (2 * q0*q0 - 1 + 2 * q1* q1) + vectorM[1] * (2 * q2*q1 - 2 * q0* q3) + vectorM[2] * (2 * q1*q3 + 2 * q0* q2);
	Rotaxis_Y = vectorM[0] * (2 * q1*q2 + 2 * q0*q3) + vectorM[1] * (2 * q0*q0 - 1 + 2 * q2*q2) + vectorM[2] * (2 * q2*q3 - 2 * q0*q1);
	Rotaxis_Z = vectorM[0] * (2 * q1*q3 - 2 * q0* q2) + vectorM[1] * (2 * q2*q3 + 2 * q0* q1) + vectorM[2] * (2 * q0*q0 - 1 + 2 * q3* q3);

	s = sqrt(Rotaxis_X*Rotaxis_X + Rotaxis_Y * Rotaxis_Y + Rotaxis_Z * Rotaxis_Z);

	Rotaxis_X = Rotaxis_X / s;
	Rotaxis_Y = Rotaxis_Y / s;
	Rotaxis_Z = Rotaxis_Z / s;	

	return Rotaxis_Z;
}

//get the raw values
Avatar iaAcquireGesture::getRawQ()
{
	bool DataAvailable = connectXS.newDataAvailable;
	
	Avatar rawAvatar = { qInit, qInit, qInit,  qInit,  qInit, qInit, qInit, qInit, qInit, qInit };

	if (DataAvailable)
	{
		QuatData_Pelvis = connectXS.xsIMU.b0;

		QuatData_head = connectXS.xsIMU.b1;

		QuatData_RightUpperArm = connectXS.xsIMU.b2;
		QuatData_RightLowerArm = connectXS.xsIMU.b3;

		QuatData_LeftUpperArm = connectXS.xsIMU.b4;
		QuatData_LeftLowerArm = connectXS.xsIMU.b5;

		QuatData_RightUpperLeg = connectXS.xsIMU.b6;
		QuatData_RightLowerLeg = connectXS.xsIMU.b7;

		QuatData_LeftUpperLeg = connectXS.xsIMU.b8;
		QuatData_LeftLowerLeg = connectXS.xsIMU.b9;

		rawAvatar = { QuatData_Pelvis, QuatData_head, QuatData_RightUpperArm,  QuatData_RightLowerArm,  QuatData_LeftUpperArm, QuatData_LeftLowerArm
		, QuatData_RightUpperLeg, QuatData_RightLowerLeg, QuatData_LeftUpperLeg, QuatData_LeftLowerLeg };

		return rawAvatar;
	}
	else
		return  rawAvatar;
}

void iaAcquireGesture::caliberateQSF()
{
	if (iaAcquireGesture::calibIMU)
	{
		getRawQ();

		std::cout << "AttentionPose:" << std::endl;

		firstPlvCalib = QuatData_Pelvis.Inverse();
		std::cout << "Pelvis Quat:\t\t" << QuatData_Pelvis.mData[3] << "\t" << QuatData_Pelvis.mData[0] << "\t" << QuatData_Pelvis.mData[1] << "\t" << QuatData_Pelvis.mData[2] << std::endl;

		firstHeadCalib = QuatData_head.Inverse();
		std::cout << "Head Quat:\t\t" << QuatData_head.mData[3] << "\t" << QuatData_head.mData[0] << "\t" << QuatData_head.mData[1] << "\t" << QuatData_head.mData[2] << std::endl;

		firstInvQuat_RightUpperArm = QuatData_RightUpperArm.Inverse();
		std::cout << "Right Upper-Arm Quat:\t" << QuatData_RightUpperArm.mData[3] << "\t" << QuatData_RightUpperArm.mData[0] << "\t" << QuatData_RightUpperArm.mData[1] << "\t" << QuatData_RightUpperArm.mData[2] << std::endl;

		firstInvQuat_RightLowerArm = QuatData_RightLowerArm.Inverse();
		std::cout << "Right Lower-Arm Quat:\t" << QuatData_RightLowerArm.mData[3] << "\t" << QuatData_RightLowerArm.mData[0] << "\t" << QuatData_RightLowerArm.mData[1] << "\t" << QuatData_RightLowerArm.mData[2] << std::endl;

		firstInvQuat_LeftUpperArm = QuatData_LeftUpperArm.Inverse();
		std::cout << "Left Upper-Arm Quat:\t" << QuatData_LeftUpperArm.mData[3] << "\t" << QuatData_LeftUpperArm.mData[0] << "\t" << QuatData_LeftUpperArm.mData[1] << "\t" << QuatData_LeftUpperArm.mData[2] << std::endl;

		firstInvQuat_LeftLowerArm = QuatData_LeftLowerArm.Inverse();
		std::cout << "Left Lower-Arm Quat:\t" << QuatData_LeftLowerArm.mData[3] << "\t" << QuatData_LeftLowerArm.mData[0] << "\t" << QuatData_LeftLowerArm.mData[1] << "\t" << QuatData_LeftLowerArm.mData[2] << std::endl;

		firstInvQuat_RightUpperLeg = QuatData_RightUpperLeg.Inverse();
		std::cout << "Right Upper-Leg Quat:\t" << QuatData_RightUpperLeg.mData[3] << "\t" << QuatData_RightUpperLeg.mData[0] << "\t" << QuatData_RightUpperLeg.mData[1] << "\t" << QuatData_RightUpperLeg.mData[2] << std::endl;

		firstInvQuat_RightLowerLeg = QuatData_RightLowerLeg.Inverse();
		std::cout << "Right Lower-Leg Quat:\t" << QuatData_RightLowerLeg.mData[3] << "\t" << QuatData_RightLowerLeg.mData[0] << "\t" << QuatData_RightLowerLeg.mData[1] << "\t" << QuatData_RightLowerLeg.mData[2] << std::endl;

		firstInvQuat_LeftUpperLeg = QuatData_LeftUpperLeg.Inverse();
		std::cout << "Left Upper-Leg Quat:\t" << QuatData_LeftUpperLeg.mData[3] << "\t" << QuatData_LeftUpperLeg.mData[0] << "\t" << QuatData_LeftUpperLeg.mData[1] << "\t" << QuatData_LeftUpperLeg.mData[2] << std::endl;

		firstInvQuat_LeftLowerLeg = QuatData_LeftLowerLeg.Inverse();
		std::cout << "Left Lower-Leg Quat:\t" << QuatData_LeftLowerLeg.mData[3] << "\t" << QuatData_LeftLowerLeg.mData[0] << "\t" << QuatData_LeftLowerLeg.mData[1] << "\t" << firstInvQuat_LeftLowerArm.mData[2] << std::endl;

		//---------------------------------

		time_t curr_time;
		curr_time = time(NULL);
		tm *tm_local = localtime(&curr_time);

		ofstream avatarDataFile;

		char fileName[1024];

		sprintf_s(fileName, ".\\SkeletonData\\CalibIMUData-00%d-%d%d%d.txt", 0, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
		avatarDataFile.open(fileName);

		avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << 0 << "\n";
		
			avatarDataFile
				<< firstPlvCalib.mData[3] << "\t" << firstPlvCalib.mData[0] << "\t" << firstPlvCalib.mData[1] << "\t" << firstPlvCalib.mData[2] << "\t"
				<< firstHeadCalib.mData[3] << "\t" << firstHeadCalib.mData[0] << "\t" << firstHeadCalib.mData[1] << "\t" << firstHeadCalib.mData[2] << "\t"
				<< firstInvQuat_RightUpperArm.mData[3] << "\t" << firstInvQuat_RightUpperArm.mData[0] << "\t" << firstInvQuat_RightUpperArm.mData[1] << "\t" << firstInvQuat_RightUpperArm.mData[2] << "\t"
				<< firstInvQuat_RightLowerArm.mData[3] << "\t" << firstInvQuat_RightLowerArm.mData[0] << "\t" << firstInvQuat_RightLowerArm.mData[1] << "\t" << firstInvQuat_RightLowerArm.mData[2] << "\t"
				<< firstInvQuat_LeftUpperArm.mData[3] << "\t" << firstInvQuat_LeftUpperArm.mData[0] << "\t" << firstInvQuat_LeftUpperArm.mData[1] << "\t" << firstInvQuat_LeftUpperArm.mData[2] << "\t"
				<< firstInvQuat_LeftLowerArm.mData[3] << "\t" << firstInvQuat_LeftLowerArm.mData[0] << "\t" << firstInvQuat_LeftLowerArm.mData[1] << "\t" << firstInvQuat_LeftLowerArm.mData[2] << "\t"
				<< firstInvQuat_RightUpperLeg.mData[3] << "\t" << firstInvQuat_RightUpperLeg.mData[0] << "\t" << firstInvQuat_RightUpperLeg.mData[1] << "\t" << firstInvQuat_RightUpperLeg.mData[2] << "\t"
				<< firstInvQuat_RightLowerLeg.mData[3] << "\t" << firstInvQuat_RightLowerLeg.mData[0] << "\t" << firstInvQuat_RightLowerLeg.mData[1] << "\t" << firstInvQuat_RightLowerLeg.mData[2] << "\t"
				<< firstInvQuat_LeftUpperLeg.mData[3] << "\t" << firstInvQuat_LeftUpperLeg.mData[0] << "\t" << firstInvQuat_LeftUpperLeg.mData[1] << "\t" << firstInvQuat_LeftUpperLeg.mData[2] << "\t"
				<< firstInvQuat_LeftLowerLeg.mData[3] << "\t" << firstInvQuat_LeftLowerLeg.mData[0] << "\t" << firstInvQuat_LeftLowerLeg.mData[1] << "\t" << firstInvQuat_LeftLowerLeg.mData[2] << "\n";
		
		//avatarDataFile.close();
		//---------------------------------
		iaAcquireGesture::calibIMU = false;
	}	
}

// get the gesture value
Avatar iaAcquireGesture::getSFQ()
{
	Avatar myAvatar;

	myAvatar.b0 = QuatData_Pelvis.mutiplication(firstPlvCalib);
	myAvatar.b1 = myAvatar.b0.Inverse().mutiplication(QuatData_head.mutiplication(firstHeadCalib));
	myAvatar.b2 = QuatData_head.mutiplication(firstHeadCalib).Inverse().mutiplication(QuatData_RightUpperArm.mutiplication(firstInvQuat_RightUpperArm));
	myAvatar.b3 = QuatData_RightUpperArm.mutiplication(firstInvQuat_RightUpperArm).Inverse().mutiplication(QuatData_RightLowerArm.mutiplication(firstInvQuat_RightLowerArm));
	myAvatar.b4 = QuatData_head.mutiplication(firstHeadCalib).Inverse().mutiplication(QuatData_LeftUpperArm.mutiplication(firstInvQuat_LeftUpperArm));
	myAvatar.b5 = QuatData_LeftUpperArm.mutiplication(firstInvQuat_LeftUpperArm).Inverse().mutiplication(QuatData_LeftLowerArm.mutiplication(firstInvQuat_LeftLowerArm));
	myAvatar.b6 = myAvatar.b0.Inverse().mutiplication(QuatData_RightUpperLeg.mutiplication(firstInvQuat_RightUpperLeg));
	myAvatar.b7 = QuatData_RightUpperLeg.mutiplication(firstInvQuat_RightUpperLeg).Inverse().mutiplication(QuatData_RightLowerLeg.mutiplication(firstInvQuat_RightLowerLeg));
	myAvatar.b8 = myAvatar.b0.Inverse().mutiplication(QuatData_LeftUpperLeg.mutiplication(firstInvQuat_LeftUpperLeg));
	myAvatar.b9 = QuatData_LeftUpperLeg.mutiplication(firstInvQuat_LeftUpperLeg).Inverse().mutiplication(QuatData_LeftLowerLeg.mutiplication(firstInvQuat_LeftLowerLeg));

	myAvatar.b0.normalize();
	myAvatar.b1.normalize();
	myAvatar.b2.normalize();
	myAvatar.b3.normalize();
	myAvatar.b4.normalize();
	myAvatar.b5.normalize();
	myAvatar.b6.normalize();
	myAvatar.b7.normalize();
	myAvatar.b8.normalize();
	myAvatar.b9.normalize();

	return myAvatar;
}

Avatar iaAcquireGesture::getFirstInvQuat()
{
	Avatar myAvatar;

	myAvatar.b0 = QuatData_Pelvis.mutiplication(firstPlvCalib);
	myAvatar.b1 = (QuatData_head.mutiplication(firstHeadCalib));//myAvatar.b0.Inverse().mutiplication(QuatData_head.mutiplication(firstHeadCalib));
	myAvatar.b2 = (QuatData_RightUpperArm.mutiplication(firstInvQuat_RightUpperArm));//myAvatar.b0.Inverse().mutiplication(QuatData_RightUpperArm.mutiplication(firstInvQuat_RightUpperArm));
	myAvatar.b3 = (QuatData_RightLowerArm.mutiplication(firstInvQuat_RightLowerArm));//myAvatar.b0.Inverse().mutiplication(QuatData_RightLowerArm.mutiplication(firstInvQuat_RightLowerArm));
	myAvatar.b4 = (QuatData_LeftUpperArm.mutiplication(firstInvQuat_LeftUpperArm));//myAvatar.b0.Inverse().mutiplication(QuatData_LeftUpperArm.mutiplication(firstInvQuat_LeftUpperArm));
	myAvatar.b5 = (QuatData_LeftLowerArm.mutiplication(firstInvQuat_LeftLowerArm));//myAvatar.b0.Inverse().mutiplication(QuatData_LeftLowerArm.mutiplication(firstInvQuat_LeftLowerArm));
	myAvatar.b6 = (QuatData_RightUpperLeg.mutiplication(firstInvQuat_RightUpperLeg));//myAvatar.b0.Inverse().mutiplication(QuatData_RightUpperLeg.mutiplication(firstInvQuat_RightUpperLeg));
	myAvatar.b7 = (QuatData_RightLowerLeg.mutiplication(firstInvQuat_RightLowerLeg));//myAvatar.b0.Inverse().mutiplication(QuatData_RightLowerLeg.mutiplication(firstInvQuat_RightLowerLeg));
	myAvatar.b8 = (QuatData_LeftUpperLeg.mutiplication(firstInvQuat_LeftUpperLeg));//myAvatar.b0.Inverse().mutiplication(QuatData_LeftUpperLeg.mutiplication(firstInvQuat_LeftUpperLeg));
	myAvatar.b9 = (QuatData_LeftLowerLeg.mutiplication(firstInvQuat_LeftLowerLeg));//myAvatar.b0.Inverse().mutiplication(QuatData_LeftLowerLeg.mutiplication(firstInvQuat_LeftLowerLeg));

	myAvatar.b0.normalize();
	myAvatar.b1.normalize();
	myAvatar.b2.normalize();
	myAvatar.b3.normalize();
	myAvatar.b4.normalize();
	myAvatar.b5.normalize();
	myAvatar.b6.normalize();
	myAvatar.b7.normalize();
	myAvatar.b8.normalize();
	myAvatar.b9.normalize();

	return myAvatar;
}

void iaAcquireGesture::calculateHeight(double &RuLeg, double &RlLeg, double &LuLeg, double &LlLeg)
{
	RuLeg = quatHeight(QuatData_RightUpperLeg.mutiplication(firstInvQuat_RightUpperLeg));
	RlLeg = quatHeight(QuatData_RightLowerLeg.mutiplication(firstInvQuat_RightLowerLeg));
	LuLeg = quatHeight(QuatData_LeftUpperLeg.mutiplication(firstInvQuat_LeftUpperLeg));
	LlLeg = quatHeight(QuatData_LeftLowerLeg.mutiplication(firstInvQuat_LeftLowerLeg));	
}

void iaAcquireGesture::getXsensData()
{
	connectXS.waitForConnections = false;
	connectXS.isRunning = true;
	connectXS.bxMTdisconnect = false;
	connectXS.xsIMU = { qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit };
}

void iaAcquireGesture::startXsensData()
{

	bool stop_restart = connectXS.stop_and_restart_everything;

	while (!stop_restart) {
		bool MTdisconnect = connectXS.bxMTdisconnect;
		MTdisconnect = !MTdisconnect;
		if (!MTdisconnect) {
			connectXS.xmtConnect();
		}
	}
}

void iaAcquireGesture::resetIMUSensor()
{
	
		for (int i = 0; i < (int)connectXS.mtwDevices.size(); ++i)
		{
			std::cout << "\n reset:" << connectXS.mtwDevices[i]->resetOrientation(XRM_Alignment) << std::endl;
		}		
	
	iaAcquireGesture::calibIMU = true;
}

void iaAcquireGesture::saveRawQuatData(int noOfFrames)
{
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	ofstream avatarDataFile;

	char fileName[1024];
	
		sprintf_s(fileName, ".\\SkeletonData\\RawIMUData-00%d-%d%d%d.txt", 0, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
		avatarDataFile.open(fileName);
	
		avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << noOfFrames << "\n";
	
	for (int tCount = 0; tCount < noOfFrames; tCount++)
	{
		avatarDataFile 
			<< sUtility.avatarData[tCount].b0.mData[3] << "\t" << sUtility.avatarData[tCount].b0.mData[0] << "\t" << sUtility.avatarData[tCount].b0.mData[1] << "\t" << sUtility.avatarData[tCount].b0.mData[2] << "\t"
			<< sUtility.avatarData[tCount].b1.mData[3] << "\t" << sUtility.avatarData[tCount].b1.mData[0] << "\t" << sUtility.avatarData[tCount].b1.mData[1] << "\t" << sUtility.avatarData[tCount].b1.mData[2] << "\t"
			<< sUtility.avatarData[tCount].b2.mData[3] << "\t" << sUtility.avatarData[tCount].b2.mData[0] << "\t" << sUtility.avatarData[tCount].b2.mData[1] << "\t" << sUtility.avatarData[tCount].b2.mData[2] << "\t"
			<< sUtility.avatarData[tCount].b3.mData[3] << "\t" << sUtility.avatarData[tCount].b3.mData[0] << "\t" << sUtility.avatarData[tCount].b3.mData[1] << "\t" << sUtility.avatarData[tCount].b3.mData[2] << "\t"
			<< sUtility.avatarData[tCount].b4.mData[3] << "\t" << sUtility.avatarData[tCount].b4.mData[0] << "\t" << sUtility.avatarData[tCount].b4.mData[1] << "\t" << sUtility.avatarData[tCount].b4.mData[2] << "\t"
			<< sUtility.avatarData[tCount].b5.mData[3] << "\t" << sUtility.avatarData[tCount].b5.mData[0] << "\t" << sUtility.avatarData[tCount].b5.mData[1] << "\t" << sUtility.avatarData[tCount].b5.mData[2] << "\t"
			<< sUtility.avatarData[tCount].b6.mData[3] << "\t" << sUtility.avatarData[tCount].b6.mData[0] << "\t" << sUtility.avatarData[tCount].b6.mData[1] << "\t" << sUtility.avatarData[tCount].b6.mData[2] << "\t"
			<< sUtility.avatarData[tCount].b7.mData[3] << "\t" << sUtility.avatarData[tCount].b7.mData[0] << "\t" << sUtility.avatarData[tCount].b7.mData[1] << "\t" << sUtility.avatarData[tCount].b7.mData[2] << "\t"
			<< sUtility.avatarData[tCount].b8.mData[3] << "\t" << sUtility.avatarData[tCount].b8.mData[0] << "\t" << sUtility.avatarData[tCount].b8.mData[1] << "\t" << sUtility.avatarData[tCount].b8.mData[2] << "\t"
			<< sUtility.avatarData[tCount].b9.mData[3] << "\t" << sUtility.avatarData[tCount].b9.mData[0] << "\t" << sUtility.avatarData[tCount].b9.mData[1] << "\t" << sUtility.avatarData[tCount].b9.mData[2] << "\n";
	}

	avatarDataFile.close();
}

void iaAcquireGesture::readFileQuatData(char *fileName)
{
	sUtility.readAvatarData(fileName);

	for (int i = 0; i < sUtility.noOfFrames; i++)
	{
		if (i == 0)
		{
			offlineData[i][0] = sUtility.avatarData[i].b0;
			offlineData[i][1] = sUtility.avatarData[i].b1;
			offlineData[i][2] = sUtility.avatarData[i].b2;
			offlineData[i][3] = sUtility.avatarData[i].b3;
			offlineData[i][4] = sUtility.avatarData[i].b4;
			offlineData[i][5] = sUtility.avatarData[i].b5;
			offlineData[i][6] = sUtility.avatarData[i].b6;
			offlineData[i][7] = sUtility.avatarData[i].b7;
			offlineData[i][8] = sUtility.avatarData[i].b8;
			offlineData[i][9] = sUtility.avatarData[i].b9;
		}
		else
		{
			offlineData[i][0] = sUtility.avatarData[i].b0.mutiplication(offlineData[0][0].Inverse());
			offlineData[i][1] = sUtility.avatarData[i].b1.mutiplication(offlineData[0][1].Inverse());
			offlineData[i][2] = sUtility.avatarData[i].b2.mutiplication(offlineData[0][2].Inverse());
			offlineData[i][3] = sUtility.avatarData[i].b3.mutiplication(offlineData[0][3].Inverse());
			offlineData[i][4] = sUtility.avatarData[i].b4.mutiplication(offlineData[0][4].Inverse());
			offlineData[i][5] = sUtility.avatarData[i].b5.mutiplication(offlineData[0][5].Inverse());
			offlineData[i][6] = sUtility.avatarData[i].b6.mutiplication(offlineData[0][6].Inverse());
			offlineData[i][7] = sUtility.avatarData[i].b7.mutiplication(offlineData[0][7].Inverse());
			offlineData[i][8] = sUtility.avatarData[i].b8.mutiplication(offlineData[0][8].Inverse());
			offlineData[i][9] = sUtility.avatarData[i].b9.mutiplication(offlineData[0][9].Inverse());
		}
	}
}

Avatar iaAcquireGesture::getFRQuatdata(int fNum)
{
	Avatar myAvatar;

	myAvatar.b0 = offlineData[fNum][0];
	myAvatar.b1 = offlineData[fNum][1];
	myAvatar.b2 = offlineData[fNum][2];
	myAvatar.b3 = offlineData[fNum][3];
	myAvatar.b4 = offlineData[fNum][4];
	myAvatar.b5 = offlineData[fNum][5];
	myAvatar.b6 = offlineData[fNum][6];
	myAvatar.b7 = offlineData[fNum][7];
	myAvatar.b8 = offlineData[fNum][8];
	myAvatar.b9 = offlineData[fNum][9];

	myAvatar.b0.normalize();
	myAvatar.b1.normalize();
	myAvatar.b2.normalize();
	myAvatar.b3.normalize();
	myAvatar.b4.normalize();
	myAvatar.b5.normalize();
	myAvatar.b6.normalize();
	myAvatar.b7.normalize();
	myAvatar.b8.normalize();
	myAvatar.b9.normalize();

	return myAvatar;
}

void iaAcquireGesture::saveRawQDataInRealTime(int frameIndex, bool CloseFile)
{
	
	if (frameIndex == 0)
	{
		time_t curr_time;
		curr_time = time(NULL);
		tm *tm_local = localtime(&curr_time);

		char fileName[1024];

		sprintf_s(fileName, ".\\SkeletonData\\RawnewIMUData-00%d-%d%d%d.txt", 0, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
		rawIMUDataFile.open(fileName);
		rawIMUDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << 0 << "\n";
	}


	if (frameIndex > 0 && !CloseFile)
	{
		rawIMUDataFile
			<< QuatData_Pelvis.mData[3] << "\t"	<< QuatData_Pelvis.mData[0] << "\t" << QuatData_Pelvis.mData[1] << "\t" << QuatData_Pelvis.mData[2] << "\t"
			<< QuatData_head.mData[3] << "\t"	<< QuatData_head.mData[0] << "\t" << QuatData_head.mData[1] << "\t" << QuatData_head.mData[2] << "\t"
			<< QuatData_RightUpperArm.mData[3]	<< "\t" << QuatData_RightUpperArm.mData[0] << "\t"	<< QuatData_RightUpperArm.mData[1] << "\t" <<	QuatData_RightUpperArm.mData[2] << "\t"
			<< QuatData_RightLowerArm.mData[3]	<< "\t" << QuatData_RightLowerArm.mData[0] << "\t"	<< QuatData_RightLowerArm.mData[1] << "\t" <<	QuatData_RightLowerArm.mData[2] << "\t"
			<< QuatData_LeftUpperArm.mData[3]	<< "\t" << QuatData_LeftUpperArm.mData[0] << "\t"	<< QuatData_LeftUpperArm.mData[1] << "\t" <<	QuatData_LeftUpperArm.mData[2] << "\t"
			<< QuatData_LeftLowerArm.mData[3]	<< "\t" << QuatData_LeftLowerArm.mData[0] << "\t"	<< QuatData_LeftLowerArm.mData[1] << "\t" <<	QuatData_LeftLowerArm.mData[2] << "\t"
			<< QuatData_RightUpperLeg.mData[3]	<< "\t" << QuatData_RightUpperLeg.mData[0] << "\t"	<< QuatData_RightUpperLeg.mData[1] << "\t" <<	QuatData_RightUpperLeg.mData[2] << "\t"
			<< QuatData_RightLowerLeg.mData[3]	<< "\t" << QuatData_RightLowerLeg.mData[0] << "\t"	<< QuatData_RightLowerLeg.mData[1] << "\t" <<	QuatData_RightLowerLeg.mData[2] << "\t"
			<< QuatData_LeftUpperLeg.mData[3]	<< "\t" << QuatData_LeftUpperLeg.mData[0] << "\t"	<< QuatData_LeftUpperLeg.mData[1] << "\t" <<	QuatData_LeftUpperLeg.mData[2] << "\t"
			<< QuatData_LeftLowerLeg.mData[3]	<< "\t" << QuatData_LeftLowerLeg.mData[0] << "\t"	<< QuatData_LeftLowerLeg.mData[1] << "\t" <<	QuatData_LeftLowerLeg.mData[2] << "\n";
	}
	if(CloseFile && rawIMUDataFile.is_open())
	rawIMUDataFile.close();
}


