
/*
*This work is dual-licensed under BSD-3 and Apache License 2.0. 

*You can choose between one of them if you use this work.

*SPDX-License-Identifier: BSD-3-Clause OR Apache License 2.0

*/

#include "iaMotionAuthoring.h"

// adding the mid frames 

void addMidFrame(int selectedFrame, SphereUtility *&su)
{
	Avatar newFrame;
	
	newFrame.b0 = newFrame.b0.SLERP(su->avatarData[selectedFrame - 1].b0, su->avatarData[selectedFrame].b0, 0.5); if (isnan(newFrame.b0.mData[3])) newFrame.b0 = su->avatarData[selectedFrame - 1].b0;
	newFrame.b1 = newFrame.b1.SLERP(su->avatarData[selectedFrame - 1].b1, su->avatarData[selectedFrame].b1, 0.5); if (isnan(newFrame.b1.mData[3])) newFrame.b1 = su->avatarData[selectedFrame - 1].b1;
	newFrame.b2 = newFrame.b2.SLERP(su->avatarData[selectedFrame - 1].b2, su->avatarData[selectedFrame].b2, 0.5); if (isnan(newFrame.b2.mData[3])) newFrame.b2 = su->avatarData[selectedFrame - 1].b2;
	newFrame.b3 = newFrame.b3.SLERP(su->avatarData[selectedFrame - 1].b3, su->avatarData[selectedFrame].b3, 0.5); if (isnan(newFrame.b3.mData[3])) newFrame.b3 = su->avatarData[selectedFrame - 1].b3;
	newFrame.b4 = newFrame.b4.SLERP(su->avatarData[selectedFrame - 1].b4, su->avatarData[selectedFrame].b4, 0.5); if (isnan(newFrame.b4.mData[3])) newFrame.b4 = su->avatarData[selectedFrame - 1].b4;
	newFrame.b5 = newFrame.b5.SLERP(su->avatarData[selectedFrame - 1].b5, su->avatarData[selectedFrame].b5, 0.5); if (isnan(newFrame.b5.mData[3])) newFrame.b5 = su->avatarData[selectedFrame - 1].b5;
	newFrame.b6 = newFrame.b6.SLERP(su->avatarData[selectedFrame - 1].b6, su->avatarData[selectedFrame].b6, 0.5); if (isnan(newFrame.b6.mData[3])) newFrame.b6 = su->avatarData[selectedFrame - 1].b6;
	newFrame.b7 = newFrame.b7.SLERP(su->avatarData[selectedFrame - 1].b7, su->avatarData[selectedFrame].b7, 0.5); if (isnan(newFrame.b7.mData[3])) newFrame.b7 = su->avatarData[selectedFrame - 1].b7;
	newFrame.b8 = newFrame.b8.SLERP(su->avatarData[selectedFrame - 1].b8, su->avatarData[selectedFrame].b8, 0.5); if (isnan(newFrame.b8.mData[3])) newFrame.b8 = su->avatarData[selectedFrame - 1].b8;
	newFrame.b9 = newFrame.b9.SLERP(su->avatarData[selectedFrame - 1].b9, su->avatarData[selectedFrame].b9, 0.5); if (isnan(newFrame.b9.mData[3])) newFrame.b9 = su->avatarData[selectedFrame - 1].b9;
	su->noOfFrames = su->noOfFrames + 1;

	for (int i = su->noOfFrames; i >= selectedFrame; i--)
	{
		su->avatarData[i] = su->avatarData[i - 1];
	}
	su->avatarData[selectedFrame] = newFrame;
}
// deleting the frames
void deleteFrame(int selectedFrame, SphereUtility *&su)
{
	for (int i = selectedFrame-1; i < su->noOfFrames; i++)
	{
		su->avatarData[i] = su->avatarData[i + 1];
	}
	su->noOfFrames = su->noOfFrames - 1;
}

//creating the new motion
void newMotion(SphereUtility *& su)
{
	Avatar newFrame;

	newFrame.b0 = quaternion(0,0,0,0.999999);
	newFrame.b1 = quaternion(0,0,0,0.999999);
	newFrame.b2 = quaternion(0,0,0,0.999999);
	newFrame.b3 = quaternion(0,0,0,0.999999);
	newFrame.b4 = quaternion(0,0,0,0.999999);
	newFrame.b5 = quaternion(0,0,0,0.999999);
	newFrame.b6 = quaternion(0,0,0,0.999999);
	newFrame.b7 = quaternion(0,0,0,0.999999);
	newFrame.b8 = quaternion(0,0,0,0.999999);
	newFrame.b9 = quaternion(0,0,0,0.999999);
	
	
	su->avatarData[su->noOfFrames] = newFrame;
	su->noOfFrames = su->noOfFrames + 1;
}


// generating the intermediate frames
void generateIntermediateFrames(int totalIntermediateFrames, SphereUtility *&su)
{
	int noOfKeyFrames = su->noOfFrames-1;
	int intermediateFrameCount = totalIntermediateFrames / (noOfKeyFrames);
	Avatar GeneratedData[500];
	float tStep = 1 / (float)intermediateFrameCount;
	int dataCount = 0;
	for(int i = 0 ; i < noOfKeyFrames ; i++)
	{ 
		Avatar from = su->avatarData[i];
		Avatar to = su->avatarData[i + 1];

		for (float j = 0; j <= 1; j = j + tStep)
		{
			GeneratedData[dataCount].b0 = GeneratedData[dataCount].b0.SLERP(from.b0, to.b0, j); if (isnan(GeneratedData[dataCount].b0.mData[3])) GeneratedData[dataCount].b0 = from.b0;
			GeneratedData[dataCount].b1 = GeneratedData[dataCount].b1.SLERP(from.b1, to.b1, j); if (isnan(GeneratedData[dataCount].b1.mData[3])) GeneratedData[dataCount].b1 = from.b1;
			GeneratedData[dataCount].b2 = GeneratedData[dataCount].b2.SLERP(from.b2, to.b2, j); if (isnan(GeneratedData[dataCount].b2.mData[3])) GeneratedData[dataCount].b2 = from.b2;
			GeneratedData[dataCount].b3 = GeneratedData[dataCount].b3.SLERP(from.b3, to.b3, j); if (isnan(GeneratedData[dataCount].b3.mData[3])) GeneratedData[dataCount].b3 = from.b3;
			GeneratedData[dataCount].b4 = GeneratedData[dataCount].b4.SLERP(from.b4, to.b4, j); if (isnan(GeneratedData[dataCount].b4.mData[3])) GeneratedData[dataCount].b4 = from.b4;
			GeneratedData[dataCount].b5 = GeneratedData[dataCount].b5.SLERP(from.b5, to.b5, j); if (isnan(GeneratedData[dataCount].b5.mData[3])) GeneratedData[dataCount].b5 = from.b5;
			GeneratedData[dataCount].b6 = GeneratedData[dataCount].b6.SLERP(from.b6, to.b6, j); if (isnan(GeneratedData[dataCount].b6.mData[3])) GeneratedData[dataCount].b6 = from.b6;
			GeneratedData[dataCount].b7 = GeneratedData[dataCount].b7.SLERP(from.b7, to.b7, j); if (isnan(GeneratedData[dataCount].b7.mData[3])) GeneratedData[dataCount].b7 = from.b7;
			GeneratedData[dataCount].b8 = GeneratedData[dataCount].b8.SLERP(from.b8, to.b8, j); if (isnan(GeneratedData[dataCount].b8.mData[3])) GeneratedData[dataCount].b8 = from.b8;
			GeneratedData[dataCount].b9 = GeneratedData[dataCount].b9.SLERP(from.b9, to.b9, j); if (isnan(GeneratedData[dataCount].b9.mData[3])) GeneratedData[dataCount].b9 = from.b9;
			dataCount++;
		}
	}
	su->noOfFrames = dataCount;
	for (int i = 0; i < su->noOfFrames; i++)
	{
		su->avatarData[i] = GeneratedData[i];
	}
}

//finding the duplicate frames
void duplicateCurrentFrame(int stencilIndex, SphereUtility *& su, int copyFlag)
{
	switch (copyFlag)
	{
		case COPY_AT:	for (int i = su->noOfFrames; i > stencilIndex; i--)
							{
								su->avatarData[i] = su->avatarData[i - 1];
							}

							su->noOfFrames = su->noOfFrames + 1;
						break;
		
		case COPY_END: 	su->avatarData[su->noOfFrames] = su->avatarData[stencilIndex];
						su->noOfFrames = su->noOfFrames + 1;
						break;
	}
}
