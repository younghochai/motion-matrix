
/*
*This work is dual-licensed under BSD-3 and Apache License 2.0. 

*You can choose between one of them if you use this work.

*SPDX-License-Identifier: BSD-3-Clause OR Apache License 2.0

*/

#pragma once
#include "iaSphereUtility.h"
#define COPY_END 2
#define COPY_AT 1

void addMidFrame(int selectedFrame, SphereUtility *&su);
void deleteFrame(int selectedFrame, SphereUtility *&su);
void newMotion(SphereUtility *&su);
void generateIntermediateFrames(int noOfIntermediateFrames, SphereUtility *&su);
void duplicateCurrentFrame(int stencilIndex, SphereUtility *&su, int copyFlag);