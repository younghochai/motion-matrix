#pragma once
#include "iaSphereUtility.h"
#define COPY_END 2
#define COPY_AT 1

void addMidFrame(int selectedFrame, SphereUtility *&su);
void deleteFrame(int selectedFrame, SphereUtility *&su);
void newMotion(SphereUtility *&su);
void generateIntermediateFrames(int noOfIntermediateFrames, SphereUtility *&su);
void duplicateCurrentFrame(int stencilIndex, SphereUtility *&su, int copyFlag);