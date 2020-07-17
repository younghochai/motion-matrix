#pragma once
#include "iaSphereUtility.h"

void addFrame(int selectedFrame, SphereUtility *&su);
void deleteFrame(int selectedFrame, SphereUtility *&su);
void newMotion(SphereUtility *&su);
void generateIntermediateFrames(int noOfIntermediateFrames, SphereUtility *&su);