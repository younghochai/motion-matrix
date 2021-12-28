#pragma once

/*
*This work is dual-licensed under BSD-3 and Apache License 2.0. 

*You can choose between one of them if you use this work.

*SPDX-License-Identifier: BSD-3-Clause OR Apache License 2.0

*/

#include <vtkActor.h>
#include <vtkActor.h>
#include <vtkCylinderSource.h>
#include <vtkMath.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include <array>
#include <iaSphereUtility.h>
#include <iaQuaternion.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkJPEGReader.h>
#include <vtkTexture.h>
#include <vtkOBJReader.h>
#include <vtkPlaneSource.h>
#include <vtkPropPicker.h>
#include <iaMotionSphere.h>
#include <vtkCallbackCommand.h>
#define USER_MATRIX

class VitruvianAvatar
{
public:
	static Avatar vitruvianAvatarUpdate;
	static bool isLoaded;
	static double humanHeight;
public :
	
	static void  initializeVetruvianVtkAvatar();
	void  startVetruvianAvatar();
	vtkSmartPointer<vtkRenderer>   getVitruvianAvatarRenderer();

};



