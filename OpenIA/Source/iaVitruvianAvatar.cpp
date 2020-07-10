#include "iaVitruvianAvatar.h"
#include <vtkParametricSuperEllipsoid.h>
#include <vtkParametricFunctionSource.h>

Avatar VitruvianAvatar::vitruvianAvatarUpdate;
bool VitruvianAvatar::isLoaded = false;
typedef struct vetruvianVtkAvatarSegment
{
	vtkSmartPointer<vtkParametricSuperEllipsoid> parametricObject = vtkSmartPointer<vtkParametricSuperEllipsoid>::New();
	vtkSmartPointer<vtkParametricFunctionSource> cylinderSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
	vtkSmartPointer<vtkPolyDataMapper> cylinderMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> cylinderActor = vtkSmartPointer<vtkActor>::New();

	vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
	double jointPoint[3];
	float length;
} JointSegment;

typedef struct vetruvianTorso
{
	vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
	vtkSmartPointer<vtkPolyDataMapper> coneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> coneActor = vtkSmartPointer<vtkActor>::New();

	vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
	double jointPoint[3];
	float length;
} Torso;

struct Hand
{
	vtkSmartPointer<vtkOBJReader> handSource = vtkSmartPointer<vtkOBJReader>::New();
	vtkSmartPointer<vtkPolyDataMapper> handMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> handActor = vtkSmartPointer<vtkActor>::New();
	quaternion rotation;
};

struct Hand leftHand, rightHand;
vtkSmartPointer<vtkCubeSource> leftHandSource = vtkSmartPointer<vtkCubeSource>::New();
vtkSmartPointer<vtkPolyDataMapper> leftHandMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
vtkSmartPointer<vtkActor> leftHandActor = vtkSmartPointer<vtkActor>::New();

JointSegment rightFootLowerLeg, rightKneeUpperLeg, rightHipSegment, leftFootLowerLeg, leftKneeUpperLeg, leftHipSegment, pelvisStomach;
JointSegment sternumChest, neckToChin, chinToHead, head, rightShoulderSegment, rightElbowUpperArm, rightHandLowerArm, leftShoulderSegment, leftElbowUpperArm, leftHandLowerArm;

vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

SphereUtility vitruvianSphereUtility;

bool isEqual = false;

void drawSphere(JointSegment &jointSegment, float radius)
{
	// Create spheres for start and end point

	jointSegment.sphereSource->SetCenter(jointSegment.jointPoint);
	jointSegment.sphereSource->SetRadius(radius);
	jointSegment.sphereSource->SetPhiResolution(50);
	jointSegment.sphereSource->SetThetaResolution(50);
	jointSegment.sphereMapper->SetInputConnection(jointSegment.sphereSource->GetOutputPort());

	jointSegment.sphereActor->SetMapper(jointSegment.sphereMapper);
	jointSegment.sphereActor->GetProperty()->SetColor(colors->GetColor3d("DarkOrange").GetData());
	jointSegment.sphereActor->GetProperty()->SetSpecular(.5);
	jointSegment.sphereActor->GetProperty()->SetSpecularPower(20);

	renderer->AddActor(jointSegment.sphereActor);
}

void drawCube(JointSegment& handJoint, Hand& hand, float size, char* fileName)
{
	

	/*hand.handSource->SetCenter(handJoint.jointPoint);
	hand.handSource->SetXLength(size - 2);
	hand.handSource->SetYLength(size + 1);
	hand.handSource->SetZLength(size);*/
	// Create a mapper and actor.

	hand.handMapper->SetInputConnection(hand.handSource->GetOutputPort());

	/*float axis[3], angle;
	angle = 2 * acos(hand.rotation.mData[3]);
	if (angle < 0.001)
	{
		axis[0] = hand.rotation.mData[0];
		axis[1] = hand.rotation.mData[2];
		axis[2] = -hand.rotation.mData[1];
	}
	else
	{
		axis[0] = hand.rotation.mData[0] / sqrt(1 - hand.rotation.mData[3] * hand.rotation.mData[3]);
		axis[1] = hand.rotation.mData[2] / sqrt(1 - hand.rotation.mData[3] * hand.rotation.mData[3]);
		axis[2] = -hand.rotation.mData[1] / sqrt(1 - hand.rotation.mData[3] * hand.rotation.mData[3]);
	}*/
	hand.rotation.quattoaxisangle();

	vtkSmartPointer<vtkTransform> transform =
		vtkSmartPointer<vtkTransform>::New();
	//transform->PostMultiply(); //this is the key line
	transform->Translate(handJoint.jointPoint);
	transform->Scale(size,size,size);
	/*transform->RotateX(90);
	transform->RotateY(-90);*/
	transform->RotateWXYZ(hand.rotation.axisangle[0], 
		hand.rotation.axisangle[1],
		hand.rotation.axisangle[2],
		hand.rotation.axisangle[3]);
	//hand.handActor->RotateWXYZ(angle, axis[0], axis[1], axis[2]);
	hand.handActor->SetMapper(hand.handMapper);
	hand.handActor->SetUserTransform(transform);
	hand.handActor->GetProperty()->SetSpecular(.5);
	hand.handActor->GetProperty()->SetSpecularPower(10);
	renderer->AddActor(hand.handActor);
}

vtkSmartPointer<vtkOBJReader> headSource = vtkSmartPointer<vtkOBJReader>::New();
vtkSmartPointer<vtkPolyDataMapper> headMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
vtkSmartPointer<vtkActor> headActor = vtkSmartPointer<vtkActor>::New();
quaternion headQuat;

void drawHead(JointSegment &neck)
{
	

	/*hand.handSource->SetCenter(handJoint.jointPoint);
	hand.handSource->SetXLength(size - 2);
	hand.handSource->SetYLength(size + 1);
	hand.handSource->SetZLength(size);*/
	// Create a mapper and actor.

	headMapper->SetInputConnection(headSource->GetOutputPort());

	headQuat.quattoaxisangle();

	/*float axis[3], angle;
	angle = 2 * acos(headQuat.mData[3]);
	if (angle < 0.001)
	{
		axis[0] = headQuat.mData[0];
		axis[1] = headQuat.mData[1];
		axis[2] = headQuat.mData[2];
	}
	else
	{
		axis[0] = headQuat.mData[0] / sin(angle / 2);
		axis[1] = headQuat.mData[1] / sin(angle / 2);
		axis[2] = headQuat.mData[2] / sin(angle / 2);
	}*/
	//create a texture map for torso
	vtkSmartPointer<vtkTexture> texture =
		vtkSmartPointer<vtkTexture>::New();
	vtkSmartPointer<vtkJPEGReader> reader =
		vtkSmartPointer<vtkJPEGReader>::New();
	reader->SetFileName("headmap.jpg");
	// Apply the texture
	texture->SetInputConnection(reader->GetOutputPort());

	vtkSmartPointer<vtkTransform> transform =
		vtkSmartPointer<vtkTransform>::New();
	//transform->PostMultiply(); //this is the key line
	transform->Translate(neck.jointPoint);
	transform->Scale(12.5, 12.5, 12.5);
	/*transform->RotateX(90);
	transform->RotateY(-90);*/
	transform->RotateWXYZ(headQuat.axisangle[0],headQuat.axisangle[1], headQuat.axisangle[2], headQuat.axisangle[3]);
	//headActor->RotateWXYZ(angle * 180 / PI, axis[0], axis[1], axis[2]);
	headActor->SetMapper(headMapper);
	headActor->SetUserTransform(transform);
	headActor->SetTexture(texture);
	headActor->GetProperty()->SetSpecular(.5);
	headActor->GetProperty()->SetSpecularPower(10);
	renderer->AddActor(headActor);
}

void drawBoneSegment(JointSegment &startSegment, JointSegment &endSegment, float* scaling, bool isHead, char* color)
{
	//set parametric ellipsoid to source
	startSegment.cylinderSource->SetParametricFunction(startSegment.parametricObject);
	startSegment.cylinderSource->Update();
	startSegment.cylinderSource->SetGenerateTextureCoordinates(true);
	// Set the background color.
	std::array<unsigned char, 4> bkg{ {255, 255, 255, 0} };
	colors->SetColor("BkgColor", bkg.data());

	// Create a cylinder.
	// Cylinder height vector is (0,1,0).
	// Cylinder center is in the middle of the cylinder

	//startSegment.cylinderSource->SetResolution(50);
	//startSegment.cylinderSource->SetRadius(2);
	startSegment.parametricObject->SetN1(1.0);
	startSegment.parametricObject->SetXRadius(scaling[0]);
	startSegment.parametricObject->SetYRadius(scaling[1]);
	startSegment.parametricObject->SetZRadius(scaling[2]);

	//create a texture map for torso
	vtkSmartPointer<vtkTexture> texture =
		vtkSmartPointer<vtkTexture>::New();
	vtkSmartPointer<vtkJPEGReader> reader =
		vtkSmartPointer<vtkJPEGReader>::New();

	vtkSmartPointer<vtkTransform> transform =
		vtkSmartPointer<vtkTransform>::New();
	if (scaling[0] == 10)
	{
		//startSegment.cylinderActor->RotateX(90);
		startSegment.parametricObject->SetXRadius(scaling[0]);
		startSegment.parametricObject->SetYRadius(scaling[1]);
		startSegment.parametricObject->SetZRadius(scaling[2]);
		
		// Read the image which will be the texture
		reader->SetFileName("velab.jpg");
		// Apply the texture
		texture->SetInputConnection(reader->GetOutputPort());
		VitruvianAvatar::vitruvianAvatarUpdate.b1.quattoaxisangle();
		/*transform->RotateWXYZ(VitruvianAvatar::vitruvianAvatarUpdate.b1.axisangle[0],
			VitruvianAvatar::vitruvianAvatarUpdate.b1.axisangle[1],
			VitruvianAvatar::vitruvianAvatarUpdate.b1.axisangle[2],
			VitruvianAvatar::vitruvianAvatarUpdate.b1.axisangle[3]);*/
	}

	// Compute a basis
	double normalizedX[3];
	double normalizedY[3];
	double normalizedZ[3];

	// The X axis is a vector from start to end
	vtkMath::Subtract(endSegment.jointPoint, startSegment.jointPoint, normalizedX);
	double length = vtkMath::Norm(normalizedX);
	vtkMath::Normalize(normalizedX);

	vtkSmartPointer<vtkMinimalStandardRandomSequence> rng =
		vtkSmartPointer<vtkMinimalStandardRandomSequence>::New();
	// The Z axis is an arbitrary vector cross X
	double arbitrary[3];
	for (auto i = 0; i < 3; ++i)
	{
		rng->Next();
		arbitrary[i] = rng->GetRangeValue(-10, 10);
	}
	vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
	vtkMath::Normalize(normalizedZ);

	// The Y axis is Z cross X
	vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
	vtkSmartPointer<vtkMatrix4x4> matrix =
		vtkSmartPointer<vtkMatrix4x4>::New();

	// Create the direction cosine matrix
	matrix->Identity();
	for (unsigned int i = 0; i < 3; i++)
	{
		matrix->SetElement(i, 0, normalizedX[i]);
		matrix->SetElement(i, 1, normalizedY[i]);
		matrix->SetElement(i, 2, normalizedZ[i]);
	}

	
	// Apply the transforms
	
	transform->Translate(startSegment.jointPoint);   // translate to starting point
	transform->Concatenate(matrix);     // apply direction cosines
	transform->RotateZ(-90.0);          // align cylinder to x axis
	transform->RotateY(-27.0);

	transform->Scale(1.0, length, 1.0); // scale along the height vector
	transform->Translate(0, .5, 0);     // translate to start of cylinder

	// Transform the polydata
	vtkSmartPointer<vtkTransformPolyDataFilter> transformPD =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformPD->SetTransform(transform);
	transformPD->SetInputConnection(startSegment.cylinderSource->GetOutputPort());

#ifdef USER_MATRIX
	startSegment.cylinderMapper->SetInputConnection(startSegment.cylinderSource->GetOutputPort());
	startSegment.cylinderActor->SetUserMatrix(transform->GetMatrix());
#else
	mapper->SetInputConnection(transformPD->GetOutputPort());
#endif
	startSegment.cylinderActor->SetMapper(startSegment.cylinderMapper);
	if (scaling[0] == 30)
	{
		//attach texture to the actor
		//startSegment.cylinderActor->SetTexture(texture);
		//startSegment.cylinderActor->RotateWXYZ(VitruvianAvatar::vitruvianAvatarUpdate.b1.axisangle[0],
		//	VitruvianAvatar::vitruvianAvatarUpdate.b1.axisangle[1],
		//	VitruvianAvatar::vitruvianAvatarUpdate.b1.axisangle[2],
		//	VitruvianAvatar::vitruvianAvatarUpdate.b1.axisangle[3]);
		//
	}
	else
		startSegment.cylinderActor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
	startSegment.cylinderActor->GetProperty()->SetSpecular(.5);
	startSegment.cylinderActor->GetProperty()->SetSpecularPower(20);
	//draw start and end points as a sphere
	if (!isHead)
	{
		drawSphere(startSegment, 3);
		drawSphere(endSegment, 3);
	}
	

	//Add the actor to the scene

	renderer->AddActor(startSegment.cylinderActor);
}


double max(double var1, double var2)
{
	return var1 >= var2 ? var1 : var2;
}

/*avatar variables*/
double VitruvianAvatar::humanHeight = 172.0;
double headUnit;

void updatejoints()
{
	rightKneeUpperLeg.jointPoint[0] = rightFootLowerLeg.jointPoint[0];
	rightKneeUpperLeg.jointPoint[1] = rightFootLowerLeg.jointPoint[1] + 1.5 * headUnit;
	rightKneeUpperLeg.jointPoint[2] = rightFootLowerLeg.jointPoint[2];

	rightHipSegment.jointPoint[0] = rightKneeUpperLeg.jointPoint[0];
	rightHipSegment.jointPoint[1] = rightKneeUpperLeg.jointPoint[1] + 1.5*headUnit;
	rightHipSegment.jointPoint[2] = rightKneeUpperLeg.jointPoint[2];

	leftKneeUpperLeg.jointPoint[0] = leftFootLowerLeg.jointPoint[0];
	leftKneeUpperLeg.jointPoint[1] = leftFootLowerLeg.jointPoint[1] + 1.5 * headUnit;
	leftKneeUpperLeg.jointPoint[2] = leftFootLowerLeg.jointPoint[2];

	leftHipSegment.jointPoint[0] = leftKneeUpperLeg.jointPoint[0];
	leftHipSegment.jointPoint[1] = leftKneeUpperLeg.jointPoint[1] + 1.5*headUnit;
	leftHipSegment.jointPoint[2] = leftKneeUpperLeg.jointPoint[2];

	pelvisStomach.jointPoint[0] = (leftHipSegment.jointPoint[0] + rightHipSegment.jointPoint[0]) / 2;
	pelvisStomach.jointPoint[1] = max(leftHipSegment.jointPoint[1], rightHipSegment.jointPoint[1]);
	pelvisStomach.jointPoint[2] = max(leftHipSegment.jointPoint[2], rightHipSegment.jointPoint[2]);

	sternumChest.jointPoint[0] = pelvisStomach.jointPoint[0];
	sternumChest.jointPoint[1] = pelvisStomach.jointPoint[1] + headUnit;
	sternumChest.jointPoint[2] = pelvisStomach.jointPoint[2];

	neckToChin.jointPoint[0] = sternumChest.jointPoint[0];
	neckToChin.jointPoint[1] = sternumChest.jointPoint[1] + headUnit;
	neckToChin.jointPoint[2] = sternumChest.jointPoint[2];

	chinToHead.jointPoint[0] = neckToChin.jointPoint[0];
	chinToHead.jointPoint[1] = neckToChin.jointPoint[1] + (headUnit*0.4);
	chinToHead.jointPoint[2] = neckToChin.jointPoint[2];

	head.jointPoint[0] = chinToHead.jointPoint[0];
	head.jointPoint[1] = chinToHead.jointPoint[1] + (headUnit*0.6);
	head.jointPoint[2] = chinToHead.jointPoint[2];

	rightShoulderSegment.jointPoint[0] = neckToChin.jointPoint[0] - 0.8*headUnit;
	rightShoulderSegment.jointPoint[1] = neckToChin.jointPoint[1];
	rightShoulderSegment.jointPoint[2] = neckToChin.jointPoint[2];

	rightElbowUpperArm.jointPoint[0] = rightShoulderSegment.jointPoint[0];
	rightElbowUpperArm.jointPoint[1] = rightShoulderSegment.jointPoint[1] - (1.2*headUnit);
	rightElbowUpperArm.jointPoint[2] = rightShoulderSegment.jointPoint[2];

	rightHandLowerArm.jointPoint[0] = rightElbowUpperArm.jointPoint[0];
	rightHandLowerArm.jointPoint[1] = rightElbowUpperArm.jointPoint[1] - (1.2*headUnit);
	rightHandLowerArm.jointPoint[2] = rightElbowUpperArm.jointPoint[2];

	leftShoulderSegment.jointPoint[0] = neckToChin.jointPoint[0] + 0.8*headUnit;
	leftShoulderSegment.jointPoint[1] = neckToChin.jointPoint[1];
	leftShoulderSegment.jointPoint[2] = neckToChin.jointPoint[2];

	leftElbowUpperArm.jointPoint[0] = leftShoulderSegment.jointPoint[0];
	leftElbowUpperArm.jointPoint[1] = leftShoulderSegment.jointPoint[1] - (1.2*headUnit);
	leftElbowUpperArm.jointPoint[2] = leftShoulderSegment.jointPoint[2];

	leftHandLowerArm.jointPoint[0] = leftElbowUpperArm.jointPoint[0];
	leftHandLowerArm.jointPoint[1] = leftElbowUpperArm.jointPoint[1] - (1.2*headUnit);
	leftHandLowerArm.jointPoint[2] = leftElbowUpperArm.jointPoint[2];
}

void drawFloor()
{
	// Create a plane
	vtkSmartPointer<vtkPlaneSource> planeSource =
		vtkSmartPointer<vtkPlaneSource>::New();
	planeSource->SetCenter(0.0, -0.0015, 0.0);
	planeSource->SetNormal(0.0, 1.0, 0.0);
	//planeSource->set
	planeSource->Update();

	vtkPolyData* plane = planeSource->GetOutput();

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(plane);

	vtkSmartPointer<vtkTexture> planeTexture = vtkSmartPointer<vtkTexture>::New();
	vtkSmartPointer<vtkJPEGReader> reader = vtkSmartPointer<vtkJPEGReader>::New();
	// Read the image which will be the texture
	reader->SetFileName("floor.jpg");
	// Apply the texture
	planeTexture->SetInputConnection(reader->GetOutputPort());
	planeTexture->SetRepeat(true);
	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->SetTexture(planeTexture);
	actor->GetProperty()->LightingOff();
	//actor->GetProperty()->SetColor(colors->GetColor3d("Gray").GetData());
	actor->SetScale(1000.0);
	renderer->AddActor(actor);
}

void drawAvatar()
{
	float scaling[3];

	scaling[0] = 2.5; scaling[1] = 0.5; scaling[2] = 2.5;
	drawBoneSegment(rightFootLowerLeg, rightKneeUpperLeg,scaling, false,"DimGray"); // right lower leg
	scaling[0] = 3.5; scaling[1] = 0.5; scaling[2] = 3.5;
	drawBoneSegment(rightKneeUpperLeg, rightHipSegment,scaling, false, "DimGray"); // right upper leg

	scaling[0] = 2.5; scaling[1] = 0.5; scaling[2] = 2.5;
	drawBoneSegment(leftFootLowerLeg, leftKneeUpperLeg,scaling, false, "DimGray"); // left lower leg
	scaling[0] = 3.5; scaling[1] = 0.5; scaling[2] = 3.5;
	drawBoneSegment(leftKneeUpperLeg, leftHipSegment,scaling, false, "DimGray"); // left upper leg

	scaling[0] = 4.5; scaling[1] = 0.5; scaling[2] = 4.5;
	drawBoneSegment(rightHipSegment, pelvisStomach, scaling, false, "DarkGray"); // right hips
	drawBoneSegment(leftHipSegment, pelvisStomach, scaling, false, "DarkGray"); // left hips

	scaling[0] = 5.0; scaling[1] = 0.5; scaling[2] = 5.0;
	drawBoneSegment(pelvisStomach, sternumChest,scaling, false, "DarkGray"); // stomach
	scaling[0] = 10.0; scaling[1] = 0.5; scaling[2] = 7.5;
	drawBoneSegment(sternumChest, neckToChin,scaling, false, "DarkGray"); // chest
	//scaling[0] = 1.75; scaling[1] = 0.5; scaling[2] = 1.75;
	//drawBoneSegment(neckToChin, chinToHead,scaling,true, "DarkGray"); // neck
	//scaling[0] = 5.0; scaling[1] = 0.8; scaling[2] = 3.5;
	//drawBoneSegment(chinToHead, head, scaling, true, "DarkGray"); // head
	//drawBoneSegment(head, head, scaling,true); //head
	drawHead(neckToChin);

	scaling[0] = 3.5; scaling[1] = 0.5; scaling[2] = 3.5;
	drawBoneSegment(rightShoulderSegment, neckToChin,scaling, false, "DimGray"); // right shoulder
	drawBoneSegment(rightElbowUpperArm, rightShoulderSegment,scaling, false, "DimGray"); // right upper arm
	scaling[0] = 2.5; scaling[1] = 0.5; scaling[2] = 2.5;
	drawBoneSegment(rightHandLowerArm, rightElbowUpperArm,scaling, false, "DimGray"); // right lower arm
	drawCube(rightHandLowerArm, rightHand, 15,"RightHand.obj");

	scaling[0] = 3.5; scaling[1] = 0.5; scaling[2] = 3.5;
	drawBoneSegment(leftShoulderSegment, neckToChin,scaling, false, "DimGray"); // left shoulder
	drawBoneSegment(leftElbowUpperArm, leftShoulderSegment,scaling, false, "DimGray"); // left upper arm
	scaling[0] = 2.5; scaling[1] = 0.5; scaling[2] = 2.5;
	drawBoneSegment(leftHandLowerArm, leftElbowUpperArm,scaling, false, "DimGray"); // left lower arm
	drawCube(leftHandLowerArm, leftHand, 15,"LeftHand.obj");
}

void rotateAvatar(Avatar avatar)
{
	//--------------------------------------- Translate lower body to ground point ---------------------------
	double diffR = rightFootLowerLeg.jointPoint[1];
	double diffL = leftFootLowerLeg.jointPoint[1];
	
	rightFootLowerLeg.jointPoint[0] = rightFootLowerLeg.jointPoint[0];
	rightFootLowerLeg.jointPoint[1] = 0;
	rightFootLowerLeg.jointPoint[2] = rightFootLowerLeg.jointPoint[2];

	leftFootLowerLeg.jointPoint[0] = leftFootLowerLeg.jointPoint[0];
	leftFootLowerLeg.jointPoint[1] = 0;
	leftFootLowerLeg.jointPoint[2] = leftFootLowerLeg.jointPoint[2];

	rightKneeUpperLeg.jointPoint[1] = rightKneeUpperLeg.jointPoint[1] - diffR;
	rightHipSegment.jointPoint[1] = rightHipSegment.jointPoint[1] - diffR;

	leftKneeUpperLeg.jointPoint[1]	= leftKneeUpperLeg.jointPoint[1] - diffL;
	leftHipSegment.jointPoint[1]	= leftHipSegment.jointPoint[1] - diffL;
	
	
	//---------------------------------------- Bottom-Up Update -----------------------------------------------
	//Finding right knee joint from the fixed right foot.
	quaternion q = avatar.b7;// .Inverse().mutiplication(avatar.b7);
	quaternion vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	rightKneeUpperLeg.jointPoint[0] = rightFootLowerLeg.jointPoint[0] + vec.mData[0] * rightFootLowerLeg.length;
	rightKneeUpperLeg.jointPoint[1] = 0 + vec.mData[2] * rightFootLowerLeg.length;
	rightKneeUpperLeg.jointPoint[2] = rightFootLowerLeg.jointPoint[2] + -vec.mData[1] * rightFootLowerLeg.length;

	//Finding left knee joint from the fixed left foot.
	q = avatar.b9;// .Inverse().mutiplication(avatar.b9);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	leftKneeUpperLeg.jointPoint[0] = leftFootLowerLeg.jointPoint[0] + vec.mData[0] * leftFootLowerLeg.length;
	leftKneeUpperLeg.jointPoint[1] = 0 + vec.mData[2] * leftFootLowerLeg.length;
	leftKneeUpperLeg.jointPoint[2] = leftFootLowerLeg.jointPoint[2] + -vec.mData[1] * leftFootLowerLeg.length;

	//Finding right pelvis joint from the right knee.
	q = avatar.b6;// .Inverse().mutiplication(avatar.b6);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	rightHipSegment.jointPoint[0] = rightKneeUpperLeg.jointPoint[0] + vec.mData[0] * rightKneeUpperLeg.length;
	rightHipSegment.jointPoint[1] = rightKneeUpperLeg.jointPoint[1] + vec.mData[2] * rightKneeUpperLeg.length;
	rightHipSegment.jointPoint[2] = rightKneeUpperLeg.jointPoint[2] + -vec.mData[1] * rightKneeUpperLeg.length;

	//Finding left pelvis joint from the left knee.
	q = avatar.b8;// .Inverse().mutiplication(avatar.b8);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	leftHipSegment.jointPoint[0] = leftKneeUpperLeg.jointPoint[0] + vec.mData[0] * leftKneeUpperLeg.length;
	leftHipSegment.jointPoint[1] = leftKneeUpperLeg.jointPoint[1] + vec.mData[2] * leftKneeUpperLeg.length;
	leftHipSegment.jointPoint[2] = leftKneeUpperLeg.jointPoint[2] + -vec.mData[1] * leftKneeUpperLeg.length;

	//Finding pelvis joint from the left and right pelvis.
	if (leftHipSegment.jointPoint[1] > rightHipSegment.jointPoint[1])
	{
		q = avatar.b0;// .Inverse().mutiplication(avatar.b8);
		vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

		pelvisStomach.jointPoint[0] = leftHipSegment.jointPoint[0] + vec.mData[0] * leftHipSegment.length;
		pelvisStomach.jointPoint[1] = leftHipSegment.jointPoint[1] + vec.mData[2] * leftHipSegment.length;
		pelvisStomach.jointPoint[2] = leftHipSegment.jointPoint[2] -vec.mData[1] *  leftHipSegment.length;
		
		/*float diffR = leftHipSegment.jointPoint[1] - rightHipSegment.jointPoint[1];
		rightKneeUpperLeg.jointPoint[1] = rightKneeUpperLeg.jointPoint[1] + diffR;
		rightFootLowerLeg.jointPoint[1] = rightFootLowerLeg.jointPoint[1] + diffR;
		rightHipSegment.jointPoint[1] = rightHipSegment.jointPoint[1] + diffR;

		diffR = leftHipSegment.jointPoint[2] - rightHipSegment.jointPoint[2];;
		rightKneeUpperLeg.jointPoint[2] = rightKneeUpperLeg.jointPoint[2] + diffR;
		rightFootLowerLeg.jointPoint[2] = rightFootLowerLeg.jointPoint[2] + diffR;
		rightHipSegment.jointPoint[2] = rightHipSegment.jointPoint[2] + diffR;*/

	//	isEqual = false;
		
	}else 	if (rightHipSegment.jointPoint[1] >= leftHipSegment.jointPoint[1])
	{
		q = avatar.b0;// .Inverse().mutiplication(avatar.b8);
		vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

		pelvisStomach.jointPoint[0] = rightHipSegment.jointPoint[0] + vec.mData[0] * rightHipSegment.length;
		pelvisStomach.jointPoint[1] = rightHipSegment.jointPoint[1] + vec.mData[2] * rightHipSegment.length;
		pelvisStomach.jointPoint[2] = rightHipSegment.jointPoint[2] - vec.mData[1] * rightHipSegment.length;
		
		/*float diffR = rightHipSegment.jointPoint[1] - leftHipSegment.jointPoint[1];
		leftKneeUpperLeg.jointPoint[1]	= leftKneeUpperLeg.jointPoint[1] + diffR;
		leftFootLowerLeg.jointPoint[1]	= leftFootLowerLeg.jointPoint[1] + diffR;
		leftHipSegment.jointPoint[1]	= leftHipSegment.jointPoint[1] + diffR;

		diffR = rightHipSegment.jointPoint[2] - leftHipSegment.jointPoint[2];
		leftKneeUpperLeg.jointPoint[2]	= leftKneeUpperLeg.jointPoint[2] + diffR;
		leftFootLowerLeg.jointPoint[2]	= leftFootLowerLeg.jointPoint[2] + diffR;
		leftHipSegment.jointPoint[2]	= leftHipSegment.jointPoint[2]   + diffR;*/
		// isEqual = false;
	}
	/*else
	{
		pelvisStomach.jointPoint[0] = (rightHipSegment.jointPoint[0] + leftHipSegment.jointPoint[0]) / 2;
		pelvisStomach.jointPoint[1] = (rightHipSegment.jointPoint[1] + leftHipSegment.jointPoint[1]) / 2;
		pelvisStomach.jointPoint[2] = (rightHipSegment.jointPoint[2] + leftHipSegment.jointPoint[2]) / 2;
		isEqual = true;
	}*/

//----------------------------------------Top-Down Update---------------------------------------------------------------------------------------
		//Finding right pelvis joint from the pelvis.
		q = avatar.b0;// .Inverse().mutiplication(avatar.b0);
		vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

		rightHipSegment.jointPoint[0] = pelvisStomach.jointPoint[0] + vec.mData[0] * rightHipSegment.length;
		rightHipSegment.jointPoint[1] = pelvisStomach.jointPoint[1] + vec.mData[2] * rightHipSegment.length; //Changes in Lower body with pelvis rotation and model moves continously in z axis
		rightHipSegment.jointPoint[2] = pelvisStomach.jointPoint[2] - vec.mData[1] * rightHipSegment.length;//Changes in Lower body with pelvis rotation and model moves continously in z axis

		//Finding left pelvis joint from the pelvis.
		q = avatar.b0;// .Inverse().mutiplication(avatar.b0);
		vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

		leftHipSegment.jointPoint[0] = pelvisStomach.jointPoint[0] + vec.mData[0] * leftHipSegment.length;
		leftHipSegment.jointPoint[1] = pelvisStomach.jointPoint[1] + vec.mData[2] * leftHipSegment.length;  //Changes in Lower body with pelvis rotation and model moves continously in z axis
		leftHipSegment.jointPoint[2] = pelvisStomach.jointPoint[2] - vec.mData[1] * leftHipSegment.length;	//Changes in Lower body with pelvis rotation and model moves continously in z axis

	//if (!isEqual) //Execute topdown approach only when left and right hips are unequal
	{
		//Finding right knee from the right pelvis point
		q = avatar.b6;// .Inverse().mutiplication(avatar.b6);
		vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

		rightKneeUpperLeg.jointPoint[0] = (rightHipSegment.jointPoint[0] + vec.mData[0] * rightKneeUpperLeg.length);
		rightKneeUpperLeg.jointPoint[1] = rightHipSegment.jointPoint[1] + vec.mData[2] * rightKneeUpperLeg.length;
		rightKneeUpperLeg.jointPoint[2] = rightHipSegment.jointPoint[2] + -vec.mData[1] * rightKneeUpperLeg.length;

		//Finding left knee from the left pelvis point
		q = avatar.b8;// .Inverse().mutiplication(avatar.b8);
		vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

		leftKneeUpperLeg.jointPoint[0] = leftHipSegment.jointPoint[0] + vec.mData[0] * leftKneeUpperLeg.length;
		leftKneeUpperLeg.jointPoint[1] = leftHipSegment.jointPoint[1] + vec.mData[2] * leftKneeUpperLeg.length;
		leftKneeUpperLeg.jointPoint[2] = leftHipSegment.jointPoint[2] + -vec.mData[1] * leftKneeUpperLeg.length;

		//Finding right foot from the right knee point
		q = avatar.b7;// .Inverse().mutiplication(avatar.b7);
		vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

		rightFootLowerLeg.jointPoint[0] = rightKneeUpperLeg.jointPoint[0] + vec.mData[0] * rightFootLowerLeg.length;
		rightFootLowerLeg.jointPoint[1] = rightKneeUpperLeg.jointPoint[1] + vec.mData[2] * rightFootLowerLeg.length;
		rightFootLowerLeg.jointPoint[2] = rightKneeUpperLeg.jointPoint[2] + -vec.mData[1] * rightFootLowerLeg.length;

		//Finding left foot from the left knee point
		q = avatar.b9;// .Inverse().mutiplication(avatar.b9);
		vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

		leftFootLowerLeg.jointPoint[0] = leftKneeUpperLeg.jointPoint[0] + vec.mData[0] * leftFootLowerLeg.length;
		leftFootLowerLeg.jointPoint[1] = leftKneeUpperLeg.jointPoint[1] + vec.mData[2] * leftFootLowerLeg.length;;
		leftFootLowerLeg.jointPoint[2] = leftKneeUpperLeg.jointPoint[2] + -vec.mData[1] * leftFootLowerLeg.length;
	}
	//---------------------------------------------Updating UpperBody starting from pelvis position --------------------
	//Finding sternum from the pelvis point
	q = avatar.b0;// .Inverse().mutiplication(avatar.b0);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	sternumChest.jointPoint[0] = pelvisStomach.jointPoint[0] + vec.mData[0] * pelvisStomach.length;
	sternumChest.jointPoint[1] = pelvisStomach.jointPoint[1] + vec.mData[2] * pelvisStomach.length;
	sternumChest.jointPoint[2] = pelvisStomach.jointPoint[2] + -vec.mData[1] * pelvisStomach.length;

	//Finding neck from the sternum point
	q = avatar.b1;// .Inverse().mutiplication(avatar.b1);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	neckToChin.jointPoint[0] = sternumChest.jointPoint[0] + vec.mData[0] * sternumChest.length;
	neckToChin.jointPoint[1] = sternumChest.jointPoint[1] + vec.mData[2] * sternumChest.length;
	neckToChin.jointPoint[2] = sternumChest.jointPoint[2] + -vec.mData[1] * sternumChest.length;

	headQuat = avatar.b1; //assigning headQuat for head rotation

	//Finding head from the neck point
	q = avatar.b1;// .Inverse().mutiplication(avatar.b1);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	chinToHead.jointPoint[0] = neckToChin.jointPoint[0] + vec.mData[0] * neckToChin.length;
	chinToHead.jointPoint[1] = neckToChin.jointPoint[1] + vec.mData[2] * neckToChin.length;
	chinToHead.jointPoint[2] = neckToChin.jointPoint[2] + -vec.mData[1] * neckToChin.length;

	head.jointPoint[0] = chinToHead.jointPoint[0] + vec.mData[0] * chinToHead.length;
	head.jointPoint[1] = chinToHead.jointPoint[1] + vec.mData[2] * chinToHead.length;
	head.jointPoint[2] = chinToHead.jointPoint[2] + -vec.mData[1] * chinToHead.length;

	//---------------------------------------------- Hands ----------------------------------------------------
		//Finding rightShoulder from the neck point
	q = avatar.b1;// .Inverse().mutiplication(avatar.b1);
	vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

	rightShoulderSegment.jointPoint[0] = neckToChin.jointPoint[0] + vec.mData[0] * rightShoulderSegment.length;
	rightShoulderSegment.jointPoint[1] = neckToChin.jointPoint[1] + vec.mData[2] * rightShoulderSegment.length;
	rightShoulderSegment.jointPoint[2] = neckToChin.jointPoint[2] + -vec.mData[1] * rightShoulderSegment.length;

	//Finding leftShoulder from the neck point
	q = avatar.b1;// .Inverse().mutiplication(avatar.b1);
	vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

	leftShoulderSegment.jointPoint[0] = neckToChin.jointPoint[0] + vec.mData[0] * leftShoulderSegment.length;
	leftShoulderSegment.jointPoint[1] = neckToChin.jointPoint[1] + vec.mData[2] * leftShoulderSegment.length;
	leftShoulderSegment.jointPoint[2] = neckToChin.jointPoint[2] + -vec.mData[1] * leftShoulderSegment.length;

	//Finding right elbow from the right shoulder point
	q = avatar.b2;// .Inverse().mutiplication(avatar.b2);
	vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

	rightElbowUpperArm.jointPoint[0] = rightShoulderSegment.jointPoint[0] + vec.mData[0] * rightElbowUpperArm.length;
	rightElbowUpperArm.jointPoint[1] = rightShoulderSegment.jointPoint[1] + vec.mData[2] * rightElbowUpperArm.length;
	rightElbowUpperArm.jointPoint[2] = rightShoulderSegment.jointPoint[2] + -vec.mData[1] * rightElbowUpperArm.length;

	//Finding left elbow from the left shoulder point
	q = avatar.b4;// .Inverse().mutiplication(avatar.b4);
	vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

	leftElbowUpperArm.jointPoint[0] = leftShoulderSegment.jointPoint[0] + vec.mData[0] * leftElbowUpperArm.length;
	leftElbowUpperArm.jointPoint[1] = leftShoulderSegment.jointPoint[1] + vec.mData[2] * leftElbowUpperArm.length;
	leftElbowUpperArm.jointPoint[2] = leftShoulderSegment.jointPoint[2] + -vec.mData[1] * leftElbowUpperArm.length;

	//Finding right hand from the right elbow point
	q = avatar.b3;// .Inverse().mutiplication(avatar.b3);
	rightHand.rotation = q;
	vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

	rightHandLowerArm.jointPoint[0] = rightElbowUpperArm.jointPoint[0] + vec.mData[0] * rightHandLowerArm.length;
	rightHandLowerArm.jointPoint[1] = rightElbowUpperArm.jointPoint[1] + vec.mData[2] * rightHandLowerArm.length;
	rightHandLowerArm.jointPoint[2] = rightElbowUpperArm.jointPoint[2] + -vec.mData[1] * rightHandLowerArm.length;

	//Finding left hand from the left elbow point
	q = avatar.b5;// .Inverse().mutiplication(avatar.b5);
	leftHand.rotation = q;
	vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

	leftHandLowerArm.jointPoint[0] = leftElbowUpperArm.jointPoint[0] + vec.mData[0] * leftHandLowerArm.length;
	leftHandLowerArm.jointPoint[1] = leftElbowUpperArm.jointPoint[1] + vec.mData[2] * leftHandLowerArm.length;
	leftHandLowerArm.jointPoint[2] = leftElbowUpperArm.jointPoint[2] + -vec.mData[1] * leftHandLowerArm.length;

	drawAvatar();
}

double myRound(double var)
{
	// 37.66666 * 100 =3766.66 
	// 3766.66 + .5 =3767.16    for rounding off value 
	// then type cast to int so value is 3767 
	// then divided by 100 so the value converted into 37.67 
	double value = (int)(var * 100 + .5);
	return (double)value / 100;
}

void rotateAvatar(int index)
{
	Avatar currentAvatar = vitruvianSphereUtility.avatarData[index];
	Avatar initialAvatar = vitruvianSphereUtility.avatarData[1];
	//--------------------------------------- Translate lower body to ground point ---------------------------
	double diffR = rightFootLowerLeg.jointPoint[1];
	double diffL = leftFootLowerLeg.jointPoint[1];

	rightFootLowerLeg.jointPoint[0] = rightFootLowerLeg.jointPoint[0];
	rightFootLowerLeg.jointPoint[1] = 0;
	rightFootLowerLeg.jointPoint[2] = rightFootLowerLeg.jointPoint[2];

	leftFootLowerLeg.jointPoint[0] = leftFootLowerLeg.jointPoint[0];
	leftFootLowerLeg.jointPoint[1] = 0;
	leftFootLowerLeg.jointPoint[2] = leftFootLowerLeg.jointPoint[2];

	rightKneeUpperLeg.jointPoint[1] = rightKneeUpperLeg.jointPoint[1] - diffR;
	rightHipSegment.jointPoint[1] = rightHipSegment.jointPoint[1] - diffR;

	leftKneeUpperLeg.jointPoint[1] = leftKneeUpperLeg.jointPoint[1] - diffL;
	leftHipSegment.jointPoint[1] = leftHipSegment.jointPoint[1] - diffL;


	//---------------------------------------- Bottom-Up Update -----------------------------------------------
	//Finding right knee joint from the fixed right foot.
	quaternion q = initialAvatar.b7.Inverse().mutiplication(currentAvatar.b7);
	quaternion vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	rightKneeUpperLeg.jointPoint[0] = rightFootLowerLeg.jointPoint[0] + vec.mData[0] * rightFootLowerLeg.length;
	rightKneeUpperLeg.jointPoint[1] = 0 + vec.mData[2] * rightFootLowerLeg.length;
	rightKneeUpperLeg.jointPoint[2] = rightFootLowerLeg.jointPoint[2] + -vec.mData[1] * rightFootLowerLeg.length;

	//Finding left knee joint from the fixed left foot.
	q = initialAvatar.b9.Inverse().mutiplication(currentAvatar.b9);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	leftKneeUpperLeg.jointPoint[0] = leftFootLowerLeg.jointPoint[0] + vec.mData[0] * leftFootLowerLeg.length;
	leftKneeUpperLeg.jointPoint[1] = 0 + vec.mData[2] * leftFootLowerLeg.length;
	leftKneeUpperLeg.jointPoint[2] = leftFootLowerLeg.jointPoint[2] + -vec.mData[1] * leftFootLowerLeg.length;

	//Finding right pelvis joint from the right knee.
	q = initialAvatar.b6.Inverse().mutiplication(currentAvatar.b6);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	rightHipSegment.jointPoint[0] = rightKneeUpperLeg.jointPoint[0] + vec.mData[0] * rightKneeUpperLeg.length;
	rightHipSegment.jointPoint[1] = rightKneeUpperLeg.jointPoint[1] + vec.mData[2] * rightKneeUpperLeg.length;
	rightHipSegment.jointPoint[2] = rightKneeUpperLeg.jointPoint[2] + -vec.mData[1] * rightKneeUpperLeg.length;

	//Finding left pelvis joint from the left knee.
	q = initialAvatar.b8.Inverse().mutiplication(currentAvatar.b8);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	leftHipSegment.jointPoint[0] = leftKneeUpperLeg.jointPoint[0] + vec.mData[0] * leftKneeUpperLeg.length;
	leftHipSegment.jointPoint[1] = leftKneeUpperLeg.jointPoint[1] + vec.mData[2] * leftKneeUpperLeg.length;
	leftHipSegment.jointPoint[2] = leftKneeUpperLeg.jointPoint[2] - vec.mData[1] * leftKneeUpperLeg.length;

	//Finding pelvis joint from the left and right pelvis.
	if (leftHipSegment.jointPoint[1] > rightHipSegment.jointPoint[1])
	{
		q = initialAvatar.b0.Inverse().mutiplication(currentAvatar.b0);
		vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

		pelvisStomach.jointPoint[0] = leftHipSegment.jointPoint[0] + vec.mData[0] * leftHipSegment.length;
		pelvisStomach.jointPoint[1] = leftHipSegment.jointPoint[1] + vec.mData[2] * leftHipSegment.length;
		pelvisStomach.jointPoint[2] = leftHipSegment.jointPoint[2] - vec.mData[1] * leftHipSegment.length;

		/*float diffR = leftHipSegment.jointPoint[1] - rightHipSegment.jointPoint[1];
		rightKneeUpperLeg.jointPoint[1] = rightKneeUpperLeg.jointPoint[1] + diffR;
		rightFootLowerLeg.jointPoint[1] = rightFootLowerLeg.jointPoint[1] + diffR;
		rightHipSegment.jointPoint[1] = rightHipSegment.jointPoint[1] + diffR;

		diffR = leftHipSegment.jointPoint[2] - rightHipSegment.jointPoint[2];;
		rightKneeUpperLeg.jointPoint[2] = rightKneeUpperLeg.jointPoint[2] + diffR;
		rightFootLowerLeg.jointPoint[2] = rightFootLowerLeg.jointPoint[2] + diffR;
		rightHipSegment.jointPoint[2] = rightHipSegment.jointPoint[2] + diffR;*/

		//	isEqual = false;

	}
	else if (rightHipSegment.jointPoint[1] >= leftHipSegment.jointPoint[1])
	{
		q = initialAvatar.b0.Inverse().mutiplication(currentAvatar.b0);
		vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

		pelvisStomach.jointPoint[0] = rightHipSegment.jointPoint[0] + vec.mData[0] * rightHipSegment.length;
		pelvisStomach.jointPoint[1] = rightHipSegment.jointPoint[1] + vec.mData[2] * rightHipSegment.length;
		pelvisStomach.jointPoint[2] = rightHipSegment.jointPoint[2] - vec.mData[1] * rightHipSegment.length;

		/*float diffR = rightHipSegment.jointPoint[1] - leftHipSegment.jointPoint[1];
		leftKneeUpperLeg.jointPoint[1]	= leftKneeUpperLeg.jointPoint[1] + diffR;
		leftFootLowerLeg.jointPoint[1]	= leftFootLowerLeg.jointPoint[1] + diffR;
		leftHipSegment.jointPoint[1]	= leftHipSegment.jointPoint[1] + diffR;

		diffR = rightHipSegment.jointPoint[2] - leftHipSegment.jointPoint[2];
		leftKneeUpperLeg.jointPoint[2]	= leftKneeUpperLeg.jointPoint[2] + diffR;
		leftFootLowerLeg.jointPoint[2]	= leftFootLowerLeg.jointPoint[2] + diffR;
		leftHipSegment.jointPoint[2]	= leftHipSegment.jointPoint[2]   + diffR;*/
		// isEqual = false;
	}
	/*else
	{
		pelvisStomach.jointPoint[0] = (rightHipSegment.jointPoint[0] + leftHipSegment.jointPoint[0]) / 2;
		pelvisStomach.jointPoint[1] = (rightHipSegment.jointPoint[1] + leftHipSegment.jointPoint[1]) / 2;
		pelvisStomach.jointPoint[2] = (rightHipSegment.jointPoint[2] + leftHipSegment.jointPoint[2]) / 2;
		isEqual = true;
	}*/

	//----------------------------------------Top-Down Update---------------------------------------------------------------------------------------
			//Finding right pelvis joint from the pelvis.
	q = initialAvatar.b0.Inverse().mutiplication(currentAvatar.b0);
	vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

	rightHipSegment.jointPoint[0] = pelvisStomach.jointPoint[0] + vec.mData[0] * rightHipSegment.length;
	rightHipSegment.jointPoint[1] = pelvisStomach.jointPoint[1] + vec.mData[2] * rightHipSegment.length; //Changes in Lower body with pelvis rotation and model moves continously in z axis
	rightHipSegment.jointPoint[2] = pelvisStomach.jointPoint[2] - vec.mData[1] * rightHipSegment.length;//Changes in Lower body with pelvis rotation and model moves continously in z axis

	//Finding left pelvis joint from the pelvis.
	q = initialAvatar.b0.Inverse().mutiplication(currentAvatar.b0);
	vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

	leftHipSegment.jointPoint[0] = pelvisStomach.jointPoint[0] + vec.mData[0] * leftHipSegment.length;
	leftHipSegment.jointPoint[1] = pelvisStomach.jointPoint[1] + vec.mData[2] * leftHipSegment.length;  //Changes in Lower body with pelvis rotation and model moves continously in z axis
	leftHipSegment.jointPoint[2] = pelvisStomach.jointPoint[2] - vec.mData[1] * leftHipSegment.length;	//Changes in Lower body with pelvis rotation and model moves continously in z axis

//if (!isEqual) //Execute topdown approach only when left and right hips are unequal
	{
		//Finding right knee from the right pelvis point
		q = initialAvatar.b6.Inverse().mutiplication(currentAvatar.b6);
		vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

		rightKneeUpperLeg.jointPoint[0] = (rightHipSegment.jointPoint[0] + vec.mData[0] * rightKneeUpperLeg.length);
		rightKneeUpperLeg.jointPoint[1] = rightHipSegment.jointPoint[1] + vec.mData[2] * rightKneeUpperLeg.length;
		rightKneeUpperLeg.jointPoint[2] = rightHipSegment.jointPoint[2] + -vec.mData[1] * rightKneeUpperLeg.length;

		//Finding left knee from the left pelvis point
		q = initialAvatar.b8.Inverse().mutiplication(currentAvatar.b8);
		vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

		leftKneeUpperLeg.jointPoint[0] = leftHipSegment.jointPoint[0] + vec.mData[0] * leftKneeUpperLeg.length;
		leftKneeUpperLeg.jointPoint[1] = leftHipSegment.jointPoint[1] + vec.mData[2] * leftKneeUpperLeg.length;
		leftKneeUpperLeg.jointPoint[2] = leftHipSegment.jointPoint[2] + -vec.mData[1] * leftKneeUpperLeg.length;

		//Finding right foot from the right knee point
		q = initialAvatar.b7.Inverse().mutiplication(currentAvatar.b7);
		vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

		rightFootLowerLeg.jointPoint[0] = rightKneeUpperLeg.jointPoint[0] + vec.mData[0] * rightFootLowerLeg.length;
		rightFootLowerLeg.jointPoint[1] = rightKneeUpperLeg.jointPoint[1] + vec.mData[2] * rightFootLowerLeg.length;
		rightFootLowerLeg.jointPoint[2] = rightKneeUpperLeg.jointPoint[2] + -vec.mData[1] * rightFootLowerLeg.length;

		//Finding left foot from the left knee point
		q = initialAvatar.b9.Inverse().mutiplication(currentAvatar.b9);
		vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

		leftFootLowerLeg.jointPoint[0] = leftKneeUpperLeg.jointPoint[0] + vec.mData[0] * leftFootLowerLeg.length;
		leftFootLowerLeg.jointPoint[1] = leftKneeUpperLeg.jointPoint[1] + vec.mData[2] * leftFootLowerLeg.length;;
		leftFootLowerLeg.jointPoint[2] = leftKneeUpperLeg.jointPoint[2] + -vec.mData[1] * leftFootLowerLeg.length;
	}
	//---------------------------------------------Updating UpperBody starting from pelvis position --------------------
	//Finding sternum from the pelvis point
	q = initialAvatar.b0.Inverse().mutiplication(currentAvatar.b0);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	sternumChest.jointPoint[0] = pelvisStomach.jointPoint[0] + vec.mData[0] * pelvisStomach.length;
	sternumChest.jointPoint[1] = pelvisStomach.jointPoint[1] + vec.mData[2] * pelvisStomach.length;
	sternumChest.jointPoint[2] = pelvisStomach.jointPoint[2] + -vec.mData[1] * pelvisStomach.length;

	//Finding neck from the sternum point
	q = initialAvatar.b1.Inverse().mutiplication(currentAvatar.b1);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	neckToChin.jointPoint[0] = sternumChest.jointPoint[0] + vec.mData[0] * sternumChest.length;
	neckToChin.jointPoint[1] = sternumChest.jointPoint[1] + vec.mData[2] * sternumChest.length;
	neckToChin.jointPoint[2] = sternumChest.jointPoint[2] + -vec.mData[1] * sternumChest.length;

	headQuat = q; //assigning headQuat for head rotation

	//Finding head from the neck point
	q = initialAvatar.b1.Inverse().mutiplication(currentAvatar.b1);
	vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

	chinToHead.jointPoint[0] = neckToChin.jointPoint[0] + vec.mData[0] * neckToChin.length;
	chinToHead.jointPoint[1] = neckToChin.jointPoint[1] + vec.mData[2] * neckToChin.length;
	chinToHead.jointPoint[2] = neckToChin.jointPoint[2] + -vec.mData[1] * neckToChin.length;

	head.jointPoint[0] = chinToHead.jointPoint[0] + vec.mData[0] * chinToHead.length;
	head.jointPoint[1] = chinToHead.jointPoint[1] + vec.mData[2] * chinToHead.length;
	head.jointPoint[2] = chinToHead.jointPoint[2] + -vec.mData[1] * chinToHead.length;

	//---------------------------------------------- Hands ----------------------------------------------------
		//Finding rightShoulder from the neck point
	q = initialAvatar.b1.Inverse().mutiplication(currentAvatar.b1);
	vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

	rightShoulderSegment.jointPoint[0] = neckToChin.jointPoint[0] + vec.mData[0] * rightShoulderSegment.length;
	rightShoulderSegment.jointPoint[1] = neckToChin.jointPoint[1] + vec.mData[2] * rightShoulderSegment.length;
	rightShoulderSegment.jointPoint[2] = neckToChin.jointPoint[2] + -vec.mData[1] * rightShoulderSegment.length;

	//Finding leftShoulder from the neck point
	q = initialAvatar.b1.Inverse().mutiplication(currentAvatar.b1);
	vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

	leftShoulderSegment.jointPoint[0] = neckToChin.jointPoint[0] + vec.mData[0] * leftShoulderSegment.length;
	leftShoulderSegment.jointPoint[1] = neckToChin.jointPoint[1] + vec.mData[2] * leftShoulderSegment.length;
	leftShoulderSegment.jointPoint[2] = neckToChin.jointPoint[2] + -vec.mData[1] * leftShoulderSegment.length;

	//Finding right elbow from the right shoulder point
	q = initialAvatar.b2.Inverse().mutiplication(currentAvatar.b2);
	vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

	rightElbowUpperArm.jointPoint[0] = rightShoulderSegment.jointPoint[0] + vec.mData[0] * rightElbowUpperArm.length;
	rightElbowUpperArm.jointPoint[1] = rightShoulderSegment.jointPoint[1] + vec.mData[2] * rightElbowUpperArm.length;
	rightElbowUpperArm.jointPoint[2] = rightShoulderSegment.jointPoint[2] + -vec.mData[1] * rightElbowUpperArm.length;

	//Finding left elbow from the left shoulder point
	q = initialAvatar.b4.Inverse().mutiplication(currentAvatar.b4);
	vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

	leftElbowUpperArm.jointPoint[0] = leftShoulderSegment.jointPoint[0] + vec.mData[0] * leftElbowUpperArm.length;
	leftElbowUpperArm.jointPoint[1] = leftShoulderSegment.jointPoint[1] + vec.mData[2] * leftElbowUpperArm.length;
	leftElbowUpperArm.jointPoint[2] = leftShoulderSegment.jointPoint[2] + -vec.mData[1] * leftElbowUpperArm.length;

	//Finding right hand from the right elbow point
	q = initialAvatar.b3.Inverse().mutiplication(currentAvatar.b3);
	rightHand.rotation = q;
	vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

	rightHandLowerArm.jointPoint[0] = rightElbowUpperArm.jointPoint[0] + vec.mData[0] * rightHandLowerArm.length;
	rightHandLowerArm.jointPoint[1] = rightElbowUpperArm.jointPoint[1] + vec.mData[2] * rightHandLowerArm.length;
	rightHandLowerArm.jointPoint[2] = rightElbowUpperArm.jointPoint[2] + -vec.mData[1] * rightHandLowerArm.length;

	//Finding left hand from the left elbow point
	q = initialAvatar.b5.Inverse().mutiplication(currentAvatar.b5);
	leftHand.rotation = q;
	vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

	leftHandLowerArm.jointPoint[0] = leftElbowUpperArm.jointPoint[0] + vec.mData[0] * leftHandLowerArm.length;
	leftHandLowerArm.jointPoint[1] = leftElbowUpperArm.jointPoint[1] + vec.mData[2] * leftHandLowerArm.length;
	leftHandLowerArm.jointPoint[2] = leftElbowUpperArm.jointPoint[2] + -vec.mData[1] * leftHandLowerArm.length;

	drawAvatar();
}


float i = 1;

// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
	static KeyPressInteractorStyle* New();
	vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

	virtual void OnKeyPress()
	{
		// Get the keypress
		vtkRenderWindowInteractor *rwi = this->Interactor;
		std::string key = rwi->GetKeySym();

		// Output the key that was pressed
		std::cout << "Pressed " << key << std::endl;

		// Handle an arrow key
		if (key == "Up")
		{
			std::cout << "The up arrow was pressed." << std::endl;
		}

		// Handle a "normal" key
		if (key == "a")
		{
			i = 1;
			VitruvianAvatar::isLoaded = true;
		}
		

		// Forward events
		vtkInteractorStyleTrackballCamera::OnKeyPress();
	}

};
vtkStandardNewMacro(KeyPressInteractorStyle);

class vtkTimerCallback : public vtkCommand
{
public:
	static vtkTimerCallback *New()
	{
		vtkTimerCallback *cb = new vtkTimerCallback;
		cb->TimerCount = 0;
		return cb;
	}

	virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId,
		void *vtkNotUsed(callData))
	{
		if (vtkCommand::TimerEvent == eventId)
		{
			++this->TimerCount;
		}
		if (VitruvianAvatar::isLoaded /*&& i < vitruvianSphereUtility.noOfFrames*/)
		{
			//rotateAvatar(i);
			//i++;
			rotateAvatar(VitruvianAvatar::vitruvianAvatarUpdate);
		}		
		renderWindow->Render();
	}

private:
	int TimerCount;

};

void VitruvianAvatar::initializeVetruvianVtkAvatar()
{
	headUnit = VitruvianAvatar::humanHeight / 7.5;
	rightFootLowerLeg.jointPoint[0] = -0.5*headUnit;
	rightFootLowerLeg.jointPoint[1] = rightFootLowerLeg.jointPoint[2] = 0;

	leftFootLowerLeg.jointPoint[0] = 0.5*headUnit;
	leftFootLowerLeg.jointPoint[1] = leftFootLowerLeg.jointPoint[2] = 0;

	leftFootLowerLeg.length = rightFootLowerLeg.length = 1.5 * headUnit;
	leftFootLowerLeg.length = rightFootLowerLeg.length = 1.5 * headUnit;

	leftKneeUpperLeg.length = rightKneeUpperLeg.length = 1.5 * headUnit;
	leftKneeUpperLeg.length = rightKneeUpperLeg.length = 1.5 * headUnit;

	leftHipSegment.length = rightHipSegment.length = 0.5 * headUnit;

	pelvisStomach.length = sternumChest.length = headUnit;
	neckToChin.length = chinToHead.length = headUnit * 0.5;

	rightShoulderSegment.length = leftShoulderSegment.length = 0.8*headUnit;
	rightElbowUpperArm.length = leftElbowUpperArm.length = 1.2*headUnit;
	rightHandLowerArm.length = leftHandLowerArm.length = 1.2*headUnit;

	//------------------------------- read obj files --------------------------------
	// readRightHand.
	rightHand.handSource->SetFileName("RightHand.obj");
	rightHand.handSource->Update();

	// readLeftHand.
	leftHand.handSource->SetFileName("LeftHand.obj");
	leftHand.handSource->Update();

	//readHead
	headSource->SetFileName("humanHead.obj");
	headSource->Update();

	updatejoints();
}

void VitruvianAvatar::startVetruvianAvatar()
{
	//initializeVetruvianVtkAvatar();
	drawFloor();
	drawAvatar();
	vitruvianSphereUtility.readAvatarData("FormFile.txt");
	//Create a renderer, render window, and interactor
	renderWindow->AddRenderer(renderer);
	renderWindow->SetWindowName("Oriented Cylinder");
	renderWindowInteractor->SetRenderWindow(renderWindow);
	renderer->SetBackground(colors->GetColor3d("BkgColor").GetData());

	// Initialize must be called prior to creating timer events.
	renderWindowInteractor->Initialize();

	// Sign up to receive TimerEvent
	vtkSmartPointer<vtkTimerCallback> cb =
		vtkSmartPointer<vtkTimerCallback>::New();
	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);

	int timerId = renderWindowInteractor->CreateRepeatingTimer(10);

	vtkSmartPointer<KeyPressInteractorStyle> style =
		vtkSmartPointer<KeyPressInteractorStyle>::New();
	renderWindowInteractor->SetInteractorStyle(style);
	style->SetCurrentRenderer(renderer);

	//Render and interact
	renderWindow->Render();
	renderWindowInteractor->Start();


}