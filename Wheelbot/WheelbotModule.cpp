//
//  WheelbotModule.cpp
//  ReMod3D
//
//  Created by Thomas Collins on 7/24/13.
//
//

#include "WheelbotModule.h"
#include "WheelbotModule.h"
#include "WheelbotSensors.h"
#include "WheelbotActuators.h"
#include <BlobResult.h>
/*Begin constants associated with Superbot construction*/
const int rm3d::ai::WheelbotModule::posOrientInt = 0;
const int rm3d::ai::WheelbotModule::linearVelocitiesInt = 1;
const int rm3d::ai::WheelbotModule::angularVelocitiesInt = 2;
const int rm3d::ai::WheelbotModule::rangedMessageInt = 3;
const int rm3d::ai::WheelbotModule::obstaclePoseInt = 4;
const int rm3d::ai::WheelbotModule::obstaclePropertiesInt = 5;
const int rm3d::ai::WheelbotModule::xAxisRaycastInt = 6;
const int rm3d::ai::WheelbotModule::negxAxisRaycastInt =7;
const int rm3d::ai::WheelbotModule::colorBlobInt = 8;
const int rm3d::ai::WheelbotModule::flWheelInt = 9;
const int rm3d::ai::WheelbotModule::frWheelInt = 10;
const int rm3d::ai::WheelbotModule::rlWheelInt = 11;
const int rm3d::ai::WheelbotModule::rrWheelInt = 12;
const int rm3d::ai::WheelbotModule::numActuatorsSensors = 13;
/*End Constants Associated with Superbot Construction*/


rm3d::ai::WheelbotModule::WheelbotModule(rm3d::ai::WheelbotSim *sim, string name, const PxTransform& transform) {
    this->name = name;
    this->sim = sim;
    this->init(transform);
}

rm3d::ai::WheelbotModule::~WheelbotModule() {
    for (int i=0; i<this->numDocks; i++) {
        if (this->docks[i])delete this->docks[i];
    }
    if (this->docks != NULL) delete docks;
    if (this->program != NULL)delete this->program;
    if (this->model != NULL){rm3d::ai::WheelbotModel *m = (rm3d::ai::WheelbotModel *)this->model;
        delete m;
    }
    if (this->currentState) delete currentState;
    if (this->desiredState) delete desiredState;
    
}

void rm3d::ai::WheelbotModule::init (const PxTransform& transform) {
    this->executionDelay = 0;
    WheelbotSimBase *rsim = (WheelbotSimBase *) this->sim;
    this->firstStep = true;
    vector<PxVec3> bodyVecs;
    vector<PxVec3> wheelVecs;
    float wheelRadiusInc = wheelbotWheelRadius/pointsPerRadius;
    float circleIncCountWheel = 0.0;
    bodyVecs.push_back(PxVec3(wheelbotHalfWidth, wheelbotHalfHeight, wheelbotHalfDepth));
    bodyVecs.push_back(PxVec3(wheelbotHalfWidth, wheelbotHalfHeight, -wheelbotHalfDepth));
    bodyVecs.push_back(PxVec3(wheelbotHalfWidth, -wheelbotHalfHeight, wheelbotHalfDepth));
    bodyVecs.push_back(PxVec3(wheelbotHalfWidth, -wheelbotHalfHeight, -wheelbotHalfDepth));
    bodyVecs.push_back(PxVec3(-wheelbotHalfWidth, wheelbotHalfHeight, wheelbotHalfDepth));
    bodyVecs.push_back(PxVec3(-wheelbotHalfWidth, wheelbotHalfHeight, -wheelbotHalfDepth));
    bodyVecs.push_back(PxVec3(-wheelbotHalfWidth, -wheelbotHalfHeight, wheelbotHalfDepth));
    bodyVecs.push_back(PxVec3(-wheelbotHalfWidth, -wheelbotHalfHeight, -wheelbotHalfDepth));
    while (circleIncCountWheel <= wheelbotWheelRadius) {
        if (circleIncCountWheel == 0) {
            wheelVecs.push_back(PxVec3(0,wheelbotWheelRadius,wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(0,wheelbotWheelRadius,-wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(0,-wheelbotWheelRadius,wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(0,-wheelbotWheelRadius,-wheelbotHalfWheelDepth));
        } else if (circleIncCountWheel == wheelbotWheelRadius) {
            wheelVecs.push_back(PxVec3(wheelbotWheelRadius,0,wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(wheelbotWheelRadius,0,-wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(-wheelbotWheelRadius,0,wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(-wheelbotWheelRadius,0,-wheelbotHalfWheelDepth));
        } else {
            wheelVecs.push_back(PxVec3(wheelbotWheelRadius - circleIncCountWheel,sqrt(powf(wheelbotWheelRadius,2.0) - powf(wheelbotWheelRadius - circleIncCountWheel, 2.0)),-wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(wheelbotWheelRadius - circleIncCountWheel,-sqrt(powf(wheelbotWheelRadius,2.0) - powf(wheelbotWheelRadius - circleIncCountWheel, 2.0)),-wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(-wheelbotWheelRadius + circleIncCountWheel,sqrt(powf(wheelbotWheelRadius,2.0) - powf(-wheelbotWheelRadius + circleIncCountWheel, 2.0)),-wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(-wheelbotWheelRadius + circleIncCountWheel,-sqrt(powf(wheelbotWheelRadius,2.0) - powf(-wheelbotWheelRadius + circleIncCountWheel, 2.0)),-wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(wheelbotWheelRadius - circleIncCountWheel,sqrt(powf(wheelbotWheelRadius,2.0) - powf(wheelbotWheelRadius - circleIncCountWheel, 2.0)),wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(wheelbotWheelRadius - circleIncCountWheel,-sqrt(powf(wheelbotWheelRadius,2.0) - powf(wheelbotWheelRadius - circleIncCountWheel, 2.0)),wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(-wheelbotWheelRadius + circleIncCountWheel,sqrt(powf(wheelbotWheelRadius,2.0) - powf(-wheelbotWheelRadius + circleIncCountWheel, 2.0)),wheelbotHalfWheelDepth));
            wheelVecs.push_back(PxVec3(-wheelbotWheelRadius + circleIncCountWheel,-sqrt(powf(wheelbotWheelRadius,2.0) - powf(-wheelbotWheelRadius + circleIncCountWheel, 2.0)),wheelbotHalfWheelDepth));
        }
        
        circleIncCountWheel += wheelRadiusInc;
    }
    //Convex mesh for inner shapes
    WheelbotSimBase * simulator = (WheelbotSimBase *) sim;
    PxConvexMesh *bodyMesh = rm3d::SimulationUtility::GenerateConvex(*(simulator->getSimulationPhysics()),*simulator->getSimulationCooking(), bodyVecs.size(), &bodyVecs[0]);
    PxConvexMesh *wheelMesh = rm3d::SimulationUtility::GenerateConvex(*(simulator->getSimulationPhysics()),*simulator->getSimulationCooking(), wheelVecs.size(), &wheelVecs[0]);
    bodyMaterial = simulator->getSimulationPhysics()->createMaterial(wheelbotStaticFrictionBody,wheelbotDynamicFrictionBody,wheelbotRestitutionBody);
    wheelMaterial = simulator->getSimulationPhysics()->createMaterial(wheelbotStaticFrictionWheel,wheelbotDynamicFrictionWheel,wheelbotRestitutionWheel);
    wheelbot = simulator->getSimulationPhysics()->createArticulation();
    body = wheelbot->createLink(NULL, transform);
    bodyShape = body->createShape(PxConvexMeshGeometry(bodyMesh), *bodyMaterial);
    bodyShape->setLocalPose(PxTransform(PxVec3(0), PxQuat(PxHalfPi, PxVec3(1,0,0))));
    leftFrontWheel = wheelbot->createLink(body, transform*PxTransform(PxVec3(wheelbotHalfWidth,0,-wheelbotHalfHeight - wheelbotWheelDepth/2.0),
                                                                      PxQuat(0, PxVec3(0,1,0))));
    leftFrontWheelShape = leftFrontWheel->createShape(PxConvexMeshGeometry(wheelMesh),*wheelMaterial);
    
    leftBackWheel = wheelbot->createLink(body,transform*PxTransform(PxVec3(-wheelbotHalfWidth,0,-wheelbotHalfHeight - wheelbotWheelDepth/2.0),
                                                                    PxQuat(0, PxVec3(0,1,0))));
    leftBackWheelShape = leftBackWheel->createShape(PxConvexMeshGeometry(wheelMesh),*wheelMaterial);
    
    rightFrontWheel = wheelbot->createLink(body, transform*PxTransform(PxVec3(wheelbotHalfWidth,0,wheelbotHalfHeight + wheelbotWheelDepth/2.0),
                                                                       PxQuat(0, PxVec3(0,1,0))));
    rightFrontWheelShape = rightFrontWheel->createShape(PxConvexMeshGeometry(wheelMesh),*wheelMaterial);
    
    rightBackWheel = wheelbot->createLink(body, transform*PxTransform(PxVec3(-wheelbotHalfWidth,0,wheelbotHalfHeight + wheelbotWheelDepth/2.0),
                                                                      PxQuat(0, PxVec3(0,1,0))));
    rightBackWheelShape = rightBackWheel->createShape(PxConvexMeshGeometry(wheelMesh),*wheelMaterial);
    
    this->bodyString = this->name + "Body";
    this->frontLeftWheelString = this->name + "FL";
    this->backLeftWheelString = this->name + "RL";
    this->backRightWheelString = this->name + "RR";
    this->frontRightWheelString = this->name + "FR";
    //set names for shapes
    body->setName(bodyString.c_str());
    rightFrontWheel->setName(this->frontRightWheelString.c_str());
    rightBackWheel->setName(this->backRightWheelString.c_str());
    leftBackWheel->setName(this->backLeftWheelString.c_str());
    leftFrontWheel->setName(this->frontLeftWheelString.c_str());
    
    simulator->addColorToColorsDictionary(Color(70.0/256.0,70.0/256.0,70.0/256.0), this->bodyString);
    simulator->addColorToColorsDictionary(Color(0,0,1), this->frontLeftWheelString);
    simulator->addColorToColorsDictionary(Color(1,0,0), this->frontRightWheelString);
    simulator->addColorToColorsDictionary(Color(0,1,0), this->backLeftWheelString);
    simulator->addColorToColorsDictionary(Color(1,1,0), this->backRightWheelString);
    PxRigidBodyExt::setMassAndUpdateInertia(*body, wheelbotBodyMass);
    PxRigidBodyExt::setMassAndUpdateInertia(*leftFrontWheel, wheelbotWheelMass);
    PxRigidBodyExt::setMassAndUpdateInertia(*rightFrontWheel, wheelbotWheelMass);
    PxRigidBodyExt::setMassAndUpdateInertia(*leftBackWheel, wheelbotWheelMass);
    PxRigidBodyExt::setMassAndUpdateInertia(*rightBackWheel, wheelbotWheelMass);
    
    wheelbot->setSeparationTolerance(0.001f);
    
    frontLeftJoint = leftFrontWheel->getInboundJoint();
    frontRightJoint = rightFrontWheel->getInboundJoint();
    backLeftJoint = leftBackWheel->getInboundJoint();
    backRightJoint = rightBackWheel->getInboundJoint();
    
    
    leftFrontWheel->getInboundJoint()->setParentPose(PxTransform(PxVec3(wheelbotHalfWidth,0,-wheelbotHalfHeight),PxQuat(-PxHalfPi,PxVec3(0,1,0))));
    leftFrontWheel->getInboundJoint()->setChildPose(PxTransform(PxVec3(0,0,wheelbotWheelDepth/2.0),PxQuat(-PxHalfPi,PxVec3(0,1,0))));
    
    rightFrontWheel->getInboundJoint()->setParentPose(PxTransform(PxVec3(wheelbotHalfWidth,0,wheelbotHalfHeight),PxQuat(-PxHalfPi,PxVec3(0,1,0))));
    rightFrontWheel->getInboundJoint()->setChildPose(PxTransform(PxVec3(0,0,-wheelbotWheelDepth/2.0),PxQuat(-PxHalfPi,PxVec3(0,1,0))));
    
    leftBackWheel->getInboundJoint()->setParentPose(PxTransform(PxVec3(-wheelbotHalfWidth,0,-wheelbotHalfHeight),PxQuat(-PxHalfPi,PxVec3(0,1,0))));
    leftBackWheel->getInboundJoint()->setChildPose(PxTransform(PxVec3(0,0,wheelbotWheelDepth/2.0),PxQuat(-PxHalfPi,PxVec3(0,1,0))));
    
    rightBackWheel->getInboundJoint()->setParentPose(PxTransform(PxVec3(-wheelbotHalfWidth,0,wheelbotHalfHeight),PxQuat(-PxHalfPi,PxVec3(0,1,0))));
    rightBackWheel->getInboundJoint()->setChildPose(PxTransform(PxVec3(0,0,-wheelbotWheelDepth/2.0),PxQuat(-PxHalfPi,PxVec3(0,1,0))));
    
    
    
    
    
    frontLeftJoint->setSwingLimit(0.001,0.001);
    frontRightJoint->setSwingLimit(0.001,0.001);
    backLeftJoint->setSwingLimit(0.001,0.001);
    backRightJoint->setSwingLimit(0.001,0.001);
    
    
    frontLeftJoint->setSwingLimitEnabled(true);
    frontRightJoint->setSwingLimitEnabled(true);
    backLeftJoint->setSwingLimitEnabled(true);
    backRightJoint->setSwingLimitEnabled(true);
    
    frontLeftJoint->setTwistLimitEnabled(false);
    frontRightJoint->setTwistLimitEnabled(false);
    backLeftJoint->setTwistLimitEnabled(false);
    backRightJoint->setTwistLimitEnabled(false);
    
    //frontLeftJoint->setTargetOrientation(PxQuat(0,PxVec3(1,0,0)));
    //frontRightJoint->setTargetOrientation(PxQuat(0,PxVec3(1,0,0)));
    //backLeftJoint->setTargetOrientation(PxQuat(0,PxVec3(1,0,0)));
    //backRightJoint->setTargetOrientation(PxQuat(0,PxVec3(1,0,0)));
    
    frontLeftJoint->setExternalCompliance(.001);
    frontLeftJoint->setInternalCompliance(.001);
    frontRightJoint->setExternalCompliance(.001);
    frontRightJoint->setInternalCompliance(.001);
    backLeftJoint->setInternalCompliance(.001);
    backLeftJoint->setExternalCompliance(.001);
    backRightJoint->setInternalCompliance(.001);
    backRightJoint->setExternalCompliance(.001);
    
    frontLeftJoint->setSpring(0);
    frontRightJoint->setSpring(0);
    backLeftJoint->setSpring(0);
    backRightJoint->setSpring(0);
    
    frontLeftJoint->setDamping(2000);
    frontRightJoint->setDamping(2000);
    backLeftJoint->setDamping(2000);
    backRightJoint->setDamping(2000);
    
    
    frontLeftJoint->setTargetVelocity(PxVec3(0,0,0));
    frontRightJoint->setTargetVelocity(PxVec3(0,0,0));
    backLeftJoint->setTargetVelocity(PxVec3(0,0,0));
    backRightJoint->setTargetVelocity(PxVec3(0,0,0));
    
    wheelbot->setSolverIterationCounts(255);
    
    simulator->getSimulationScene()->addArticulation(*wheelbot);
    //Set names for rigid bodies (for coloring purposes)
    this->currentState = new rm3d::ai::WheelbotState();
    this->desiredState = new rm3d::ai::WheelbotState();
    this->numDocks = 6;
    this->docks = (rm3d::ai::Dock<PxShape*, PxShape*, PxArticulationLink*, PxFixedJoint*>**)new rm3d::ai::WheelbotDock*[this->numDocks];
    for (int i=0; i<this->numDocks; i++) {
        this->docks[i] = NULL;
    }
    for (int i=0; i<this->numActuatorsSensors; i++) {
        this->sensors[i] = NULL;
        this->actuators[i] = NULL;
        this->percepts[i] = NULL;
        this->actions[i] = NULL;
    }
    
    this->numLinks = 1;
    this->links = new PxArticulationLink*[this->numLinks];
    this->links[0] = body;
    this->numJoints = 0;
    this->joints = NULL;
    this->mBody = wheelbot;
    
    
    this->sensors[posOrientInt] = (rm3d::ai::Sensor *) new rm3d::ai::WheelbotPositionOrientationSensor(posOrientInt);
    this->sensors[linearVelocitiesInt] = (rm3d::ai::Sensor *) new rm3d::ai::WheelbotLinearVelocitiesSensor(linearVelocitiesInt);
    this->sensors[angularVelocitiesInt] = (rm3d::ai::Sensor *) new rm3d::ai::WheelbotAngularVelocitiesSensor(angularVelocitiesInt);
    this->sensors[rangedMessageInt] = (rm3d::ai::Sensor *) new rm3d::ai::WheelbotRangedMessageSensor(rangedMessageInt, sim);
    this->sensors[obstaclePoseInt] = (rm3d::ai::Sensor *) new rm3d::ai::WheelbotObstaclePoseSensor(obstaclePoseInt);
    this->sensors[obstaclePropertiesInt] = (rm3d::ai::Sensor *) new rm3d::ai::WheelbotObstaclePropertiesSensor(obstaclePropertiesInt);
    this->sensors[xAxisRaycastInt] = (rm3d::ai::Sensor *) new rm3d::ai::WheelbotXAxisRaycastSensor(xAxisRaycastInt, 0.3);
    this->sensors[negxAxisRaycastInt] = (rm3d::ai::Sensor *) new rm3d::ai::WheelbotNegXAxisRaycastSensor(negxAxisRaycastInt, 1.0);
    this->sensors[colorBlobInt] = (rm3d::ai::Sensor *) new rm3d::ai::WheelbotColorBlobSensor(colorBlobInt);
    
    
    this->percepts[posOrientInt] = (rm3d::ai::Percept *)new rm3d::ai::WheelbotPositionOrientationPercept(0.0, posOrientInt);
    this->percepts[linearVelocitiesInt] = (rm3d::ai::Percept *) new rm3d::ai::WheelbotLinearVelocityPercept(0.0, linearVelocitiesInt);
    this->percepts[angularVelocitiesInt] = (rm3d::ai::Percept *) new rm3d::ai::WheelbotAngularVelocityPercept(0.0, angularVelocitiesInt);
    this->percepts[rangedMessageInt] = (rm3d::ai::Percept *) new rm3d::ai::WheelbotMessagePercept(0.0, rangedMessageInt);
    this->percepts[obstaclePoseInt] = (rm3d::ai::Percept *) new rm3d::ai::WheelbotObstaclePercept(0.0, obstaclePoseInt);
    this->percepts[obstaclePropertiesInt] = (rm3d::ai::Percept *) new rm3d::ai::WheelbotObstaclePercept(0.0, obstaclePropertiesInt);
    this->percepts[xAxisRaycastInt] = (rm3d::ai::Percept *) new rm3d::ai::WheelbotRaycastPercept(0.0, xAxisRaycastInt);
    this->percepts[negxAxisRaycastInt] = (rm3d::ai::Percept *) new rm3d::ai::WheelbotRaycastPercept(0.0, negxAxisRaycastInt);
    this->percepts[colorBlobInt] = (rm3d::ai::Percept *) new rm3d::ai::WheelbotFramePercept(0.0, colorBlobInt);
    
    
    this->actuators[rangedMessageInt] = (rm3d::ai::Actuator *)new rm3d::ai::WheelbotRangedMessageActuator(rangedMessageInt, sim, 5.0);
    this->actuators[flWheelInt] = (rm3d::ai::Actuator *) new rm3d::ai::WheelbotFrontLeftWheelActuator(flWheelInt);
    this->actuators[frWheelInt] = (rm3d::ai::Actuator *) new rm3d::ai::WheelbotFrontRightWheelActuator(frWheelInt);
    this->actuators[rlWheelInt] = (rm3d::ai::Actuator *) new rm3d::ai::WheelbotBackLeftWheelActuator(rlWheelInt);
    this->actuators[rrWheelInt] = (rm3d::ai::Actuator *) new rm3d::ai::WheelbotBackRightWheelActuator(rrWheelInt);
    
    this->actions[rangedMessageInt] = (rm3d::ai::Action *) new rm3d::ai::WheelbotMessageAction(rangedMessageInt);
    this->actions[flWheelInt] = (rm3d::ai::Action *) new rm3d::ai::WheelbotWheelAction(flWheelInt);
    this->actions[frWheelInt] = (rm3d::ai::Action *) new rm3d::ai::WheelbotWheelAction(frWheelInt);
    this->actions[rlWheelInt] = (rm3d::ai::Action *) new rm3d::ai::WheelbotWheelAction(rlWheelInt);
    this->actions[rrWheelInt] = (rm3d::ai::Action *) new rm3d::ai::WheelbotWheelAction(rrWheelInt);
    
    this->model = (void *)new rm3d::ai::WheelbotModel(10);
}


void rm3d::ai::WheelbotModule::setColorForModule(const Color& colorBody, string bodyName) {
    WheelbotSimBase *simulator  = (WheelbotSimBase *) this->sim;
    simulator->addColorToColorsDictionary(colorBody, bodyName);
}

void rm3d::ai::WheelbotModule::setWheelColors(const Color& colorFLWheelColor, string flWheel,
                    const Color& colorFRWheelColor, string frWheel,
                    const Color& colorRLWheelColor, string rlWheel,
                    const Color& colorRRWheelColor, string rrWheel) {
    WheelbotSimBase *simulator = (WheelbotSimBase *) this->sim;
    simulator->addColorToColorsDictionary(colorFLWheelColor, this->name + flWheel);
    simulator->addColorToColorsDictionary(colorFRWheelColor, this->name + frWheel);
    simulator->addColorToColorsDictionary(colorRLWheelColor, this->name + rlWheel);
    simulator->addColorToColorsDictionary(colorRRWheelColor, this->name + rrWheel);
    
}

void rm3d::ai::WheelbotModule::addForceToBody(const PxVec3& force) {
    this->body->addForce(force);
}

void rm3d::ai::WheelbotModule::changeVelocity(const PxVec3& velocityDiff) {
    this->body->addForce(velocityDiff, PxForceMode::eVELOCITY_CHANGE);
}

void rm3d::ai::WheelbotModule::changeAngularVelocity(const PxVec3& angVelocityDiff) {
    body->addTorque(angVelocityDiff, PxForceMode::eVELOCITY_CHANGE);
}

void rm3d::ai::WheelbotModule::wakeUp() {
    this->wheelbot->wakeUp();
}

bool rm3d::ai::WheelbotModule::isSleeping() {
    return this->wheelbot->isSleeping();
}

rm3d::ai::WheelbotSim *rm3d::ai::WheelbotModule::getSim() {
    return this->sim;
}

void rm3d::ai::WheelbotModule::setJointSolverIterationCount(int count) {
    this->wheelbot->setSolverIterationCounts(count);
}

PxArticulationLink *rm3d::ai::WheelbotModule::getBody() {
    return this->body;
}

PxArticulationLink *rm3d::ai::WheelbotModule::getWheelFL() {
    return this->leftFrontWheel;
}

PxArticulationLink *rm3d::ai::WheelbotModule::getWheelFR() {
    return this->rightFrontWheel;
}

PxArticulationLink *rm3d::ai::WheelbotModule::getWheelRL() {
    return this->leftBackWheel;
}

PxArticulationLink *rm3d::ai::WheelbotModule::getWheelRR() {
    return this->rightBackWheel;
}

void rm3d::ai::WheelbotModule::step() {
    //Wake up module
    if (this->isSleeping()) {
        this->wakeUp();
    }
    WheelbotSimBase *simulator = (WheelbotSimBase *) this->sim;
    //Update state with sensor values observed through percepts
    for(std::map<int,Sensor *>::iterator iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    {
        int k = iter->first;
        //Messages are consumable. Only read them when we can use them. Continue sensing for experience even when we aren't moving yet
        if ((!simulator->getSaveFrames() && k == colorBlobInt) || k==rangedMessageInt ) {continue;}
        if (this->percepts[k] == NULL || this->sensors[k] == NULL) continue;
        this->currentState->sensorValues[k] = this->percepts[k]->addNoise(this->sensors[k]->getValue(this));
        if (firstStep) {this->desiredState->sensorValues[k] = this->currentState->sensorValues[k];}
    }
    if (firstStep) {
        firstStep = false;
    }
    //Update model with current state
    rm3d::ai::WheelbotModel *m = (rm3d::ai::WheelbotModel *)model;
    m->setCurrentState(this->currentState);
    m->setDesiredState(this->desiredState);
    if (this->executionDelay <= 0) {
        this->executionDelay = 0;
        //Read messages here.
        this->currentState->sensorValues[rangedMessageInt] = this->percepts[rangedMessageInt]->addNoise(this->sensors[rangedMessageInt]->getValue(this));
        if (program){
            program->step(model);
            rm3d::ai::ActionLog **actionLog =  program->getActionQueue();
            rm3d::ai::Message<rm3d::ai::WheelbotDock*> **messages = (rm3d::ai::Message<rm3d::ai::WheelbotDock *>**)program->getMessageQueue();
            vector<rm3d::ai::ActionLog*> actionLogVec;
            for (int i=0; i<program->getNumberOfActions(); i++) {
                rm3d::ai::ActionLog* a = actionLog[i];
                this->actuators[a->action]->writeValue(this, this->actions[a->action]->addNoise(this, a->value));
                this->executionDelay = this->executionDelay + a->executionDelay;
                actionLogVec.push_back(a);
            }
            if (program->getNumberOfActions() == 0) {
                actionLogVec.push_back(new rm3d::ai::ActionLog(-1,0,0.0));
            }
            //m->addExperienceObservation(new rm3d::ai::WheelbotState(this->currentState));
            //m->addExperienceAction(actionLogVec);
        }
    } else {
        this->executionDelay = this->executionDelay - 1;
        //m->addExperienceObservation(new rm3d::ai::WheelbotState(this->currentState));
        vector<rm3d::ai::ActionLog*> actionLogVec;
        //actionLogVec.push_back(new rm3d::ai::ActionLog(-1,0,0.0));
        //m->addExperienceAction(actionLogVec);
    }
    
}

rm3d::ai::WheelbotDock** rm3d::ai::WheelbotModule::getDocks() {
    return (rm3d::ai::WheelbotDock **)this->docks;
}

PxArticulationLink** rm3d::ai::WheelbotModule::getLinks() {
    return this->links;
}

void rm3d::ai::WheelbotModule::loadProgram(rm3d::ai::ModuleProgram<rm3d::ai::WheelbotDock *> *program) {
    this->program = (rm3d::ai::ModuleProgram<rm3d::ai::Dock<PxShape*, PxShape*, PxArticulationLink*, PxFixedJoint*> *>*)program;
}

string rm3d::ai::WheelbotModule::getName() {
    return this->name;
}

PxTransform rm3d::ai::WheelbotModule::getPositionOrientation() {
    return this->body->getGlobalPose();
}

void rm3d::ai::WheelbotModule::setName(string name) {
    this->name = name;
}

rm3d::ai::WheelbotModel * rm3d::ai::WheelbotModule::getModel() {
    return (rm3d::ai::WheelbotModel *)this->model;
}
