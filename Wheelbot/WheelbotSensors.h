//
//  WheelbotSensors.h
//  ReMod3D
//
//  Created by Thomas Collins on 7/24/13.
//
//

#ifndef __ReMod3D__WheelbotSensors__
#define __ReMod3D__WheelbotSensors__

#include <iostream>
#include "Sensor.h"
#include <boost/any.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/type.hpp>
#include "WheelbotModule.h"
#include "Percept.h"
using namespace rm3d::ai::WheelbotConstants;
namespace rm3d {
    namespace ai {
        class WheelbotModule;
    }
}

namespace rm3d {
    namespace ai {
        class WheelbotLinearVelocitySensor {
        public:
            static PxVec3 linearVelocityOfModule(rm3d::ai::WheelbotModule *module) {
                return module->body->getLinearVelocity();
            }
            static PxVec3 linearVelocityFrontLeftWheel(rm3d::ai::WheelbotModule *module) {
                return module->leftFrontWheel->getLinearVelocity();
            }
            static PxVec3 linearVelocityFrontRightWheel(rm3d::ai::WheelbotModule *module) {
                return module->rightFrontWheel->getLinearVelocity();
            }
            static PxVec3 linearVelocityBackLeftWheel(rm3d::ai::WheelbotModule *module) {
                return module->leftBackWheel->getLinearVelocity();
            }
            static PxVec3 linearVelocityBackRightWheel(rm3d::ai::WheelbotModule *module) {
                return module->rightBackWheel->getLinearVelocity();
            }
        };
        
        class WheelbotAngularVelocitySensor {
        public:
            static PxVec3 angVelocityFrontLeftWheel(rm3d::ai::WheelbotModule *module) {
                return module->leftFrontWheel->getAngularVelocity();
            }
            static PxVec3 angVelocityFrontRightWheel(rm3d::ai::WheelbotModule *module) {
                return module->rightFrontWheel->getAngularVelocity();
            }
            static PxVec3 angVelocityBackLeftWheel(rm3d::ai::WheelbotModule *module) {
                return module->leftBackWheel->getAngularVelocity();
            }
            static PxVec3 angVelocityBackRightWheel(rm3d::ai::WheelbotModule *module) {
                return module->rightBackWheel->getAngularVelocity();
            }
            static PxVec3 angularVelocityOfModule(rm3d::ai::WheelbotModule *module) {
                return module->body->getAngularVelocity();
            }
        };
        
        class WheelbotObstacleSensor {
        public:
            static vector<PxTransform> getPosesOfObstacles(rm3d::ai::WheelbotModule *module) {
                WheelbotSimBase * simBase = (WheelbotSimBase *)module->getSim();
                return simBase->getPosesOfObstacles();
            }
            static vector<rm3d::ai::ObstacleProperties> getPropertiesOfObstacles(rm3d::ai::WheelbotModule *module) {
                WheelbotSimBase * simBase = (WheelbotSimBase *)module->getSim();
                return simBase->getPropertiesOfObstacles();
            }
            
        };
        
        class WheelbotRaycastSensor {
        public:
            static PxRaycastHit makeRaycast(WheelbotModule *module, const PxVec3& origin, const PxVec3& unitDirection, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                rm3d::ai::WheelbotDock*,WheelbotModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                 rm3d::ai::WheelbotDock*,WheelbotModDock> *)module->getSim();
                PxRaycastHit hit;
                const PxSceneQueryFlags outputFlags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
                bool status = simulator->getSimulationScene()->raycastSingle(origin, unitDirection, maxDistance, outputFlags, hit);
                return hit;
            }
            
            static PxRaycastHit makeXAxisRaycast(WheelbotModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                rm3d::ai::WheelbotDock*,WheelbotModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                 rm3d::ai::WheelbotDock*,WheelbotModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(rm3d::ai::WheelbotConstants::wheelbotHalfWidth + 0.01,0,0));
                return makeRaycast(module, origin.p, origin.q.getBasisVector0(), maxDistance);
            }
            static PxRaycastHit makeYAxisRaycast(WheelbotModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                rm3d::ai::WheelbotDock*,WheelbotModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                 rm3d::ai::WheelbotDock*,WheelbotModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.01,0));
                return makeRaycast(module, origin.p, origin.q.getBasisVector1(), maxDistance);
            }
            static PxRaycastHit makeZAxisRaycast(WheelbotModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                rm3d::ai::WheelbotDock*,WheelbotModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                 rm3d::ai::WheelbotDock*,WheelbotModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.01,0));
                return makeRaycast(module, origin.p, origin.q.getBasisVector2(), maxDistance);
            }
            static PxRaycastHit makeNegXAxisRaycast(WheelbotModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                rm3d::ai::WheelbotDock*,WheelbotModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                 rm3d::ai::WheelbotDock*,WheelbotModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.001,0));
                return makeRaycast(module, origin.p, -origin.q.getBasisVector0(), maxDistance);
            }
            static PxRaycastHit makeNegYAxisRaycast(WheelbotModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                rm3d::ai::WheelbotDock*,WheelbotModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                 rm3d::ai::WheelbotDock*,WheelbotModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.01,0));
                return makeRaycast(module, origin.p, -origin.q.getBasisVector1(), maxDistance);
            }
            static PxRaycastHit makeNegZAxisRaycast(WheelbotModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                rm3d::ai::WheelbotDock*,WheelbotModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::WheelbotState*, PxArticulationLink*,PxArticulationJoint*, PxShape*, PxShape*, PxArticulation*, PxFixedJoint*,
                 rm3d::ai::WheelbotDock*,WheelbotModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.01,0));
                return makeRaycast(module, origin.p, -origin.q.getBasisVector2(), maxDistance);
            }
        };
        
    
        
        
        class WheelbotAngularVelocitiesSensor: rm3d::ai::Sensor {
        public:
            WheelbotAngularVelocitiesSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::WheelbotModule *m = boost::any_cast<rm3d::ai::WheelbotModule *>(module);
                vector<PxVec3> angularVelocities;
                angularVelocities.push_back(WheelbotAngularVelocitySensor::angularVelocityOfModule(m));
                angularVelocities.push_back(WheelbotAngularVelocitySensor::angVelocityFrontLeftWheel(m));
                angularVelocities.push_back(WheelbotAngularVelocitySensor::angVelocityFrontRightWheel(m));
                angularVelocities.push_back(WheelbotAngularVelocitySensor::angVelocityBackLeftWheel(m));
                angularVelocities.push_back(WheelbotAngularVelocitySensor::angVelocityBackRightWheel(m));
                return angularVelocities;
            }
        };
        
        class WheelbotLinearVelocitiesSensor: rm3d::ai::Sensor {
        public:
            WheelbotLinearVelocitiesSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::WheelbotModule *m = boost::any_cast<rm3d::ai::WheelbotModule *>(module);
                vector<PxVec3> linearVelocities;
                linearVelocities.push_back(WheelbotLinearVelocitySensor::linearVelocityOfModule(m));
                linearVelocities.push_back(WheelbotLinearVelocitySensor::linearVelocityFrontLeftWheel(m));
                linearVelocities.push_back(WheelbotLinearVelocitySensor::linearVelocityFrontRightWheel(m));
                linearVelocities.push_back(WheelbotLinearVelocitySensor::linearVelocityBackLeftWheel(m));
                linearVelocities.push_back(WheelbotLinearVelocitySensor::linearVelocityBackRightWheel(m));
                return linearVelocities;
            }
        };
        
        
        class WheelbotXAxisRaycastSensor: rm3d::ai::Sensor {
        public:
            PxReal maxDistance;
            WheelbotXAxisRaycastSensor(int name, float maxDistance) {
                this->name = name;
                this->maxDistance = maxDistance;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::WheelbotModule *m = boost::any_cast<rm3d::ai::WheelbotModule *>(module);
                return WheelbotRaycastSensor::makeXAxisRaycast(m, this->maxDistance);
            }
        };
        
        class WheelbotNegXAxisRaycastSensor: rm3d::ai::Sensor {
        public:
            PxReal maxDistance;
            WheelbotNegXAxisRaycastSensor(int name, float maxDistance) {
                this->name = name;
                this->maxDistance = maxDistance;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::WheelbotModule *m = boost::any_cast<rm3d::ai::WheelbotModule *>(module);
                return WheelbotRaycastSensor::makeNegXAxisRaycast(m, this->maxDistance);
            }
        };
        
        
        
        class WheelbotObstaclePoseSensor: rm3d::ai::Sensor {
        public:
            WheelbotObstaclePoseSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::WheelbotModule *m = boost::any_cast<rm3d::ai::WheelbotModule *>(module);
                return WheelbotObstacleSensor::getPosesOfObstacles(m);
            }
        };
        
        class WheelbotObstaclePropertiesSensor: rm3d::ai::Sensor {
        public:
            WheelbotObstaclePropertiesSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::WheelbotModule *m = boost::any_cast<rm3d::ai::WheelbotModule *>(module);
                return WheelbotObstacleSensor::getPropertiesOfObstacles(m);
            }
        };
        
        
        class WheelbotColorBlobSensor: rm3d::ai::Sensor {
        public:
            WheelbotColorBlobSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::WheelbotModule *m = boost::any_cast<rm3d::ai::WheelbotModule *>(module);
                WheelbotSimBase * simBase = (WheelbotSimBase *)m->getSim();
                return SimulationUtility::frameToBlobs(simBase->getCurrentSimulatorFrame(), simBase->getFrameColumnCount(), simBase->getFrameRowCount());
            }
        };
        

        class WheelbotPositionOrientationSensor: rm3d::ai::Sensor {
        public:
            WheelbotPositionOrientationSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::WheelbotModule *m = boost::any_cast<rm3d::ai::WheelbotModule *>(module);
                return m->body->getGlobalPose();
            }
        };
        
        
        
        class WheelbotRangedMessageSensor: rm3d::ai::Sensor {
        public:
            WheelbotSim *sim;
            WheelbotRangedMessageSensor(int name, WheelbotSim *sim) {
                this->name = name;
                this->sim = sim;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::WheelbotModule *m = boost::any_cast<rm3d::ai::WheelbotModule *>(module);
                rm3d::ai::RangedMessage* message;
                vector<rm3d::ai::RangedMessage*> messages;
                while ((message = sim->getLastRangedMessage(m->getName())) != NULL) {
                    messages.push_back(message);
                }
                return messages;
                
            }
        };
        
        
        class WheelbotPositionOrientationPercept: rm3d::ai::Percept {
            
        public:
            WheelbotPositionOrientationPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                PxTransform val = boost::any_cast<PxTransform>(value);
                val.p.x += boost::any_cast<float>(this->noise);
                val.p.y += boost::any_cast<float>(this->noise);
                val.p.z += boost::any_cast<float>(this->noise);
                return  val;
            };
        };
        
        class WheelbotMessagePercept: rm3d::ai::Percept {
        public:
            WheelbotMessagePercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
        class WheelbotLinearVelocityPercept: rm3d::ai::Percept {
        public:
            WheelbotLinearVelocityPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
        class WheelbotAngularVelocityPercept: rm3d::ai::Percept {
        public:
            WheelbotAngularVelocityPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
        class WheelbotObstaclePercept: rm3d::ai::Percept {
        public:
            WheelbotObstaclePercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
        class WheelbotRaycastPercept: rm3d::ai::Percept {
        public:
            WheelbotRaycastPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
        class WheelbotFramePercept: rm3d::ai::Percept {
        public:
            WheelbotFramePercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
        class WheelbotColorBlobPercept: rm3d::ai::Percept {
        public:
            WheelbotColorBlobPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
    };
};
#endif /* defined(__ReMod3D__WheelbotSensors__) */
