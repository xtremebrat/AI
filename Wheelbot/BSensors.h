/*
 *  This file is part of ReMod3D
 *  Copyright (C) 2013
 *     Thomas Collins, Nadeesha Ranasinghe, Wei-Min Shen
 */


/*
 * BSensors.h
 * Sensors/percepts for B modules
 */
#ifndef __ReMod3D__BSensors__
#define __ReMod3D__BSensors__

#include <iostream>
#include "Sensor.h"
#include <boost/any.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/type.hpp>
#include "BModule.h"
#include "Percept.h"
namespace rm3d {
    namespace ai {
            class BModule;
    }
}

namespace rm3d {
    namespace ai {
        class BLinearVelocitySensor {
        public:
            static PxVec3 linearVelocityOfModule(rm3d::ai::BModule *module) {
                return module->body->getLinearVelocity();
            }
        };
        
        class BObstacleSensor {
        public:
            static vector<PxTransform> getPosesOfObstacles(rm3d::ai::BModule *module) {
                BSimBase * simBase = (BSimBase *)module->getSim();
                return simBase->getPosesOfObstacles();
            }
            static vector<rm3d::ai::ObstacleProperties> getPropertiesOfObstacles(rm3d::ai::BModule *module) {
                BSimBase * simBase = (BSimBase *)module->getSim();
                return simBase->getPropertiesOfObstacles();
            }
            
        };
        
        class BRaycastSensor {
        public:
            static PxRaycastHit makeRaycast(BModule *module, const PxVec3& origin, const PxVec3& unitDirection, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                rm3d::ai::BDock*,BModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                 rm3d::ai::BDock*,BModDock> *)module->getSim();
                PxRaycastHit hit;
                const PxSceneQueryFlags outputFlags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
                bool status = simulator->getSimulationScene()->raycastSingle(origin, unitDirection, maxDistance, outputFlags, hit);
                return hit;
            }
            
            static PxRaycastHit makeXAxisRaycast(BModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                rm3d::ai::BDock*,BModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                 rm3d::ai::BDock*,BModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.01,0));
                return makeRaycast(module, origin.p, origin.q.getBasisVector0(), maxDistance);
            }
            static PxRaycastHit makeYAxisRaycast(BModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                rm3d::ai::BDock*,BModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                 rm3d::ai::BDock*,BModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.01,0));
                return makeRaycast(module, origin.p, origin.q.getBasisVector1(), maxDistance);
            }
            static PxRaycastHit makeZAxisRaycast(BModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                rm3d::ai::BDock*,BModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                 rm3d::ai::BDock*,BModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.01,0));
                return makeRaycast(module, origin.p, origin.q.getBasisVector2(), maxDistance);
            }
            static PxRaycastHit makeNegXAxisRaycast(BModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                rm3d::ai::BDock*,BModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                 rm3d::ai::BDock*,BModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.01,0));
                return makeRaycast(module, origin.p, -origin.q.getBasisVector0(), maxDistance);
            }
            static PxRaycastHit makeNegYAxisRaycast(BModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                rm3d::ai::BDock*,BModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                 rm3d::ai::BDock*,BModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.01,0));
                return makeRaycast(module, origin.p, -origin.q.getBasisVector1(), maxDistance);
            }
            static PxRaycastHit makeNegZAxisRaycast(BModule *module, PxReal maxDistance) {
                rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                rm3d::ai::BDock*,BModDock> *simulator =
                (rm3d::ai::Simulator<rm3d::ai::BState*, PxRigidDynamic*, PxFixedJoint*, PxShape*, PxShape*, PxRigidDynamic*,PxFixedJoint*,
                 rm3d::ai::BDock*,BModDock> *)module->getSim();
                PxTransform origin = module->body->getGlobalPose()*PxTransform(PxVec3(0,-module->halfSide - 0.01,0));
                return makeRaycast(module, origin.p, -origin.q.getBasisVector2(), maxDistance);
            }
        };
        

        class BAngularVelocitySensor {
        public:
            static PxVec3 angularVelocityOfModule(rm3d::ai::BModule *module) {
                return module->body->getAngularVelocity();
            }
        };

        class BDockSensor {
        public:
            static bool isEnabled(rm3d::ai::BModule *module, BModDock dockName) {
                rm3d::ai::BDock *dock = (rm3d::ai::BDock *)module->docks[dockName];
                if (dock) return dock->getEnabled();
                else return false;
            }
            
            static bool isEngaged(rm3d::ai::BModule *module, BModDock dockName) {
                rm3d::ai::BDock *dock = (rm3d::ai::BDock *)module->docks[dockName];
                if (dock) return dock->getEngaged();
                else return false;
            }
        };
        
        class BXAxisRaycastSensor: rm3d::ai::Sensor {
        public:
            PxReal maxDistance;
            BXAxisRaycastSensor(int name, float maxDistance) {
                this->name = name;
                this->maxDistance = maxDistance;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BRaycastSensor::makeXAxisRaycast(m, this->maxDistance);
            }
        };
        
        class BYAxisRaycastSensor: rm3d::ai::Sensor {
        public:
            PxReal maxDistance;
            BYAxisRaycastSensor(int name, float maxDistance) {
                this->name = name;
                this->maxDistance = maxDistance;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BRaycastSensor::makeYAxisRaycast(m, this->maxDistance);
            }
        };
        
        class BZAxisRaycastSensor: rm3d::ai::Sensor {
        public:
            PxReal maxDistance;
            BZAxisRaycastSensor(int name, float maxDistance) {
                this->name = name;
                this->maxDistance = maxDistance;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BRaycastSensor::makeZAxisRaycast(m, this->maxDistance);
            }
        };
        
        
        class BNegXAxisRaycastSensor: rm3d::ai::Sensor {
        public:
            PxReal maxDistance;
            BNegXAxisRaycastSensor(int name, float maxDistance) {
                this->name = name;
                this->maxDistance = maxDistance;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BRaycastSensor::makeNegXAxisRaycast(m, this->maxDistance);
            }
        };
        
        class BNegYAxisRaycastSensor: rm3d::ai::Sensor {
        public:
            PxReal maxDistance;
            BNegYAxisRaycastSensor(int name, float maxDistance) {
                this->name = name;
                this->maxDistance = maxDistance;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BRaycastSensor::makeNegYAxisRaycast(m, this->maxDistance);
            }
        };
        
        class BNegZAxisRaycastSensor: rm3d::ai::Sensor {
        public:
            PxReal maxDistance;
            BNegZAxisRaycastSensor(int name, float maxDistance) {
                this->name = name;
                this->maxDistance = maxDistance;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BRaycastSensor::makeNegZAxisRaycast(m, this->maxDistance);
            }
        };

        class BXAxisSensor: rm3d::ai::Sensor {
        public:
            BXAxisSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BLinearVelocitySensor::linearVelocityOfModule(m).x;
            }
        };

        class BRotXAxisSensor: rm3d::ai::Sensor {
        public:
            BRotXAxisSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BAngularVelocitySensor::angularVelocityOfModule(m).x;
            }
        };

        class BObstaclePoseSensor: rm3d::ai::Sensor {
        public:
            BObstaclePoseSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BObstacleSensor::getPosesOfObstacles(m);
            }
        };
        
        class BObstaclePropertiesSensor: rm3d::ai::Sensor {
        public:
            BObstaclePropertiesSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BObstacleSensor::getPropertiesOfObstacles(m);
            }
        };

        class BYAxisSensor: rm3d::ai::Sensor {
        public:
            BYAxisSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BLinearVelocitySensor::linearVelocityOfModule(m).y;
            }
        };

        class BRotYAxisSensor: rm3d::ai::Sensor {
        public:
            BRotYAxisSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BAngularVelocitySensor::angularVelocityOfModule(m).y;
            }
        };

        class BZAxisSensor: rm3d::ai::Sensor {
        public:
            BZAxisSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BLinearVelocitySensor::linearVelocityOfModule(m).z;
            }
        };

        class BRotZAxisSensor: rm3d::ai::Sensor {
        public:
            BRotZAxisSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BAngularVelocitySensor::angularVelocityOfModule(m).z;
            }
        };
        
        class BColorBlobSensor: rm3d::ai::Sensor {
        public:
            BColorBlobSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                BSimBase * simBase = (BSimBase *)m->getSim();
                return SimulationUtility::frameToBlobs(simBase->getCurrentSimulatorFrame(), simBase->getFrameColumnCount(), simBase->getFrameRowCount());
            }
        };

        class BFrontDockEnabledSensor: rm3d::ai::Sensor {
        public:
            BFrontDockEnabledSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEnabled(m, FRONT_DOCK);
            }
        };
        class BFrontDockEngagedSensor: rm3d::ai::Sensor {
        public:
            BFrontDockEngagedSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEngaged(m, FRONT_DOCK);
            }
        };
        class BBackDockEnabledSensor: rm3d::ai::Sensor {
        public:
            BBackDockEnabledSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEnabled(m, BACK_DOCK);
            }
        };
        class BBackDockEngagedSensor: rm3d::ai::Sensor {
        public:
            BBackDockEngagedSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEngaged(m, BACK_DOCK);
            }
        };
        class BLeftDockEnabledSensor: rm3d::ai::Sensor {
        public:
            BLeftDockEnabledSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEnabled(m, LEFT_DOCK);
            }
        };
        class BLeftDockEngagedSensor: rm3d::ai::Sensor {
        public:
            BLeftDockEngagedSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEngaged(m, LEFT_DOCK);
            }
        };
        class BRightDockEnabledSensor: rm3d::ai::Sensor {
        public:
            BRightDockEnabledSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEnabled(m, RIGHT_DOCK);
            }
        };
        class BRightDockEngagedSensor: rm3d::ai::Sensor {
        public:
            BRightDockEngagedSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEngaged(m, RIGHT_DOCK);
            }
        };
        class BUpDockEnabledSensor: rm3d::ai::Sensor {
        public:
            BUpDockEnabledSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEnabled(m, UP_DOCK);
            }
        };
        class BUpDockEngagedSensor: rm3d::ai::Sensor {
        public:
            BUpDockEngagedSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEngaged(m, UP_DOCK);
            }
        };
        class BDownDockEnabledSensor: rm3d::ai::Sensor {
        public:
            BDownDockEnabledSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEnabled(m, DOWN_DOCK);
            }
        };
        class BDownDockEngagedSensor: rm3d::ai::Sensor {
        public:
            BDownDockEngagedSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return BDockSensor::isEngaged(m, DOWN_DOCK);
            }
        };
        class BPositionOrientationSensor: rm3d::ai::Sensor {
        public:
            BPositionOrientationSensor(int name) {
                this->name = name;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                return m->body->getGlobalPose();
            }
        };

        class BMessageSensor: rm3d::ai::Sensor {
        public:
            BSim *sim;
            BMessageSensor(int name, BSim *sim) {
                this->name = name;
                this->sim = sim;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                rm3d::ai::Message<rm3d::ai::BDock*>* message;
                vector<rm3d::ai::Message<rm3d::ai::BDock*>*> messages;
                while ((message = sim->getLastMessage(m->getName())) != NULL) {
                    messages.push_back(message);
                }
                return messages;
                
            }
        };

        class BRangedMessageSensor: rm3d::ai::Sensor {
        public:
            BSim *sim;
            BRangedMessageSensor(int name, BSim *sim) {
                this->name = name;
                this->sim = sim;
            }
            boost::any getValue(boost::any module) {
                rm3d::ai::BModule *m = boost::any_cast<rm3d::ai::BModule *>(module);
                rm3d::ai::RangedMessage* message;
                vector<rm3d::ai::RangedMessage*> messages;
                while ((message = sim->getLastRangedMessage(m->getName())) != NULL) {
                    messages.push_back(message);
                }
                return messages;
                
            }
        };

        class BJointPercept: rm3d::ai::Percept{
        public:
            BJointPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                //Here, you could add, for instance, gaussian noise to each motor
                double new_val = boost::any_cast<float>(this->noise) + boost::any_cast<float>(value);
                return new_val;
            }
        };
        class BDockPercept: rm3d::ai::Percept{
            
        public:
            BDockPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                if (((double) rand() / (RAND_MAX)) < boost::any_cast<float>(this->noise)) {
                    return !boost::any_cast<bool>(value);
                } else {
                    return boost::any_cast<bool>(value);
                }
            };
        };
        class BPositionOrientationPercept: rm3d::ai::Percept {
            
        public:
            BPositionOrientationPercept(float noise, int name) {
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

        class BMessagePercept: rm3d::ai::Percept {
        public:
            BMessagePercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };

        class BLinearVelocityPercept: rm3d::ai::Percept {
        public:
            BLinearVelocityPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };

        class BAngularVelocityPercept: rm3d::ai::Percept {
        public:
            BAngularVelocityPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
        class BObstaclePercept: rm3d::ai::Percept {
        public:
            BObstaclePercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
        class BRaycastPercept: rm3d::ai::Percept {
        public:
            BRaycastPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
        class BFramePercept: rm3d::ai::Percept {
        public:
            BFramePercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
        class BColorBlobPercept: rm3d::ai::Percept {
        public:
            BColorBlobPercept(float noise, int name) {
                this->noise = noise;
                this->name = name;
            }
            boost::any addNoise(boost::any value) {
                return value;
            }
        };
        
    };
};
#endif
