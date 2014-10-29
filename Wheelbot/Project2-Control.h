//
//  Project2-Control.h
//  ReMod3D
//
//  Created by Thomas Collins on 9/21/13.
//
//

#ifndef ReMod3D_Project2_Control_h
#define ReMod3D_Project2_Control_h


/*
 * Keyboard Map
 * See Camera.h for detailed camera control information
 * Buttons 1,2,3,4,5,6,w,a,s,d,(up arrow),(down arrow),(left arrow),(right arrow) control the camera
 * Holding down the left mouse button and moving the mouse manipulates the orientation of the camera
 * Button p sets program pause state (pause/unpause)
 * Button f switches between Wheelbot view and global view (NOTE: Color blob sensing will not work in global view, and the camera cannot be manually
 * moved in Wheelbot view)
 */

#include "ModuleProgram.h"
#include "WheelbotModule.h"
#include "SimulationUtility.h"
#include "ActionLog.h"
#include <map>
#include <queue>
#include <utility>
using namespace std;

///DO NOT MODIFY-------------------------------------------//
//Possible Wheelbot actions
enum WheelbotActions {
    ACTION_FORWARD = 0,
    ACTION_BACKWARD,
    ACTION_LEFT,
    ACTION_RIGHT,
    ACTION_STOP
};

//Class for representing a graph - in this case a map
class Graph {
public:
    //vector of map nodes (3d points) - indices of nodes in this vector are the node "names"
    vector<PxVec3> graphPoints;
    //vector of map edges (pairs of node indices) - nodes represented as their indices
    vector<pair<int, int> > graphEdges;
    Graph(vector<PxVec3> p, vector<pair<int, int> >  e) {
        this->graphPoints = p;
        this->graphEdges = e;
    }
    Graph(){}
};
///END DO NOT MODIFY-------------------------------------------//

class Project2_Control:rm3d::ai::ModuleProgram<rm3d::ai::WheelbotDock *> {
private:
    WheelbotSimBase* simulator;
    rm3d::ai::WheelbotModel *model;
    //Radius around goal, indicating when we have arrived at a map node
    static PxReal currentGoalRadius;
    //Magnitude of wheel velocities for movement
    PxReal velocityMagnitude;
    //start map node index
    static int start;
    //goal map node index
    static int goal;
    //Flag for determining whether or not a plan has been made already and is still being executed
    bool havePlanned;
    //Current step in plan being executed
    int currentPlanStep;
    //Map of the cities
    static Graph g;
    //Current position of Wheelbot - used for rendering
    static PxVec3 currentWheelbotPosition;
    //Position that is the "current goal" (i.e., the current node Wheelbot is going toward).
    //It will not correspond to the final goal until we make it to a node adjacent to the goal.
    static PxVec3 currentGoalPosition;
    //A plan of points (cities) to visit
    static vector<PxVec3> plan;
    //Should render path planned - useful for debugging
    static bool showPath;
public:
    //Helpful rendering functionality to show map and current path
    ///DO NOT MODIFY-------------------------------------------//
    static void render() {
        for (int i=0; i<g.graphPoints.size(); i++) {
            if ((currentWheelbotPosition - g.graphPoints[i]).magnitude() <= currentGoalRadius + 0.05) {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,1,1));
            } else if (i == goal) {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,1,0));
            } else if (i == start) {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(1,1,0));
            } else {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,0,1));
            }
            
        }
        for (int j=0; j<g.graphEdges.size(); j++) {
            rm3d::Renderer::DrawLine(PxVec3(g.graphPoints[g.graphEdges[j].first].x,
                                            g.graphPoints[g.graphEdges[j].first].y + 0.05,
                                            g.graphPoints[g.graphEdges[j].first].z),
                                     PxVec3(g.graphPoints[g.graphEdges[j].second].x,
                                            g.graphPoints[g.graphEdges[j].second].y + 0.05,
                                            g.graphPoints[g.graphEdges[j].second].z), rm3d::Color(0,1,1));
        }
        
        
        if (showPath) {
            if (plan.size() > 0) {
                for (int i=0; i<plan.size() -1; i++) {
                    rm3d::Renderer::DrawLine(PxVec3(plan[i].x, plan[i].y + .1, plan[i].z),
                                             PxVec3(plan[i + 1].x,
                                             plan[i + 1].y + .1, plan[i + 1].z), rm3d::Color(0,1,0));
                }
            }
        }
        
    }
    ///END DO NOT MODIFY-------------------------------------------//
    
    Project2_Control(WheelbotSimBase* simulator, Graph graph, int startNode, int goalNode) {
        this->programCounter = 0;
        this->actionQueue = new rm3d::ai::ActionLog*[4];
        this->simulator = simulator;
        this->actionQueue[0] = NULL;
        this->actionQueue[1] = NULL;
        this->actionQueue[2] = NULL;
        this->actionQueue[3] = NULL;
        this->havePlanned = false;
        this->velocityMagnitude = 15.0;
        start = startNode;
        goal = goalNode;
        g = graph;
    }
    
    ///DO NOT MODIFY-------------------------------------------//
    ~Project2_Control() {
        if (this->actionQueue[0] != NULL) delete this->actionQueue[0];
        if (this->actionQueue[1] != NULL) delete this->actionQueue[1];
        if (this->actionQueue[2] != NULL) delete this->actionQueue[2];
        if (this->actionQueue[3] != NULL) delete this->actionQueue[3];
        delete actionQueue;
    }
    
    rm3d::ai::ActionLog **getActionQueue() {
        return this->actionQueue;
    }
    rm3d::ai::Message<rm3d::ai::WheelbotDock*> **getMessageQueue() {
        return this->messageQueue;
    }
    rm3d::ai::RangedMessage** getRangedMessageQueue() {
        return this->rangedMessageQueue;
    }
    int getNumberOfRangedMessages() {
        return this->numRangedMessages;
    }
    int getNumberOfActions() {
        return this->numActions;
    }
    int getNumberOfMessages() {
        return this->numMessages;
    }
    
    PxTransform getCurrentWheelbotPose() {
        return boost::any_cast<PxTransform>(this->model->getCurrentState()->sensorValues[rm3d::ai::WheelbotModule::posOrientInt]);
    }
    
    PxVec3 getGoalPosition() {
        return currentGoalPosition;
    }
    
    PxRaycastHit getRangeResult() {
        PxRaycastHit hit = boost::any_cast<PxRaycastHit>(this->model->getCurrentState()->sensorValues[rm3d::ai::WheelbotModule::xAxisRaycastInt]);
        return hit;
    }
    
    void takeAction(WheelbotActions action) {
        this->numActions = 4;
        if (action == ACTION_BACKWARD) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)velocityMagnitude);
        } else if (action == ACTION_FORWARD) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)-velocityMagnitude);
        } else if (action == ACTION_LEFT) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)-velocityMagnitude);
        } else if (action == ACTION_RIGHT) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)velocityMagnitude);
        } else if (action == ACTION_STOP) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)0);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)0);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)0);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)0);
        }
    }
    ///END DO NOT MODIFY-------------------------------------------//
    
    void step(void *model) {
        ///DO NOT MODIFY------------------///
        simulator->setCustomRenderFunction(render);
        this->model = (rm3d::ai::WheelbotModel *)model;
        currentWheelbotPosition = getCurrentWheelbotPose().p;
        if (this->actionQueue[0] != NULL) delete this->actionQueue[0];
        if (this->actionQueue[1] != NULL) delete this->actionQueue[1];
        if (this->actionQueue[2] != NULL) delete this->actionQueue[2];
        if (this->actionQueue[3] != NULL) delete this->actionQueue[3];
        this->numActions = 4;
        ///END DO NOT MODIFY------------------///
        
        ///TODO:
        ////1. Plan a least cost path to the goal using the actual distances between nodes (cities) and the
        ////straight-line distance heuristic.
        ////2. Execute the plan until you detect a road block (red obstacle) with your range sensor.
        ////3. If you detect an obstacle, update the map using this new knowledge and return to the previous
        ////city (the one that you just left).
        ////4. Update the map (this->g) with the knowledge that this link is no longer traversible or has a different
        ////cost than was originally thought. NOTE: obstacles will never disappear. All obstacles stay exactly where they
        ////are throughout the course of the project run. They will, in general, change between runs, though.
        ////5. Replan from the city you returned to based on the updated map (using A*)
        ////6. Continue doing the above steps until you encounter one of two conditions:
        ////   a. You reach the final goal city. In this case, you should stop and declare "Success"
        ////   b. You have exhausted all possible paths to the goal city and all are blocked. In this case, you
        ////   should stop and declare "Failure".
        
        //Check to see if I have planned yet. I don't want to plan each time step is called, only when I
        //am done executing my plan or when an obstacle forces me to replan.
        if (!havePlanned) {
            //Make sure currentPlanStep is reset
            currentPlanStep = 0;
            //Set the flag to true, so we know we have planned and are going to start executing
            havePlanned  = true;
            //Push the start node onto the plan
            plan.push_back(g.graphPoints[start]);
            //Determine the actions available from the start node
            vector<pair<int, int> > actions;
            //Loop through each edge
            for (int i=0; i<g.graphEdges.size(); i++) {
                //If the first node is the start node, this
                //edge is an "action" we can take from the start node, and
                //we save it.
                if (g.graphEdges[i].first == start) {
                    actions.push_back(g.graphEdges[i]);
                }
            }
            //Choose a random action out of the set of available ones
            int nextStep = rand() % actions.size();
            currentPlanStep++;
            //Add the map node resulting from the selected action to the plan.
            plan.push_back(g.graphPoints[actions[nextStep].second]);
            //Set the chosen action's node as the current goal position to traverse the edge
            currentGoalPosition = plan[currentPlanStep];
        }
        
        
        //Check our raycast sensor
        PxRaycastHit hit = getRangeResult();
        //If we see something less than 0.28 meters in front of us, we stop and return
        if (hit.distance < 0.28 && hit.distance > 0.01) {
            takeAction(ACTION_STOP);
            //NOTE: the return is simply so we don't run into the obstacle. You should remove it when
            //you code your solution because you will have to return to the most recent city you were at.
            return;
        }
        
        //The following code is almost exactly taken from the solution to Project 1
        
        //2D position vector of Wheelbot (in global frame)
        PxVec2 pos2D = PxVec2(getCurrentWheelbotPose().p.x, getCurrentWheelbotPose().p.z);
        //3D position vector of point 0.3 units along Wheelbot's x-axis (in global frame)
        PxVec3 posAlongX = (getCurrentWheelbotPose()*PxTransform(PxVec3(0.3,0,0))).p;
        //2D position vector of point 0.3 units along Wheelbot's x-axis (in global frame)
        PxVec2 posAlongX2D = PxVec2(posAlongX.x, posAlongX.z);
        //2D position vector of goal (in global frame)
        PxVec2 goal2D = PxVec2(getGoalPosition().x, getGoalPosition().z);
        //2D Vector of length 0.3 along Wheelbot's x-axis starting at Wheelbot's position
        PxVec2 A = posAlongX2D - pos2D;
        //2D Vector from Wheelbot's position to the goal
        PxVec2 B = goal2D - pos2D;
        
        //The following code extracts the angle between the above vectors using atan2
        A.normalize();
        B.normalize();
        
        float thetaA = atan2(A.x, A.y);
        float thetaB = atan2(B.x, B.y);
        
        float thetaAB = thetaB - thetaA;
        
        while (thetaAB <= - PxPi)
            thetaAB += 2 * PxPi;
        
        while (thetaAB > PxPi)
            thetaAB -= 2 * PxPi;
        
        //Here is the result: the angular error between Wheelbot's heading and the goal direction
        PxReal angleError = thetaAB;
        
        //If the angular error is too large
        if (fabs(angleError) > 0.05) {
            //turn right if it is less than zero, otherwise turn left
            if (angleError < 0) {
                takeAction(ACTION_RIGHT);
            } else {
                takeAction(ACTION_LEFT);
            }
        //If we are close enough to the goal
        } else if ((goal2D - pos2D).magnitude() <= currentGoalRadius) {
            //stop
            takeAction(ACTION_STOP);
            //Increment the plan step
            currentPlanStep++;
            //If the plan step is less than the plan size
            if (currentPlanStep < plan.size()) {
                //set the currentGoalPosition to the next node (city) on the path
                currentGoalPosition = plan[currentPlanStep];
            }
        //Otherwise, we need to keep going forward
        } else {
            takeAction(ACTION_FORWARD);
        }
        this->programCounter++;
    }
};

//Static variable definitions

PxVec3 Project2_Control::currentWheelbotPosition;
PxVec3 Project2_Control::currentGoalPosition;
vector<PxVec3> Project2_Control::plan;
//DO NOT MODIFY ----------------------------------//
PxReal Project2_Control::currentGoalRadius = 0.1;
//END DO NOT MODIFY ------------------------------//
bool Project2_Control::showPath = true;
Graph Project2_Control::g;
int Project2_Control::goal;
int Project2_Control::start;
#endif
