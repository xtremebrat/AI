/*
 *  This file is part of ReMod3D
 *  Copyright (C) 2013
 *     Thomas Collins, Nadeesha Ranasinghe, Wei-Min Shen
 */

/*
 * Simulator.h
 * Base class from which simulators for modules can be derived
 */

/**
 * @class Simulator
 *
 * @brief Base class for PhysX-based simulations of modules
 *
 * @author Thomas Collins
 *
 * @version 1.0
 *
 * @date 01/01/2013
 *
 * Contact: collinst@usc.edu
 *
 * Created on: 01/01/2013
 *
 */
#ifndef SuperbotSimulator_Simulator_h
#define SuperbotSimulator_Simulator_h
#include <vector>
#include "Global.h"
#include "Module.h"
#include "Environment.h"
#include "Obstacle.h"
#include "RangedMessage.h"
#include "Raycast.h"
#include "Cloth.h"
#include <math.h>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace physx;
#define MAX_PATH 256
namespace rm3d {
    namespace ai {
        class Obstacle;
    }
}

namespace rm3d {
    namespace ai {
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        class Simulator {
        protected:
            /**
             *	@brief	Function pointer for user initialization function
             */
            static void (*init_function_sim)();
            /**
             *	@brief	Function pointer for user step function
             */
            static void (*step_function_sim)();
            /**
             *	@brief	Function pointer for user shutdown function
             */
            static void (*shutdown_function_sim)();
            /**
             *	@brief	Function pointer for user render function
             */
            static void (*custom_render_function_sim)();
            /**
             *	@brief	vector of function pointers for user simulation step functions
             */
            static vector<void (*)()> sim_step_functions;
            /**
             *	@brief	Simulation camera
             */
            static rm3d::Camera camera;
            /**
             *	@brief	Color of the ground plane
             */
            static rm3d::Color groundColor;

            /**
             *	@brief	Color of the sky
             */
            static rm3d::Color skyColor;
            /**
             *	@brief	accumulates simulated time until it reaches the set level
             */
            static double accumulator;

            /**
             *	@brief	dictates how much the camera translates/rotates in response to keyboard presses/mouse moves
             */
            static float motion_scale_factor;
            /**
             *	@brief	Pointer to PhsyX physics sdk object
             */
             static PxPhysics* physics;
            /**
             *	@brief	Pointer to PhysX foundation object
             */
             static PxFoundation* foundation;
            /**
             *	@brief	Pointer to PhysX cooking object
             */
             static PxCooking* cook;
            /**
             *	@brief	Vector setting simulation gravity
             */
             static PxVec3 gravity;
            /**
             *	@brief	Pointer to PhysX scene object
             */
             static PxScene* scene;
            /**
             *	@brief	Simulation ground plane
             */
             static PxRigidStatic* plane;
            /**
             *	@brief	Material used in simulation
             */
             static PxMaterial* planeMaterial;
            /**
             *	@brief	Default error callback
             */
             static PxDefaultErrorCallback gDefaultErrorCallback;
            /**
             *	@brief	Default allocator callback
             */
             static PxDefaultAllocator gDefaultAllocatorCallback;
            /**
             *	@brief	Default filter shader
             */
             static PxSimulationFilterShader gDefaultFilterShader;
            /**
             *	@brief	Mapping from names of rigid bodies to colors to use in simulation
             */
            static map<string, rm3d::Color> colors;
            /**
             *	@brief	mapping from module nums to indices in the modules vector
             */
            static  map<string, int> namesToNums;
            /**
             *	@brief	Vector of modules involved in the simulation.
             */
            static vector<Module<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint> *> modules;
            /**
             *	@brief	A vector of frames to render at each timestep
             */
            static vector<PxTransform> framesToRenderForBodies;
            /**
             *	@brief	A vector of Raycasts to render at each timestep
             */
            static vector<rm3d::ai::Raycast> raycastsToRender;
            /**
             *	@brief	Vector of strings to render on each OpenGL frame
             */
             static vector<const char *> custom_buffers;
            /**
             *	@brief	x-coordinates on OpenGL frame of string in custom_buffers at the corresponding index
             */
            static  vector<int> custom_x;
            /**
             *	@brief	y-coordinates on OpenGL frame of string in custom_buffers at the corresponding index
             */
             static vector<int> custom_y;
            /**
             *	@brief	mapping between keyboard keys and pointers to functions to be invoked by that key
             */
            static  map<char, void(*)()> keys_to_functions;
            /**
             *	@brief	Obstacles in the environment;
             */
            static vector<rm3d::ai::Obstacle> obstacles;
            /**
             *	@brief	Mapping between obstacle names and indices in obstacle vector
             */
            static map<string, int> namesToObstacles;
            /**
             *	@brief	Default simulation color
             */
            static  rm3d::Color default_color;
            /**
             * @brief local transform from module to camera
             */
            static PxTransform localTransformCamera;
            /**
             *	@brief	timestep of the simulation (i.e. the length of each simulation step)
             */
            static  PxReal simTimestep;
            /**
             *	@brief	Current frames/second of simulation
             */
             static float fps;
            /**
             *	@brief	start time of the simulation
             */
             static int startTime;
            /**
             *	@brief	Total number of OpenGL frames produced by the simulation
             */
             static int totalFrames;
            /**
             *	@brief	Scale of the simulation
             */
             static float scale;
            /**
             *	@brief	initial number of time steps to wait before simulated bodies begin executing actions
             */
            static  int initial_wait_count;
            /**
             *	@brief	Current module number being tracked by camera.
             */
            static int currentModuleNum;

            /**
             *	@brief flag indicating whether or not the program is paused
             */
            static  bool pause_program;
            /**
             *	@brief	buffer for rendering fps
             */
            static  char buffer[MAX_PATH];
            /**
             *	@brief	buffer for rendering fps
             */
            static  char *custom_buffer;
            /**
             *	@brief	which docks should the simulator monitor at each time step for docking and undocking
             */
            static  vector<Dock> docksToMonitor;
            /**
             *	@brief	Converts the indices of two modules and a dock of each one to a single string
             */
            static  string moduleNumsAndDocksToString(int module1_num, int module2_num, DockFace module1_dock, DockFace module2_dock);
            /**
             *	@brief	width of the OpenGL window
             */
            static  int windowHeight;
            /**
             *	@brief position error tolerance during docking. 
             *  This is how far about the two docks can be from one another before automatic magnetic 
             *  docking is initiated
             */
            static  double errorPositionDock;
            /**
             *	@brief	queue of docks to delete from the docksToMonitor vector
             */
            static  vector<pair<string, DockFace> > deleteQueue;
            /**
             * @brief queue of docks to add to the docksToMonitor vector
             */
            static vector< Dock > addQueue;
            /**
             *	@brief	mapping between a module's name and the dock object associated with
             *   each of the module's docks
             */
            static  map<string, map<DockFace, Dock> > connections;
            /**
             *	@brief	mapping between a module's name and its current queued messages
             */
            static  map<string, vector<Message<Dock>*> > messageRoom;
            /**
             *	@brief	mapping between a module's name and its current queued ranged messages
             */
            static  map<string, vector<rm3d::ai::RangedMessage *> > messageRoomRanged;
            /**
             *	@brief	orientation tolerance in the docking process. This is the magnitude of orientation difference
             *   allowed between the docks that are to be joined with a joint
             */
            static  double errorOrientationDock;
            /**
             *	@brief	height of the OpenGL window
             */
            static int windowWidth;
            /**
             *	@brief	Number of the current frame
             */
            static uint64_t currentFrameNumber;
            /**
             *	@brief	Whether or not to save the frames of the simulation in an OpenCV Matrix
             */
            static bool saveFrames;
            /**
             *	@brief	Frame image
             */
            static cv::Mat image;
            /**
             *	@brief	Flipped frame image
             */
            static cv::Mat flipped;
            /**
             *	@brief	Near plane (Z Plane)
             */
            static float zNearPlane;
            /**
             *	@brief	far Z plane (OpenGL)
             */
            static float zFarPlane;
            /**
             *	@brief	Should lighting be disabled
             */
            static bool disableLigthing;
            /**
             *	@brief	Particle fluids in the simulation
             */
            static map<string, PxParticleFluid *> particleFluids;

            /**
             *	@brief	Particle systems in the simulation
             */
            static map<string, PxParticleSystem *> particleSystems;
            /**
             * @brief  Cloths in simulation
             */
            static map<string, Cloth *> cloths;
            /**
             * @brief should show fps
             */
            static bool showFPS;
            static vector<PxHeightFieldSample> samples;
            static PxHeightFieldDesc hfDesc;
            static int numRows;
            static int numCols;
            static float heightScale;
            static float rowScale;
            static float colScale;
            static bool useHeightField;
            static bool canRaycastPlane;
        public:
            /**
             *	@brief	Creates a particle system with the given properties
             *
             *	@param 	maxParticles 	maximum number of particles the system can handle
             *	@param 	gridSize 	grid size
             *	@param 	maxMotiondistance 	max motion distance of particles
             *	@param 	restOffset 	rest offset
             *	@param 	contactOffset 	contact offset
             *	@param 	damping 	damping of fluid particles
             *	@param 	restitution 	restitution of fluid
             *	@param 	dynamicFriction 	fluid dynamic friction
             */
            static void createParticleSystem(int maxParticles, const char* name, const rm3d::Color& particleColor,
                                             float gridSize = 5.0f, float maxMotionDistance = 0.3f,
                                             float restOffset = 0.08f*0.3f, float contactOffset = 0.08f*0.3f*2, float damping = 0.0f,
                                             float restitution = 0.3f, float dynamicFriction = 0.001f, float mass = 5.0f) {
                PxParticleSystem* ps = physics->createParticleSystem(maxParticles);
                ps->setGridSize(gridSize);
                ps->setMaxMotionDistance(maxMotionDistance);
                ps->setRestOffset(restOffset);
                ps->setContactOffset(contactOffset);
                ps->setDamping(damping);
                ps->setRestitution(restitution);
                ps->setDynamicFriction(dynamicFriction);
                ps->setParticleReadDataFlag(PxParticleReadDataFlag::eVELOCITY_BUFFER, true);
                ps->setName(name);
                //ps->setParticleMass(mass);
                ps->setParticleBaseFlag(PxParticleBaseFlag::eCOLLISION_WITH_DYNAMIC_ACTORS, true);
                ps->setParticleBaseFlag(PxParticleBaseFlag::eCOLLISION_TWOWAY, true);
                addColorToColorsDictionary(particleColor, name);
			   #if PX_SUPPORT_GPU_PHYSX
				cout<<"Using GPU Particles"<<endl;
                ps->setParticleBaseFlag(PxParticleBaseFlag::eGPU, true);
                #endif
				scene->addActor(*ps);
                particleSystems[name] = ps;
            }
            /**
             *	@brief	Creates a particle fluid with the given properties
             *
             *	@param 	maxParticles 	maximum number of particles the system can handle
             *	@param 	gridSize 	grid size
             *	@param 	maxMotiondistance 	max motion distance of particles
             *	@param 	restOffset 	rest offset
             *	@param 	contactOffset 	contact offset
             *	@param 	damping 	damping of fluid particles
             *	@param 	restitution 	restitution of fluid
             *	@param 	dynamicFriction 	fluid dynamic friction
             *	@param 	restPartDistance 	particle rest distance
             *	@param 	viscosity 	viscosity of fluid
             *	@param 	stiffness 	stiffness of fluid
             */
            static void createParticleFluidSystem(int maxParticles, const char* name, const rm3d::Color& particleColor,
                                            float gridSize = 5.0f, float maxMotionDistance = 0.3f,
                                            float restOffset = 0.08f*0.3f, float contactOffset = 0.08f*0.3f*2, float damping = 0.0f,
                                            float restitution = 0.3f, float dynamicFriction = 0.001f, float restPartDistance = 0.08f,
                                            float viscosity = 60.0f, float stiffness = 45.0f, float particleMass = 0.001f) {
                PxParticleFluid *pf = physics->createParticleFluid(maxParticles);
                pf->setGridSize(gridSize);
                pf->setMaxMotionDistance(maxMotionDistance);
                pf->setRestOffset(restOffset);
                pf->setContactOffset(contactOffset);
                pf->setDamping(damping);
                pf->setRestitution(restitution);
                pf->setDynamicFriction(dynamicFriction);
                pf->setRestParticleDistance(restPartDistance);
                pf->setViscosity(viscosity);
                pf->setStiffness(stiffness);
                pf->setParticleMass(particleMass);
                pf->setName(name);
                pf->setParticleBaseFlag(PxParticleBaseFlag::eCOLLISION_WITH_DYNAMIC_ACTORS, true);
               pf->setParticleBaseFlag(PxParticleBaseFlag::eCOLLISION_TWOWAY, true);
                addColorToColorsDictionary(particleColor, name);
                //pf->setParticleMass(5.0);
                pf->setParticleReadDataFlag(PxParticleReadDataFlag::eVELOCITY_BUFFER, true);
                #if PX_SUPPORT_GPU_PHYSX
				cout<<"Using GPU Fluid"<<endl;
                pf->setParticleBaseFlag(PxParticleBaseFlag::eGPU, true);
                #endif
                scene->addActor(*pf);
                particleFluids[name] = pf;
            }
            
            static void createCloth(string name, const PxTransform& tr, float radius, float width, float height, bool topLeft, bool topRight, bool bottomLeft, bool bottomRight) {
                vector<PxTransform> transforms(modules.size());
                for (int i=0; i<modules.size(); i++) {
                    transforms[i] = modules[i]->getPositionOrientation();
                }
                cloths[name] = new Cloth(name, physics, scene, cook, tr, transforms,radius, width, height, topLeft, topRight, bottomLeft, bottomRight);
            }
            
            static int getWindowWidth() {
                return windowWidth;
            }
            
            static int getWindowHeight() {
                return windowHeight;
            }
            
            /**
             *	@brief	creates fluid particles
             *
             *	@param 	name 	name of particle fluid system
             *	@param 	numParticles 	number particles to create
             *	@param 	creationData 	creation data for particles
             */
            static void createFluidParticles(string name, int numParticles, PxParticleCreationData creationData) {

                PxParticleFluid *pf = particleFluids[name];
                pf->createParticles(creationData);
            }
            
            /**
             *	@brief	creates particles
             *
             *	@param 	name 	name of particle system
             *	@param 	numParticles 	number particles to create
             *	@param 	creationData 	creation data for particles
             */
            static void createParticles(string name, int numParticles, PxParticleCreationData creationData) {

                PxParticleSystem *ps = particleSystems[name];
                ps->createParticles(creationData);
            }

            /**
             *	@brief	Returns the number of rows in the image frame (camera on module)
             *
             *	@return	integer representing number of rows in image frame
             */
            static int getFrameRowCount() {
                return glutGet(GLUT_WINDOW_HEIGHT);
            }
            /**
             *	@brief	Returns the number of cols in the image frame (camera on module)
             *
             *	@return	integer representing number of cols in image frame
             */
            static int getFrameColumnCount() {
                return glutGet(GLUT_WINDOW_WIDTH);
            }
            /**
             *	@brief	search for docks that are close enough (within tolerances) and create a joint between them
             */
            static void engageDocks() {};
            
            /**
             *	@brief	remove joints between modules when one or both docks are disabled
             */
            static void disengageDocks() {};
            
            /**
             *	@brief	Getter for docksToMonitor variable
             *
             *	@return	docks for the simulation to monitor. Only enabled, !engaged docks need be monitored
             */
            static vector<Dock> getDocksToMonitor(){
                return docksToMonitor;
            }
            
            static vector<Dock> getAddQueue() {
                return addQueue;
            }
            
            /**
             *	@brief	setter for init_function_sim
             *
             *	@param 	init 	user initialization function
             */
            static void setInitFunction(void(*init)()) {

                init_function_sim = init;
            }
            
            /**
             *	@brief	setter for step_function_sim
             *
             *	@param 	step 	user step function
             */
            static void setStepFunction(void(*step)()) {

                step_function_sim = step;
            }
            
            /**
             *	@brief	Tells the simulator to render the given frame on the next frame
             *
             *	@param 	t 	Transform to render
             */
            static void addFrameToRender(const PxTransform& t) {

                framesToRenderForBodies.push_back(t);
            }
            
            /**
             *	@brief	Tells the simulator to render the frame of the given actor on next frame
             *
             *	@param 	r raycast to render
             */
            static void addRaycastToRender(const Raycast& r) {
                
                raycastsToRender.push_back(r);
            }
            
            /**
             *	@brief	setter for shutdown_function_sim
             *
             *	@param 	shutdown 	user shutdown function
             */
            static void setShutdownFunction(void(*shutdown)()) {
                shutdown_function_sim = shutdown;
            }
            
            /**
             *	@brief	setter for custom_render_function_sim
             *
             *	@param 	custom_render 	user render function
             */
            static void setCustomRenderFunction(void(*custom_render)()) {
                custom_render_function_sim = custom_render;
            }

            /**
             *	@brief	rotate the mouse on left mouse button hold
             *
             *	@param 	x 	x-coordinate of mouse
             *	@param 	y 	y-coordinate of mouse
             */
            static void MotionRotate(int x, int y) {

                camera.mouseRotate(x, y);
            }
            
            /**
             *	@brief	translate the mouse up and down on right mouse button hold
             *
             *	@param 	x 	x-coordinate of the mouse
             *	@param 	y 	y-coordinate of the mosue
             */
            static void MotionUpDown(int x,int y) {

                camera.mouseTranslate(x, y);
            }
            
            /**
             *	@brief	Sets the module to follow the given module
             *
             *	@param 	followModule module to follow
             */
            static void followModuleWithCamera() {
                camera.setCameraObjectMode(true);
                if (currentModuleNum + 1 >= (int)modules.size()) {
                    currentModuleNum = -1;
                    camera.setCameraObjectMode(false);
                } else {
                    currentModuleNum += 1;
                }
            }
            
            /**
             *	@brief	getter for initial_wait_count variable
             *
             *	@return	returns initial number of simulation steps before modules begin to move
             */
            static int getInitialWaitCount() {

                return initial_wait_count;
            }
            
            /**
             *	@brief	decrement initial_wait_count by one
             */
            static void decrementInitialWaitCount() {

                initial_wait_count--;
            }
            
            /**
             *	@brief	getter for pause_program which indicates whether or not the simulator is paused
             *
             *	@return	returns true if the simulation is paused; false otherwise
             */
            static bool getProgramPaused() {
                
                return pause_program;
            }

            /**
             *	@brief	setter for pause_program which pauses the simulation
             *
             *	@param 	paused 	if true, pauses the simulation; if false, unpauses the simulation
             */
            static void setProgramPaused(bool paused) {
                
                pause_program = paused;
            }
            
            /**
             *	@brief	getter for simTimestep
             *
             *	@return	returns the current simulation timestep (inverse of number of simulation steps/second)
             */
            static PxReal getSimulationTimestep() {
                
                return simTimestep;
            }
            
            /**
             *	@brief	setter for simTimestep
             *
             *	@param 	timestep 	timestep for simulation (inverse of number of simulation steps per second)
             */
            static void setSimulationTimestep(PxReal timestep) {
                simTimestep = timestep;
            }
            
           /**
            *	@brief	create a mapping between a keyboard key and function pointer such that
            *   the function is invoked when the user presses the key
            *   @param  key   keyboard key to invoke function
            *	@param 	func 	function pointer to invoke on key press
            */
           static  void addKeyboardKeyMapping(char key, void(* func)()) {

                keys_to_functions[key] = func;
            }
            
           /**
            *	@brief	Main simulation step function, called at each time step. Pushes simulation forward in time
            */
           static void StepPhysX() {
               //If the program is not paused, we simulate another time step worth of time, decrement the wait count (if there is one) and fetch simulation results
               if (!pause_program) {
                if (!initial_wait_count) {
                    for(int i=0; i<(int)modules.size(); i++) {
                        modules[i]->step();
                    }
                    
                    for (int i=0; i<(int)sim_step_functions.size(); i++) {
                        sim_step_functions[i]();
                    }
                    updateDockLists();
                }
                
                    scene->simulate(simTimestep);
                    
                    if (initial_wait_count > 0) initial_wait_count--;
                    //Note that we must block here to wait for the simulation data to be updated. Do not change this without good cause.
                    while(!scene->fetchResults(true)){}
                }
            }
            
            /**
             *	@brief	Tears down the simulation and cleans up dynamic memory
             */
            static void ShutdownPhysX() {

                //Release dynamic memory allocated by this class and by the PhysX simulator itself
                if (shutdown_function_sim) {
                    shutdown_function_sim();
                }
            }
            
            /**
             *	@brief	Class destructor
             *
             */
            ~Simulator() {

                for (int i=0; i<(int)modules.size(); i++) {
                    delete modules[i];
                }
                cook->release();
                scene->release();
                physics->release();
                foundation->release();
            }
            
            /**
             *	@brief	Initializes OpenGL values for simulation rendering
             */
            static void InitGL() {

                //Initialize OpenGL values. Standard values have been used. Only modify if you have a good understanding of OpenGL.
                glEnable(GL_DEPTH_TEST);
                glDepthFunc(GL_NEAREST);
                glEnable(GL_CULL_FACE);
                glEnable(GL_LIGHTING);
                glEnable(GL_LIGHT0);
                
                GLfloat ambient[4]={0.25f,0.25f,0.25f,0.25f};
                GLfloat diffuse[4]={1,1,1,1};
                GLfloat mat_diffuse[4]={0.5f,0.5,0.5,0.5};
                
                glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
                glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
                glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diffuse);
                
                glDisable(GL_LIGHTING);
            }
            
           /**
            *	@brief	Reshapes the OpenGL window at each time step
            *
            *	@param 	nw 	window width (pixels)
            *	@param 	nh 	window height (pixels)
            */
           static  void OnReshape(int nw, int nh) {

                //Reshape the window. Again, standard OpenGL code that should only be changed by someone with good OpenGL knowledge.
                glViewport (0 ,0 ,nw, nh ) ;
                glMatrixMode(GL_PROJECTION);
                glLoadIdentity();
                gluPerspective (60.0, (float)glutGet(GLUT_WINDOW_WIDTH)/(float)glutGet(GLUT_WINDOW_HEIGHT), zNearPlane, zFarPlane);
            }
            
            /**
             *	@brief	Renders the OpenGL frame at each timestep
             */
            static void OnRender() {

                //Calculate fps
                totalFrames++;
                currentFrameNumber++;
                int current = glutGet(GLUT_ELAPSED_TIME);
                if((current-startTime)>1000)
                {
                    float elapsedTime = float(current-startTime);
                    fps = ((totalFrames * 1000.0f)/ elapsedTime) ;
                    startTime = current;
                    totalFrames=0;
                }
                
                if (showFPS) sprintf(buffer, "FPS: %3.2f",fps);
                //Update PhysX by calling our step function each time we render.
                if (scene)
                {
                    if (step_function_sim) {
                        //Completely manual step function specified externally
                        step_function_sim();
                    } else {
                        //Default step function
                        StepPhysX();
                    }
                }
                
                
                //Clear some OpenGL buffer bits for next render
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                
                // Setup camera. We start with a camera at (0,0,0) with no pan or tilt and translate/rotate it according to our camera values.
                //This is how we achieve the effect that we are in complete control of the camera.
                glMatrixMode(GL_PROJECTION);
                glLoadIdentity();
                gluPerspective(60.0, (float)glutGet(GLUT_WINDOW_WIDTH)/(float)glutGet(GLUT_WINDOW_HEIGHT), zNearPlane, zFarPlane);
                glMatrixMode(GL_MODELVIEW);
                glLoadIdentity();
                float xrot, yrot;
                if (camera.getCameraObjectMode()) {
                    Renderer::LookAtPosOr(modules[currentModuleNum]->getPositionOrientation()*localTransformCamera);
                } else {
                    camera.getOrientations(&xrot, &yrot);
                    glRotatef(yrot,1.0,0.0,0.0);  //rotate our camera on teh
                    glRotatef(xrot,0.0,1.0,0.0);  //rotate our camera on the
                    PxVec3 trans = camera.getTranslationVector();
                    glTranslated(trans.x,trans.y,trans.z); //translate the screen
                }
                //Draw the grid on the plane
                //Renderer::DrawGrid(10);
                if (!disableLigthing) {
                    glEnable(GL_LIGHTING);
                }
                //Draw all the actors in the scene (including articulations)
                rm3d::Renderer::DrawActors(scene, colors, default_color);
                
                
                
                
                //update collider position based on the new position of box;
                for (map<string, Cloth*>::iterator it = cloths.begin(); it!=cloths.end(); it++) {
                    Cloth *c = it->second;
                    for (int i=0; i<(int)c->module_colliders.size(); i++) {
                        c->module_colliders[i].pos = modules[i]->getPositionOrientation().p;
                    }
                    c->cloth->setCollisionSpheres(&c->module_colliders[0]);
                    
                    //update the cloth data
                    PxClothReadData* pData = c->cloth->lockClothReadData();
                    PxClothParticle* pParticles = const_cast<PxClothParticle*>(pData->particles);
                    
                    //update the positions
                    for(size_t i=0;i<c->pos.size();i++) {
                        c->pos[i] = pParticles[i].pos;
                    }
                    pData->unlock();
                    
                    //update normals
                    for(size_t i=0;i<c->indices.size();i+=3) {
                        PxVec3 p1 = c->pos[c->indices[i]];
                        PxVec3 p2 = c->pos[c->indices[i+1]];
                        PxVec3 p3 = c->pos[c->indices[i+2]];
                        PxVec3 n  = (p2-p1).cross(p3-p1);
                        
                        c->normal[c->indices[i]]    += n/3.0f ;
                        c->normal[c->indices[i+1]]  += n/3.0f ;
                        c->normal[c->indices[i+2]]  += n/3.0f ;
                    }
                    
                    for(size_t i=0;i<c->normal.size();i++) {
                        PxVec3& n  = c->normal[i];
                        n= n.getNormalized();
                    }
                    rm3d::Renderer::DrawCloth(c->cloth, c->pos, c->normal, c->indices, rm3d::Color(.8,.8,.8));
                    
                }
                
                
                
                
                
                if (custom_render_function_sim) {
                    custom_render_function_sim();
                }
                for (int i=0; i<(int)framesToRenderForBodies.size(); i++) {
                    rm3d::Renderer::DrawFrame(framesToRenderForBodies[i]);
                }
                for (int i=0; i<(int)raycastsToRender.size(); i++) {
                    PxVec3 point1 = raycastsToRender[i].source.p;
                    PxVec3 point2 = (raycastsToRender[i].source*PxTransform(raycastsToRender[i].direction*raycastsToRender[i].maxDistance)).p;
                    rm3d::Renderer::DrawLine(point1, point2, raycastsToRender[i].color);
                }
                framesToRenderForBodies.clear();
                raycastsToRender.clear();
                //Capture the current frame
                if (disableLigthing) {
                    glDisable(GL_LIGHTING);
                }
                rm3d::Renderer::SetOrthoForFont();
                glColor3f(1,1,1);
                //Show the fps
                rm3d::Renderer::RenderSpacedBitmapString(20,20,0,GLUT_BITMAP_HELVETICA_12,buffer);
                for (int i=0; i<(int)custom_buffers.size(); i++) {
                    rm3d::Renderer::RenderSpacedBitmapString(custom_x[i], custom_y[i], 0, GLUT_BITMAP_HELVETICA_12, custom_buffers[i]);
                }
                if (saveFrames) {
                    image = cv::Mat(windowHeight, windowWidth, CV_8UC3);
                    flipped = cv::Mat(windowHeight, windowWidth, CV_8UC3);
                    glPixelStorei(GL_PACK_ALIGNMENT, (flipped.step & 3) ? 1 : 4);
                    glPixelStorei(GL_PACK_ROW_LENGTH, flipped.step/flipped.elemSize());
                    glReadPixels(0, 0, flipped.cols, flipped.rows, GL_BGR_EXT, GL_UNSIGNED_BYTE, flipped.data);
                    cv::flip(flipped, image, 0);
                }
                
                rm3d::Renderer::ResetPerspectiveProjection();
                glutSwapBuffers();
            }

            
            /**
             *	@brief	Calls PhsyX shutdown function when OpenGL shuts down
             */
            static void OnShutdown() {

                ShutdownPhysX();
            }
            
            /**
             *	@brief	Called when the uses presses a mouse button
             *
             *	@param 	button 	the mouse button that causes the callback
             *	@param 	s 	not used
             *	@param 	x 	x-coordinate of the mouse
             *	@param 	y 	y-coordinate of the mouse
             */
            static void Mouse(int button, int s, int x, int y) {

                //When the left button is held down, mouse motion rotates the camera
                if (button == GLUT_LEFT_BUTTON) {
                    glutMotionFunc(MotionRotate);
                    //When the right button is held down, mouse motion is an up and down camera translation
                } else if (button == GLUT_RIGHT_BUTTON) {
                    glutMotionFunc(MotionUpDown);
                    //Scrolling up is zooming in
                } else if (button == 4) {
                    camera.cameraZoomIn();
                    //Scrolling down is zooming out
                } else if (button == 3) {
                    camera.cameraZoomOut();
                    return;
                }
                //Here we updated the last known position of the mouse (to avoid camera "jumps" in the motion function)
                camera.updateLastXLastY(x,y);
            }
            
            /**
             *	@brief	Makes use of OpenGL idle time
             */
            static void OnIdle() {

                glutPostRedisplay();
            }
            
            /**
             *	@brief	returns the obstacle associated with the given name
             *
             *	@param 	name 	name of the obstacle
             *
             *	@return	obstacle object associated with the given name
             */
            static rm3d::ai::Obstacle obstacleForName(string name) {
                return obstacles[namesToObstacles[name]];
            }
            
            /**
             *	@brief	Creates an obstacle with the given specifications and adds it the scene
             *
             *	@param 	initial 	initial position/orientation of the obstacle
             *	@param 	obstacleGeometry 	geometry of the obstacle
             *	@param 	obstacleMaterial 	material of the obstacle
             *	@param 	mass 	mass of the obstacle
             *	@param 	staticOb 	whether (true) or not (false) the obstacle is a static actor
             */
            static void createObstacle(const PxTransform& initial, const PxGeometry& obstacleGeometry, PxMaterial *obstacleMaterial, float mass, float density, const char* name, bool staticOb = true,
                                        bool kinematicObj = false, const Color& c = Color(1.0,0.0,0.0), const PxVec3& initialLin = PxVec3(0), const PxVec3& initialAng = PxVec3(0)) {
                rm3d::ai::Obstacle ob;
                if (staticOb) {
                    PxRigidStatic *rs = PxCreateStatic(*physics, initial, obstacleGeometry, *obstacleMaterial);
                    ob.obstacleBody = rs;
                    ob.obstacleBody->setName(name);
                } else if (!staticOb && kinematicObj) {
                    PxRigidDynamic *a = PxCreateDynamic(*physics, initial, obstacleGeometry, *obstacleMaterial, density);
                    ob.obstacleBody = a;
                    a->setLinearVelocity(initialLin);
                    a->setAngularVelocity(initialAng);
                    PxRigidBodyExt::setMassAndUpdateInertia(*a, mass);
                    ob.obstacleBody->setName(name);
                    a->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
                    /* ob.obstacleBody = PxCreateStatic(*physics, initial, obstacleGeometry, *obstacleMaterial);
                    ob.obstacleBody->setName(name.c_str());*/
                } else {
                    PxRigidDynamic *a = PxCreateDynamic(*physics, initial, obstacleGeometry, *obstacleMaterial, density);
                    ob.obstacleBody = a;
                    a->setLinearVelocity(initialLin);
                    a->setAngularVelocity(initialAng);
                    PxRigidBodyExt::setMassAndUpdateInertia(*a, mass);
                    ob.obstacleBody->setName(name);
                    
                }
                ob.mass = mass;
                ob.obstacleMaterial = obstacleMaterial;
                ob.obstacleGeometry = obstacleGeometry;
                ob.initialPose = initial;
                ob.staticObstacle = staticOb;
                ob.density = density;
                ob.name = name;
                ob.c = c;
                addColorToColorsDictionary(c, name);
                scene->addActor(*ob.obstacleBody);
                int index = obstacles.size();
                namesToObstacles[name] = index;
                obstacles.push_back(ob);
            }
            
            /**
             *	@brief	Returns poses of all obstacles
             *
             *	@return	position/orientation information for all obstacles in the simulation
             */
            static vector<PxTransform> getPosesOfObstacles() {

                vector<PxTransform> poses;
                for (int i =0; i<(int)obstacles.size(); i++) {
                    poses.push_back(obstacles[i].obstacleBody->getGlobalPose());
                }
                return poses;
            }
            
            /**
             *	@brief	Returns properties of all obstacles
             *
             *	@return	vector ob ObstacleProperties objects for each obstacle
             */
            static vector<ObstacleProperties> getPropertiesOfObstacles() {

                vector<ObstacleProperties> properties;
                for  (int i=0; i<(int)obstacles.size(); i++) {
                    ObstacleProperties obProp =  ObstacleProperties();
                    obProp.type = obstacles[i].obstacleBody->getType();
                    if (obProp.type == PxActorType::eRIGID_DYNAMIC) {
                        PxRigidDynamic *rd = (PxRigidDynamic *)obstacles[i].obstacleBody;
                        obProp.linearVelocity = rd->getLinearVelocity();
                        obProp.angularVelocity = rd->getAngularVelocity();
                        obProp.mass = obstacles[i].mass;
                        obProp.density = obstacles[i].density;
                    } else if (obProp.type == PxActorType::eRIGID_STATIC) {
                        obProp.linearVelocity = PxVec3(0);
                        obProp.angularVelocity = PxVec3(0);
                        obProp.mass = -1;
                        obProp.density = -1;
                    }
                    obProp.geometry = obstacles[i].obstacleGeometry;
                    obProp.color = obstacles[i].c;
                    obProp.pose = obstacles[i].obstacleBody->getGlobalPose();
                    obProp.name = obstacles[i].name;
                    PxBoxGeometry boxG = PxBoxGeometry();
                    properties.push_back(obProp);
                }
                return properties;
            }
            
            static vector<Obstacle> getObstacles() {
                return obstacles;
            }
            
            static void clearObstacles() {
                for (int i=0; i<obstacles.size(); i++) {
                    PxRigidActor* actor = obstacles[i].obstacleBody;
                    scene->removeActor(*actor);
                    obstacles[i].obstacleBody->release();
                }
                obstacles.clear();
            }
            
            /**
             *	@brief	Callback invoked when an arrow key is pressed
             *
             *	@param 	key 	the arrow key pressed
             *	@param 	x 	x-coordinate of the mouse
             *	@param 	y 	y-coordinate of the mouse
             */
            static void ArrowKeyCallback(int key, int x, int y) {

                if (key == GLUT_KEY_UP) {
                    //Up arrow is movement of the camera forward along the camera lens axis
                    camera.cameraRelativeForward();
                    return;
                } else if (key == GLUT_KEY_DOWN) {
                    //Down arrow is movement of the camera backward along the camera lens axis
                    camera.cameraRelativeBackward();
                    return;
                } else if (key == GLUT_KEY_RIGHT) {
                    //Right arrow is movement of the camera right along the camera tilt axis
                    camera.cameraRelativeRight();
                    return;
                } else if (key == GLUT_KEY_LEFT) {
                    //Left arrow is movement of camera left along the camera tilt axis
                    camera.cameraRelativeLeft();
                    return;
                }
            }
            
            /**
             *	@brief	Callback invoked when user presses a keyboard key
             *
             *	@param 	key 	keyboard key pressed
             *	@param 	x 	x-coordinate of mouse
             *	@param 	y 	y-coordinate of mouse
             */
            static void KeyboardCallback(unsigned char key, int x, int y) {

                switch (key)
                {
                    case '=': {
                        camera.cameraZoomIn();
                        return;
                    }
                    case '-':
                    {
                        camera.cameraZoomOut();
                        return;
                    }
                    case 'a': {
                        camera.cameraPanLeft();
                        return;
                    }
                    case 'd':
                    {
                        camera.cameraPanRight();
                        return;
                    }
                    case 's':
                    {
                        camera.cameraTiltDown();
                        return;
                    }
                    case 'w':
                    {
                        camera.cameraTiltUp();
                        return;
                    }
                    case '1':
                    {
                        camera.cameraAbsoluteUp();
                        return;
                    }
                    case '2':
                    {
                        camera.cameraAbsoluteDown();
                        return;
                    }
                    case '3':
                    {
                        camera.cameraAbsoluteLeft();
                        return;
                    }
                    case '4':
                    {
                        camera.cameraAbsoluteRight();
                        return;
                    }
                    case '5':
                    {
                        camera.cameraAbsoluteBackward();
                        return;
                    }
                    case '6':
                    {
                        camera.cameraAbsoluteForward();
                        return;
                    }
                    case 'p':
                    {
                        pause_program = !pause_program;
                        return;
                    }
                    case 'f':
                    {
                        followModuleWithCamera();
                        return;
                    }
                    case 'g':
                    {
                        //Toggle Gravity. HIGHLY EXPERIMENTAL AND DOES NOT WORK CONSISTENTLY. Use at your own risk.
                        if (scene->getGravity().y == 0) {
                            scene->setGravity(PxVec3(0,-9.8,0));
                        } else if (scene->getGravity().y < 0) {
                            scene->setGravity(PxVec3(0,0,0));
                        }
                        
                        return;
                    }
                    default:
                    {
                        if (keys_to_functions[key]) {
                            keys_to_functions[key]();
                            return;
                        }
                    }
                }
            }
            
            /**
             *	@brief	Sets up the simualated world: creates a ground plane, initializes data structures, etc
             */
            static void PreInitializePhysX() {

                srand(time(NULL));
                colors["Plane"] = rm3d::Color(0,0,0);
                cout<<"Pre-initialize the PhysX simulation"<<endl;
                foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
                physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale());
                cook = PxCreateCooking(PX_PHYSICS_VERSION, *foundation, PxCookingParams());
                
                //Initialize PhysX extensions (needed for Superbot modules)
                PxInitExtensions(*physics);
                //Create the scene
                PxSceneDesc sceneDesc(physics->getTolerancesScale());
                sceneDesc.gravity= gravity;
                
                //Set up GPU acceleration and shading
                if(!sceneDesc.cpuDispatcher) {
                    PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(4);
                    sceneDesc.cpuDispatcher = mCpuDispatcher;
                }
                if(!sceneDesc.filterShader)
                    sceneDesc.filterShader  = gDefaultFilterShader;
                physx::pxtask::CudaContextManagerDesc cudaDesc;
                physx::pxtask::CudaContextManager* m_pCudaContextManager = physx::pxtask::createCudaContextManager(*foundation, cudaDesc, physics->getProfileZoneManager());
                if(!sceneDesc.gpuDispatcher && m_pCudaContextManager != NULL)
                {
                    cout << "PhysX using Gpu - " << m_pCudaContextManager->getDeviceName() << ", " <<
                    m_pCudaContextManager->getMultiprocessorCount() << " cores @" << m_pCudaContextManager->getClockRate() << "\n";
                    sceneDesc.gpuDispatcher = m_pCudaContextManager->getGpuDispatcher();
                }
                
                //Create the scene
                scene = physics->createScene(sceneDesc);
                scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, true);
                //gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE,				 1.0);
                //gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES,	1.0f);
                //gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
                //gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);
                //gScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES, 1.0f);
                
                planeMaterial = physics->createMaterial(0.5,0.5,0.5);
                if (useHeightField) {
                    InitializeHeightfield();
                } else {
                    //Ground plane pose
                    PxTransform pose = PxTransform(PxVec3(0.0f, 0, 0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
                    //Create rigid static plane body
                    plane = physics->createRigidStatic(pose);
                    //Create the plane shape
                    PxShape* shape = plane->createShape(PxPlaneGeometry(), *planeMaterial);
                    if (!canRaycastPlane)shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
                }
                plane->setName("Plane");
                colors["Plane"] = groundColor;
                //Add the plane to the scene
                scene->addActor(*plane);
                
            }
            
            
            static void InitializeHeightfield() {
                //TEMPORAY HARD CODE FOR HEIGHT FIELD, this should be able to be adjusted in whatever the program is
                // set height field descriptors
                hfDesc.format = PxHeightFieldFormat::eS16_TM;
                hfDesc.nbColumns = numCols;
                hfDesc.nbRows = numRows;
                hfDesc.samples.data = &samples[0];
                hfDesc.samples.stride = sizeof(PxHeightFieldSample);
                
                //heightScale = 1.0/50.0f;
                //rowScale = 1.0f;
                //colScale = 1.0f;
                
                //Ground plane pose
                PxTransform pose = PxTransform(PxVec3(-numCols/2.0*rowScale, 0.0f, -numRows/2.0*colScale));
                //Create rigid static plane body
                plane = physics->createRigidStatic(pose);
                
                // make the height field!
                PxHeightField* aHeightField = physics->createHeightField(hfDesc);
                //PxReal heightScale, rowScale, colScale;
                //heightScale = rowScale = colScale = 2;
                PxHeightFieldGeometry hfGeom(aHeightField, PxMeshGeometryFlags(), heightScale, rowScale, colScale);
                PxShape *shape = plane->createShape(hfGeom, *planeMaterial);
            }
            
            
            static void setSimulationHeightField(int rows, int cols, PxI16* heights,
                                                 int hScale, int rScale, int cScale) {

            }
            
            /**
             *	@brief	Set default color to use in simulation
             *
             *	@param 	c 	Set default color (used for all entities that do not have another specified color)
             */
            static void setDefaultColor(rm3d::Color c) {

                default_color = c;
            }
            
            /**
             *	@brief	Returns the module at index i in the modules vector
             *
             *	@param 	i 	index of the desired module
             *
             *	@return	Pointer to the module at the given index
             */
            static Module<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint>* getModuleAtIndex(int i) {

                return modules[i];
            }
            
            /**
             *	@brief	Returns the module with the given name
             *
             *	@param 	name name of module
             *
             *	@return	Pointer to the module with give name
             */
            static Module<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint>* getModuleWithName(string name) {
                
                return modules[namesToNums[name]];
            }
            
            /**
             *	@brief	Creates a mapping between a color and a name such that all entities with that name are colored with c
             *
             *	@param 	c 	Color for entities named as key
             *	@param 	key 	name of entities to color with c
             */
            static void addColorToColorsDictionary(rm3d::Color c, const string key) {

                colors[key] = c;
            }
            
            /**
             *	@brief	Getter for physics
             *
             *	@return	PhysX sdk object pointer
             */
            static PxPhysics * getSimulationPhysics() {

                return physics;
            }
            
            /**
             *	@brief	Getter for foundation
             *
             *	@return	PhysX foundation object pointer
             */
            static PxFoundation * getSimulationFoundation() {
                return foundation;
            }
            
            /**
             *	@brief	Getter for cook
             *
             *	@return	PhysX cooking object pointer
             */
            static PxCooking * getSimulationCooking() {
                return cook;
            }
            
            /**
             *	@brief	Getter for scale
             *
             *	@return	the scale of the simulation
             */
            static float getSimulationScale() {

                return scale;
            }
            
            /**
             *	@brief	Return whether or not we are saving frames for the simulation
             *
             *	@return	boolean value indicating whether or not we are saving the simulation frames and should process them
             */
            static bool getSaveFrames() {

                return saveFrames;
            }

            /**
             *	@brief	Setter for simulation scale
             *
             *	@param 	s 	scale for the simulation ( > 1 means entities are larger than real-life dimensions and masses)
             */
            static void setSimulationScale(float s) {

                scale = s;
            }

            /**
             *	@brief	returns the number of modules in the modules vector
             *
             *	@return	the number of modules currently involved in the simulation
             */
            static int getNumModules() {

                return modules.size();
            }
            
            /**
             *	@brief	Getter for scene
             *
             *	@return	Pointer to PhysX scene object
             */
            static PxScene * getSimulationScene() {

                return scene;
            }
            
            /**
             *	@brief	applies an entire color map strategy to the simulation: specifies mappings between names and colors for all entities
             *
             *	@param 	color_map 	color map to apply to the simulation
             */
            static void setSimulatorColors(map<string, rm3d::Color> color_map) {

                colors = color_map;
            }
            
            /**
             *	@brief	Getter for modules, which holds references to all modules involved in  the simulation
             *
             *	@return	modules in the simulation
             */
            static vector<Module<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint> *> getModules() {

                return modules;
            }
            
            /**
             *	@brief	Sends a message from module to the module connected to dock
             *	@param 	module 	sending module
             *	@param 	dock 	dock out which message is sent
             *	@param 	message 	message contents
             */
            static void sendMessageToDock(rm3d::ai::Module<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint> *module, Dock dock, string message) {

                if (connections[module->getName()][dock->getDockFace()] == NULL) return;
                Message<Dock> *m = new Message<Dock>(message, dock, connections[module->getName()][dock->getDockFace()]);
                messageRoom[connections[module->getName()][dock->getDockFace()]->getModule()->getName()].push_back(m);
            }
            
            /**
             *	@brief	Sends a wireless message out from sendingModule's center of mass. The range of the message is a sphere
             *          centered at sendingModule's center of mass with a radius of range.
             *
             *	@param 	sendingModule 	module sending the message
             *	@param 	range 	range of the message
             *	@param 	message 	message contents
             */
            static void sendRangedMessage(Module<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint> *sendingModule, float range, string message) {

                rm3d::ai::RangedMessage *m = new rm3d::ai::RangedMessage(message, range);
                for (int i=0; i<(int)modules.size(); i++) {
                    if (!(modules[i]->getName().compare(sendingModule->getName()) == 0)) {
                        if (fabs((modules[i]->getPositionOrientation().p - sendingModule->getPositionOrientation().p).magnitude()) <= range) {
                            messageRoomRanged[modules[i]->getName()].push_back(m);
                        }
                    }
                }
            }
            
            /**
             *	@brief	retrieves the last message sent to the module with the given name
             *
             *	@param 	module 	name of module
             *
             *	@return	most recent message in module's queue of messages
             */
            static Message<Dock>* getLastMessage(string module) {

                if (messageRoom[module].size() == 0) return NULL;
                Message<Dock>* m = messageRoom[module].back();
                messageRoom[module].pop_back();
                return m;
            }
            
            /**
             *	@brief	retrieves the last ranged message sent to the module with the given name
             *
             *	@param 	module 	name of module
             *
             *	@return	most recent ranged message in module's queue of ranged messages
             */
            static rm3d::ai::RangedMessage* getLastRangedMessage(string module) {
                if (messageRoomRanged[module].size() == 0) return NULL;
                rm3d::ai::RangedMessage* m = messageRoomRanged[module].back();
                messageRoomRanged[module].pop_back();
                return m;
            }
            
            /**
             *	@brief	Clears (deletes) the messages of a module with the given name
             *
             *	@param 	module 	name of the module
             */
            static void clearMessages(string module) {

                messageRoom[module].clear();
            }
            
            /**
             *	@brief	Clears (deletes) the ranged messages of a module with the given name
             *
             *	@param 	module 	name of the module
             */
            static void clearRangedMessages(string module) {
                messageRoomRanged[module].clear();
            }
            
            /**
             *	@brief	tells the simulator to monitor a certain dock on a certain module for connections/disconnections
             *
             *	@param 	name 	name of the module
             *	@param 	dockNum 	which dock on the module to monitor
             *	@param 	dock 	a pointer to the dock object that represents the given dock of the given module
             */
            static void addDockToMonitor(Dock dock) {

                docksToMonitor.push_back(dock);
            }
            
            /**
             *	@brief	tells the simulator to stop monitoring a certain dock on a certain module
             *
             *	@param 	name 	name of the module
             *	@param 	dockNum 	the dock face to stop monitoring
             */
            static void removeDockFromMonitor(string name, DockFace dockNum) {

                int position = 0;
                for(typename vector<Dock>::iterator it = docksToMonitor.begin(); it != docksToMonitor.end(); ++it) {
                    Dock dock = *it;
                    if (dock->getDockFace() == dockNum && dock->getModule()->getName().compare(name) == 0) {
                        break;
                    }
                    position++;
                }
                if (docksToMonitor.size() > 0 && position < (int)docksToMonitor.size())docksToMonitor.erase(docksToMonitor.begin() + position);
            }
            
            
            /**
             *	@brief	Sets the gravity of the simulation
             *
             *	@param 	g 	vector describing the gravity of the simulation (vector of accelerations)
             */
            static void setSimulationGravity(PxVec3 g) {

                gravity = g;
            }
            
            /**
             *	@brief	Renders the given string at the given (x,y) position on the all subsequent OpenGL frames
             *
             *	@param 	buffer 	string to render
             *	@param 	x 	x-coordinate of left of string
             *	@param 	y 	y-coordinate of top of string
             */
            static void renderStringAtXYPosition(const char *buffer, int x, int y) {

                custom_buffers.push_back(buffer);
                custom_x.push_back(x);
                custom_y.push_back(y);
            }
            
            /**
             *	@brief	Removes all docks from the monitor that are to be deleted
             */
            static void updateDockLists() {

                for(int i=0; i<(int)deleteQueue.size(); i++) {
                    pair<string, DockFace> p = deleteQueue[i];
                    removeDockFromMonitor(p.first, p.second);
                }
                for(int i=0; i<(int)addQueue.size(); i++) {
                    Dock d = addQueue[i];
                    addDockToMonitor(d);
                }
                addQueue.clear();
                deleteQueue.clear();
            }
            
            /**
             *	@brief	returns the current simulator frame
             *
             *	@return	the current simulator frame as a cv::Mat object
             */
            static cv::Mat& getCurrentSimulatorFrame() {
                return image;
                
            }
            
            /**
             *	@brief	initializes the simulation and sets up the opengl callbacks and run loop
             *
             *	@param 	argc 	passed through from main
             *	@param 	*argv 	passed through from main
             *	@param 	width 	width of the OpenGL window in pixels
             *	@param 	height 	height of the OpenGl window in pixels
             *	@param 	camera_x 	x-coordinate of camera in world frame
             *	@param 	camera_y 	y-coordinate of camera in world frame
             *	@param 	camera_z 	z-coordinate of camera in world frame
             *	@param 	xrot 	tilt of camera (rotation around world x-axis)
             *	@param 	yrot 	pan of camera (rotation around world y-axis)
             */
            static void initializeSimulation(int argc, char **argv, int width, int height, rm3d::ai::Environment * env = NULL,
                                             float camera_x = 0.0, float camera_y = 2.0, float camera_z = 8.0, float xrot = 0.0, float yrot = 20.0) {
                
                if (env != NULL) {
                    gravity = env->gravity;
                    simTimestep = env->simulationTimeStep;
                    skyColor = env->skyColor;
                    groundColor = env->groundColor;
                    errorPositionDock = env->dockPositionTolerance;
                    errorOrientationDock = env->dockOrientationTolerance;
                    localTransformCamera = env->localTransformToCamera;
                    showFPS = env->showFPS;
                    saveFrames = env->shouldSaveFrames;
                    if (env->zNearPlane != 0.0) {
                        zNearPlane = env->zNearPlane;
                    }
                    if (env->zFarPlane != 0.0) {
                        zFarPlane = env->zFarPlane;
                    }
                    disableLigthing = env->disableLighting;
                    useHeightField = env->useHeightField;
                    numRows = env->heightFieldNumRows;
                    numCols = env->heightFieldNumCols;
                    samples.resize(numRows*numCols);
                    for (int i=0; i<(int)env->samples.size(); i++) {
                        samples[i].height = env->samples[i];
                        samples[i].materialIndex0 = 1;
                        samples[i].materialIndex1 = 1;
                        samples[i].setTessFlag();
                    }
                    heightScale = env->heightFieldHeightScale;
                    rowScale = env->heightFieldRowScale;
                    colScale = env->heightFieldColScale;
                    canRaycastPlane = env->canRaycastPlane;
                }
                atexit(OnShutdown);
                glutInit(&argc, argv);
                glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
                windowWidth = width;
                windowHeight = height;
                glutInitWindowSize(width, height);
                int mainHandle = glutCreateWindow("ReMod3D Simulation");
                glutSetWindow(mainHandle);
                glClearColor(skyColor.r, skyColor.g, skyColor.b, 1.0);
                glEnable(GL_DEPTH_TEST);
                glEnable(GL_COLOR_MATERIAL);
                glEnable(GL_LIGHTING);
                glEnable(GL_CULL_FACE);
                glShadeModel(GL_SMOOTH);
                glEnable(GL_POINT_SMOOTH);
                glEnable(GL_LINE_SMOOTH);
                //glEnable(GL_POLYGON_SMOOTH);
                glEnable(GL_SMOOTH);
                glHint(GL_POLYGON_SMOOTH_HINT,GL_FASTEST);
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
                float ambientColor[]	= { 0.0f, 0.1f, 0.2f, 0.0f };
                float diffuseColor[]	= { 1.0f, 1.0f, 1.0f, 0.0f };
                float specularColor[]	= { 0.0f, 0.0f, 0.0f, 0.0f };
                float position[]		= { 100.0f, 100.0f, 400.0f, 1.0f };
                glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor);
                glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseColor);
                glLightfv(GL_LIGHT0, GL_SPECULAR, specularColor);
                glLightfv(GL_LIGHT0, GL_POSITION, position);
                glEnable(GL_LIGHT0);
                glutDisplayFunc(OnRender);
                glutIdleFunc(OnIdle);
                glutReshapeFunc(OnReshape);
                glutKeyboardFunc(KeyboardCallback);
                glutSpecialFunc(ArrowKeyCallback);
                glutMouseFunc(Mouse);
                glutMotionFunc(MotionRotate);
                camera.setInitialPosOrientation(camera_x, camera_y, camera_z, xrot, yrot);
                if (init_function_sim) {
                    PreInitializePhysX();
                    init_function_sim();
                } else {
                    PreInitializePhysX();
                }
                glutMainLoop();
            }
            
            

            
        };

        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        rm3d::Camera rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::camera;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         float rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::motion_scale_factor = 0.1;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         PxPhysics* rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::physics;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         PxFoundation* rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::foundation;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         PxCooking* rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::cook;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         PxVec3 rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::gravity = PxVec3(0.0,-9.8, 0.0);
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         PxScene* rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::scene;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         PxRigidStatic* rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::plane;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         PxMaterial* rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::planeMaterial;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         PxDefaultErrorCallback rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::gDefaultErrorCallback;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         PxDefaultAllocator rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::gDefaultAllocatorCallback;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         PxSimulationFilterShader rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::gDefaultFilterShader = PxDefaultSimulationFilterShader;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        map<string, rm3d::Color> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::colors;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          map<string, int> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::namesToNums;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         vector<Module<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint> *> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::modules;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         vector<const char *> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::custom_buffers;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          vector<int> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::custom_x;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         vector<int> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::custom_y;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          map<char, void(*)()> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::keys_to_functions;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        rm3d::Color rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::default_color;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          PxReal rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::simTimestep = 1.0f/60.0f;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         float rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::fps;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         int rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::startTime;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         int rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::totalFrames;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         float rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::scale = 2.0;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          int rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::initial_wait_count = 60;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          bool rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::pause_program = false;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          char rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::buffer[MAX_PATH];
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          char *rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::custom_buffer;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          vector<Dock> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::docksToMonitor;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          string moduleNumsAndDocksToString(int module1_num, int module2_num, DockFace module1_dock, DockFace module2_dock);
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          int rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::windowWidth;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          double rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::errorPositionDock = 0.05;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          vector<pair<string, DockFace> > rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::deleteQueue;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        vector<Dock > rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::addQueue;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          map<string, map<DockFace, Dock> > rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::connections;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          map<string, vector<Message<Dock>*> > rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::messageRoom;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        map<string, vector<rm3d::ai::RangedMessage *> > rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::messageRoomRanged;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
          double rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::errorOrientationDock = 0.05;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
         int rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::windowHeight;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        void (*rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::init_function_sim)() = NULL;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        void (*rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::step_function_sim)() = NULL;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        void (*rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::shutdown_function_sim)() = NULL;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        void (*rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::custom_render_function_sim)() = NULL;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        vector<void (*)()> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::sim_step_functions;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        rm3d::Color rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::groundColor = rm3d::Color(84.0f/256.0f,65.0f/256.0f,50/256.0f);
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        rm3d::Color rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::skyColor = rm3d::Color(109.0/256.0f, 172.0/256.0f, 207.0f/256.0f);
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        vector<Obstacle> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::obstacles;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        map<string, int> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::namesToObstacles;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        vector<PxTransform> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::framesToRenderForBodies;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        vector<Raycast> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::raycastsToRender;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        int rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::currentModuleNum = -1;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        uint64_t rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::currentFrameNumber = -1;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        bool rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::saveFrames = false;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        cv::Mat rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::image;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        cv::Mat rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::flipped;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        double rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::accumulator = 0.0;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        float rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::zNearPlane = 1.0;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        float rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::zFarPlane = 1000.0;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        bool rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::disableLigthing = false;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        map<string,PxParticleFluid *> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::particleFluids;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        map<string, PxParticleSystem *> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::particleSystems;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        map<string, Cloth *> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::cloths;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        PxHeightFieldDesc rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::hfDesc;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        vector<PxHeightFieldSample> rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::samples;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        int rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::numRows;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        int rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::numCols;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        float rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::rowScale = 1.0;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        float rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::colScale = 1.0;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        bool rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::useHeightField = false;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        float rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::heightScale = 1.0;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        bool rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::canRaycastPlane = false;
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        PxTransform rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::localTransformCamera = PxTransform(PxVec3(0));
        template <class State, class Link, class IntraJoint, class DockShape, class BodyShape, class ModuleBody, class DockJoint, class Dock, class DockFace>
        bool rm3d::ai::Simulator<State, Link, IntraJoint, DockShape, BodyShape, ModuleBody, DockJoint, Dock, DockFace>::showFPS = true;
        
        
    };
};
#endif
