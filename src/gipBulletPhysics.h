/*
 *  gipBulletPhysics.h
 *
 *
 *	Edited 		: 16.02.2023
 *  	Author 	: Remzi ISCI
 */

#ifndef SRC_GIPBULLETPHYSICS_H_
#define SRC_GIPBULLETPHYSICS_H_

#include <iostream>
#include <vector>
#include "gBasePlugin.h"
#include "gImageGameObject.h"
#include "bullet/btBulletDynamicsCommon.h"

#include "glm/glm.hpp"


class gipBaseGameObject;

class gipBulletPhysics : public gBasePlugin {
public:
	friend class gipBaseGameObject;

	/*
	* Layers are bitwise varialbles
	 * You can use multiple layers together
	 * Use | (Bite or operator) to use multiple layer together
	 * etc : LAYER1 | LAYER2 | LAYER12 | LAYER22
	 */
	enum COLLISIONLAYERS {
		LAYERNONMEMBER = -1, //Dont use this, this just for check
		LAYER0 = 1 << 0,	//Dont collide layer
		LAYER1 = 1 << 1,	//Default collide layer
		LAYER2 = 1 << 2,
		LAYER3 = 1 << 3,
		LAYER4 = 1 << 4,
		LAYER5 = 1 << 5,
		LAYER6 = 1 << 6,
		LAYER7 = 1 << 7,
		LAYER8 = 1 << 8,
		LAYER9 = 1 << 9,
		LAYER10 = 1 << 10,
		LAYER11 = 1 << 11,
		LAYER12 = 1 << 12,
		LAYER13 = 1 << 13,
		LAYER14 = 1 << 14,
		LAYER15 = 1 << 15,
		LAYER16 = 1 << 16,
		LAYER17 = 1 << 17,
		LAYER18 = 1 << 18,
		LAYER19 = 1 << 19,
		LAYER20 = 1 << 20,
		LAYER21 = 1 << 21,
		LAYER22 = 1 << 22
	};

	//Physic World types, choose WORLDTYPE_RIGIDWORLD for better performance
	enum WORLDTYPE {
			WORLDTYPE_RIGIDWORLD = 0,
			WORLDTYPE_SOFTWORLD = 1
		};

	enum WORLDCOORDINATETYPE {
		WORLD2D = 0,
		WORLD3D = 1
	};

	gipBulletPhysics(WORLDCOORDINATETYPE worldcoordinate, WORLDTYPE worldType = WORLDTYPE::WORLDTYPE_RIGIDWORLD);
	virtual ~gipBulletPhysics();



	/*
	 * Physic world simulation time step 60 is deafult and ideal for most of time
	 */
	inline static void setTimeStep(float timestep);

	// Delete initialized objects
	void clean();
	// for 2d set z axis 0
	void setErp2(float value = 0.0f);
	void setglobalCfm(float value = 0.0f);
	void setNumIterations(int value = 3);
	void setSolverMode(int solverMode = SOLVER_SIMD);
	void setSplitImpulse(int splitImpulse = false);
	void setGravity(glm::vec3 gravityValue);

	// These apply methods should be used in draw method.
	void drawDebug();


 	int runPhysicWorldStep();

	/*
	 * Gets gravity ovf physicworld
	 */
	 glm::vec3 getGravity();

	float getErp2();
	float getglobalCfm();

	int getNumIterations();
	int getSolverMode();
	int getSplitImpulse();

	// Call it in stepSimulation method to see the position and rotation of the objects.
	void printObjectTransform();

	/*
	 * This function calls by physicobjects childes
	 * layers are bitwise variables
	 * use COLLISIONLAYERS enums to set layers
	 * objectlayers means object will own that flags
	 * targetlayers means object only will collide thouse layers
	 */
	int addPhysicObect(gipBaseGameObject* targetobject, int objectlayer, int masklayer);

protected:
	void removeObject(int id);

	void updateSingleAabb(btCollisionObject* rigidbody);



	/*
	 * Call this function for setting object layers
	 * LAYER0 means dont collide
	 */
	void setObjectlayers(int objectid, COLLISIONLAYERS objectlayers);

	/*
	 *Call this function for setting target layers which want to collide with object
	 *LAYER0 means dont collide
	 */
	void setTargetLayers(int objectid, COLLISIONLAYERS targetlayers);

	/*
	 * For setting mass
	 * To change mass of object  need control physic world directly
	 */
	void setMass(gipBaseGameObject* targetobject, float newmass);
private:
	/*
	 * initialize world first
	 * worldcoordinate need for debuging
	 */
	void initializeWorld(WORLDCOORDINATETYPE worldcoordinate, WORLDTYPE worldType = WORLDTYPE::WORLDTYPE_RIGIDWORLD);

	//collision dedection codes
	void checkCollisions();


	/*
	 * Nedded referances and variables for physic world
	 */
	btDefaultCollisionConfiguration* collisionconfiguration;
	btCollisionDispatcher* collisiondispatcher;
	//Needed for multithread collision dedection
	btBroadphaseInterface* overlappingpaircache;
	btSequentialImpulseConstraintSolver* solver;
	btConstraintSolver* constraintsolver;
	//Need another thread for soft objects
	btBroadphaseInterface* softwolrdbroadphase;

	btBroadphaseInterface* broadphase;

	//Physic world
	btDiscreteDynamicsWorld* _dynamicsworld;


	//Physic world will work 60 times per second, ideal for 60fps
	btScalar _timestep = 60.0f;
	// step time for each update higher value will simulate game faster
	int _maxsubsteps = 10;
	btScalar _fixedtimestep = btScalar((1.0f)/_timestep);

	//List of physic object which has been added world
	std::vector<gipBaseGameObject*> _objectlist;

	bool _isworldinitiliazed = false;

};

#endif /* SRC_GIPBULLETPHYSICS_H_ */
