/*
 * gPhysic.h
 *
 *  Created on: 19 �ub 2023
 *      Author: Remzi ����
 *
 *      This clas is for physic implementation to Glist Engine
 *      This class is based on singleton pattern
 *      Class is works for basic physic initiliaze, store data and destruction
 */

#ifndef SRC_BASES_GPHYSIC_H_
#define SRC_BASES_GPHYSIC_H_

#include "gBasePlugin.h"
#include "bullet/btBulletDynamicsCommon.h"
#include "gPhysicObject.h"
#include "glm/glm.hpp"
#include "gDebugDraw.h"

class gPhysic : public gBasePlugin {
public:
	//Singletion class should not be cloneable
	gPhysic(const gPhysic& obj) = delete;
	/*
	 * Box = 0
	 * Sphere = 8
	 * Compound = 31
	 */
	enum TRANSFORMTYPE {
		TRANSFORMTYPE_BOX = 0,
		TRANSFORMTYPE_SPHERE = 8,
		TRANSFORMTYPE_COMPOUND = 31,
	};

	/*
	 * Layers are bitwise varialbles
	 *
	 */
	 enum COLLISIONLAYERS {
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

	//Singletons should not be assignable.
	void operator = (const gPhysic &) = delete;

	//Constructor for singleton
	static gPhysic *Instance();

	//Call this function to start physic
	void startWorld(bool is2d,float timestep = 60);

	//Call this function for each frame will run physic world
	int runPhysicWorldStep();


	/*	This method is for drawing physic pbjects as debug mode
	*	Need to be called from game render draw function else it wont be called
	*/
	void drawDebug();

	/* This function is for printing the position and rotation of the objects.
	 * Call this function just for debuging and dont forget to remove when gettin rlease
	 */
	void printObjectTransform();

	/*
	 * This function calls by physicobjects childes
	 * layer mas is must become bitwise
	 * use COLLISIONLAYERS enums
	 * object layer means object will own that flag
	 * masklayer means object only will collide thouse layers
	 */
	int addPhysicObect(gPhysicObject* object, int objectlayer, int masklayer);



	void updateSingleAabb(btCollisionObject* targetcollisionobject);

	void setGravity(glm::vec3 newgravity);

	btVector3 getGravity();

	void setTimeStep(float timestep);

protected:




	/* initialize physic world first
	 * Will be called when class contstructed
	 * For better performance choose WORLDTYPE_RIGIDWORLD
	 */
	void initializeWorld(bool is2d,int worldType = WORLDTYPE_RIGIDWORLD);


	// Delete initialized objects for cleaning mamory
	void clean();

	//for detection collisions
	void checkCollisions();

private:
	gPhysic();
	virtual ~gPhysic();

	inline static gPhysic* m_physic;


	//World types
	enum worldType {
		WORLDTYPE_RIGIDWORLD,
		WORLDTYPE_SOFTWORLD
	};


	/*
	 * List of collision shapes which has been added world
	 * Make sure to reuse shapes rether than creating new one each time  if its possible
	 */
	btAlignedObjectArray<btCollisionShape*> collisionshapes;


	//List of objects which has been added world
	std::vector<gPhysicObject*> physicobjects;

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


	//Physic world
	btDiscreteDynamicsWorld* dynamicsworld;


	//primitive variables

	//Physic world will work 60 times per second, ideal for 60fps
	inline static btScalar _timestep;
	int maxsubsteps = 1;
	btScalar fixedtimestep = btScalar((1.0f)/btScalar(60.0f));
};



#endif /* SRC_BASES_GPHYSIC_H_ */
