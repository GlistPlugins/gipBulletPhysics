/*
 * gPhysic.h
 *
 *  Created on: 19 Þub 2023
 *      Author: Remzi ÝÞÇÝ
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
	enum transformtype {
		TRANSFORMTYPE_BOX = 0,
		TRANSFORMTYPE_SPHERE = 8,
		TRANSFORMTYPE_COMPOUND = 31,
	};

	//Singletons should not be assignable.
	void operator = (const gPhysic &) = delete;

	//Constructor for singleton
	static gPhysic *Instance();

	//Call this function to start physic
	void startWorld(float timestep = 60);

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
	 */
	int addPhysicObect(gPhysicObject* object);

	void updateSingleAabb(btCollisionObject* targetcollisionobject);

	void setGravity(glm::vec3 newgravity);

	btVector3 getGravity();

	void setTimeStep(float timestep);

protected:




	/* initialize physic world first
	 * Will be called when class contstructed
	 * For better performance choose WORLDTYPE_RIGIDWORLD
	 */
	void initializeWorld(int worldType = WORLDTYPE_RIGIDWORLD);


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
	int maxsubsteps = 10;
	btScalar fixedtimestep = btScalar((1.0f)/btScalar(60.0f));
};



#endif /* SRC_BASES_GPHYSIC_H_ */
