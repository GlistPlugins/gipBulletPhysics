/*
 *  gipBulletPhysics.h
 *
 *
 *	Edited 	: 16.02.2023
 *  Author 	: Remzi ISCI
 */

#ifndef SRC_GIPBULLETPHYSICS_H_
#define SRC_GIPBULLETPHYSICS_H_

#include <iostream>
#include <vector>
#include "gBasePlugin.h"
#include "gImageGameObject.h"
#include "bullet/btBulletCollisionCommon.h"
#include "bullet/btBulletDynamicsCommon.h"
#include "bullet/BulletCollision/btBulletCollisionCommon.h"
#include "bullet/BulletCollision/CollisionDispatch/btGhostObject.h"
#include "bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"

#include "glm/glm.hpp"


class gipBaseGameObject;

class gipRaycastResult {
public:
	//this should be spelled "hitobject" or "objecthit"
	gipBaseGameObject* hittedobject;
	//this should be "contactpoint"
	glm::vec3 hitpoint;
};

class gipBulletPhysics : public gBasePlugin {
public:
	static gipBulletPhysics* plugin;

	friend class gipBaseGameObject;
	enum COLLISIONOBJECTTYPE {
		COLLISIONOBJECTTYPE_RIGIDBODY,
		COLLISIONOBJECTTYPE_GHOST
	};
	/*
	 * Layers are bitwise varialbles
	 * You can use multiple layers together
	 * Use | (Bitwise or operator) to use multiple layer together
	 * e.g.: LAYER1 | LAYER2 | LAYER12 | LAYER22
	 */
	enum COLLISIONLAYERS {
		LAYERNONMEMBER = -1, //Dont use this, this just for check
		LAYER0 = 1 << 0,	//Dont collide layer
		LAYER1 = 1 << 1,	//Default collide layer
		LAYER2 = 1 << 2,	//Player bullet layer
		LAYER3 = 1 << 3,	//player layer
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


 	void runPhysicWorldStep() {
        runPhysicWorldStep(1000.0f / 60.0f);
    }

 	void runPhysicWorldStep(float deltatime);

	/*
	 * Gets gravity ovf physicworld
	 */
	 glm::vec3 getGravity();

	float getErp2();
	float getglobalCfm();

	int getNumIterations();
	int getSolverMode();
	int getSplitImpulse();

	/*
	 * This function is called by physicobjects childs
	 * layers are bitwise variables
	 * use COLLISIONLAYERS enums to set layers
	 * objectlayers means object will own those flags
	 * targetlayers means object will only collide with those layers
	 */
	void addPhysicObject(gipBaseGameObject* targetobject, int objectlayer, int masklayer);

	//This function doesn't work, rewrite needed, use gGhostGameObject3D or gGhostGameObject2D for ray
	bool raycastHit(glm::vec3 from, glm::vec3 to, int masklayers, gipRaycastResult* result);

	//Physic world
	btDiscreteDynamicsWorld* _dynamicsworld;

protected:
	void removeObject(gipBaseGameObject* object);

private:
	/*
	 * initialize world first
	 * worldcoordinate is needed for debugging
	 */
	void initializeWorld(WORLDCOORDINATETYPE worldcoordinate, WORLDTYPE worldType = WORLDTYPE::WORLDTYPE_RIGIDWORLD);

	//collision detection codes
	void checkCollisions();

	static void internalTick(btDynamicsWorld* world, btScalar timeStep);

	/*
	 * Needed references and variables for world physics
	 */
	btDefaultCollisionConfiguration* collisionconfiguration;
	btCollisionDispatcher* collisiondispatcher;
	//Needed for multithread collision dedection
	btBroadphaseInterface* overlappingpaircache;
	btSequentialImpulseConstraintSolver* solver;
	btConstraintSolver* constraintsolver;
	//Another thread is needed for soft objects
	btBroadphaseInterface* softwolrdbroadphase;

	btBroadphaseInterface* broadphase;

	//List of physic objects which have been added to the world
	std::deque<gipBaseGameObject*> _objects;

	bool _isworldinitiliazed = false;

};

#endif /* SRC_GIPBULLETPHYSICS_H_ */
