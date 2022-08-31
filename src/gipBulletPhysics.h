/*
 *  gipBulletPhysics.h
 *
 *  Created on: 5 Aug 2022
 *      Author: Faruk Aygun,
 *      		Emirhan Limon
 */

#ifndef SRC_GIPBULLETPHYSICS_H_
#define SRC_GIPBULLETPHYSICS_H_

#include "gBasePlugin.h"
#include "gImageGameObject.h"

#include "bullet/btBulletDynamicsCommon.h"

#include "glm/glm.hpp"

class gipBulletPhysics : public gBasePlugin{
public:
	gipBulletPhysics();
	virtual ~gipBulletPhysics();

	void update();

	// RigidWorld should be initialized if all objects have rigidbody.
	void initializeRigidWorld();
	// SoftRigidWorld should be initialized if some objects have softbody.
	void initializeSoftRigidWorld();

	// Delete initialized objects
	void clean();
	// for 2d set z axis 0
	void setGravity(glm::vec3 gravityValue);
	// These 6 methods should be used in update method.
	void setCentralForce(gImageGameObject* imgObject, glm::vec3 forceValue);
	void setCentralImpulse(gImageGameObject* imgObject, glm::vec3 impulseValue);
	void setForce(gImageGameObject* imgObject, glm::vec3 forceValue, glm::vec3 relPos);
	void setImpulse(gImageGameObject* imgObject, glm::vec3 impulseValue, glm::vec3 relPos);
	void setTorque(gImageGameObject* imgObject, glm::vec3 torqueValue);
	void setTorqueImpulse(gImageGameObject* imgObject, glm::vec3 torqueValue);

	// Create methods return created object id
	int createBox2dObject(gImageGameObject* imgObject);
	int createCircle2dObject(gImageGameObject* imgObject);
	int createSoftContactBox2dObject(gImageGameObject* imgObject, float stiffness = 300.0f, float damping = 10.0f);
	int createSoftCircle2dObject(gImageGameObject* imgObject);
	// The btScalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
	int  stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.));
	int  getNumCollisionObjects();

	// Return the origin vector translation
	glm::vec2 getOrigin2d(int gameObjectNo);
	/*
	 * Unlike getOrigin, these two methods arranges and returns positions according to the Glist Engine.
	 * These get methods works for soft objects.
	 */
	// convert bullet3 positions to Glist Engine positions and return for circle objects.
	glm::vec2 getCircle2dObjectPosition(gImageGameObject* imgObject);
	// convert> a bullet3 positions to Glist Engine positions and return for box objects.
	glm::vec2 getBox2dObjectPosition(gImageGameObject* imgObject);

	btCollisionObjectArray& getCollisionObjectArray();

	// keep track of the shapes, we release memory at exit.
	// make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionshapes;
	// keep track of the created game object ids
	std::vector<int> gameobjectid;

private:
	btDefaultCollisionConfiguration* collisionconfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingpaircache;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsworld;

	//soft contact
	btConstraintSolver* constraintsolver;
	btBroadphaseInterface* broadphase;
};

#endif /* SRC_GIPBULLETPHYSICS_H_ */
