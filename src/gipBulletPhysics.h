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

	void initialize();
	// Delete initialized objects
	void clean();
	void setGravity(float gravityValue);
	void addRigidBody(btRigidBody* rb);
	void create2dBoxObject(gImageGameObject* imgobject, float objMass);
	void create2dCircleObject(gImageGameObject* imgobject, float objMass);

	// The btScalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
	int  stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.));
	int  getNumCollisionObjects();

	// Return the origin vector translation
	glm::vec2 getOrigin2d(btTransform* trans);
	// Unlike getOrigin, these two methods arranges and returns positions according to the Glist Engine.
	glm::vec2 getCircle2dObjectPosition(btTransform trans, float imgWidth, float imgHeight);
	glm::vec2 getBox2dObjectPosition(btTransform trans, float imgWidth, float imgHeight);

	btCollisionObjectArray& getCollisionObjectArray();

	// keep track of the shapes, we release memory at exit.
	// make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionshapes;

private:
	btDefaultCollisionConfiguration* collisionconfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingpaircache;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsworld;
};

#endif /* SRC_GIPBULLETPHYSICS_H_ */
