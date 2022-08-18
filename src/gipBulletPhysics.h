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

#include "bullet/btBulletDynamicsCommon.h"

#include <stdio.h>

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
	void create2dBoxObject(gImage img, float x, float y, float objMass);
	void create2dCircleObject(gImage img, float x, float y, float objMass);

	// The btScalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
	int  stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.));
	int  getNumCollisionObjects();

	// Return the origin vector translation
	btVector3& getOrigin(btTransform* trans);
	// Unlike getOrigin, these two methods arranges and returns positions according to the Glist Engine.
	btVector3 getCircle2dObjectPosition(btTransform trans, float imgWidth, float imgHeight);
	btVector3 getBox2dObjectPosition(btTransform trans, float imgWidth, float imgHeight);
	btCollisionObjectArray& getCollisionObjectArray();

	// keep track of the shapes, we release memory at exit.
	// make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

private:
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
};

#endif /* SRC_GIPBULLETPHYSICS_H_ */
