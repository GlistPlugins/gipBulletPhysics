/*
 *  gipBulletPhysics.h
 *
 *  Created on: 5 Aug 2022
 *      Author: Faruk Aygun,
 *      		Emirhan Limon
 *
 *	Edited 		: 16.02.2023
 *  	Author 	: Remzi iﬁ«›
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

	// keep track of the shapes, we release memory at exit.
	// make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionshapes;
	// keep track of the created game objects
	std::vector<gImageGameObject*> gameobjects;

	void update();

	// initialize world first
	void initializeWorld(int worldType);

	// Delete initialized objects
	void clean();
	// for 2d set z axis 0
	void setErp2(float value = 0.0f);
	void setglobalCfm(float value = 0.0f);
	void setNumIterations(int value = 3);
	void setSolverMode(int solverMode = SOLVER_SIMD);
	void setSplitImpulse(int splitImpulse = false);
	void setGravity(glm::vec3 gravityValue);
	void setFriction(gImageGameObject* imgObject, float frictionValue);
	void setRollingFriction(gImageGameObject* imgObject, float frictionValue);
	void setSpinningFriction(gImageGameObject* imgObject, float frictionValue);
	void setAnisotropicFriction(gImageGameObject* imgObject, glm::vec3 frictionValue, int frictionMode);

	// These apply methods should be used in update method.
	void applyCentralForce(gImageGameObject* imgObject, glm::vec3 forceValue);
	void applyCentralImpulse(gImageGameObject* imgObject, glm::vec3 impulseValue);
	void applyForce(gImageGameObject* imgObject, glm::vec3 forceValue, glm::vec3 relPos);
	void applyImpulse(gImageGameObject* imgObject, glm::vec3 impulseValue, glm::vec3 relPos);
	void applyTorque(gImageGameObject* imgObject, glm::vec3 torqueValue);
	void applyTorqueImpulse(gImageGameObject* imgObject, glm::vec3 torqueValue);

	// These apply methods should be used in draw method.
	void drawDebug();

	float getErp2();
	float getglobalCfm();

	int getNumIterations();
	int getSolverMode();
	int getSplitImpulse();
	// Create methods return created object id
	int createBox2dObject(gImageGameObject* imgObject);
	int createCircle2dObject(gImageGameObject* imgObject);
	// increase stiffness, reduce dumping for harder floor. rotation is degree not radyan
	int createSoftContactBox2dObject(gImageGameObject* imgObject, float stiffness = 300.0f, float damping = 10.0f, float rotation = 0.0f);
	int createSoftCircle2dObject(gImageGameObject* imgObject);
	// The btScalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
	int stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.));

	// Return the origin vector translation
	glm::vec2 getOrigin2d(int imgObjectId);
	/*
	 * Unlike getOrigin, these two methods arranges and returns positions according to the Glist Engine.
	 * These get methods works for soft objects too.
	 */
	// convert bullet3 positions to Glist Engine positions and return for circle objects.
	glm::vec2 getCircle2dObjectPosition(gImageGameObject* imgObject);
	// convert a bullet3 positions to Glist Engine positions and return for box objects.
	glm::vec2 getBox2dObjectPosition(gImageGameObject* imgObject);
	glm::vec3 get2dObjectRotation(gImageGameObject* imgObject);

private:
	enum worldType {
		rigidWorld = 0,
		softRigidWorld = 1
	};

	btDefaultCollisionConfiguration* collisionconfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingpaircache;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsworld;

	//soft contact
	btConstraintSolver* constraintsolver;
	btBroadphaseInterface* broadphase;

	btTransform getTransform(int imgObjectId);
	btRigidBody* getRigidBody(gImageGameObject* imgObject);

	// Call it in stepSimulation method to see the position and rotation of the objects.
	void printObjectTransform();
};

#endif /* SRC_GIPBULLETPHYSICS_H_ */
