/*
 *  gipBulletPhysics.cpp
 *
 *  Created on: 5 Aug 2022
 *      Author: Faruk Aygun
 *      		Emirhan Limon
 */

#include "gipBulletPhysics.h"


gipBulletPhysics::gipBulletPhysics() {
}

gipBulletPhysics::~gipBulletPhysics() {
}

void gipBulletPhysics::update() {
}

void gipBulletPhysics::initialize() {
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	overlappingPairCache = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver ;
	dynamicsWorld = new btDiscreteDynamicsWorld (dispatcher, overlappingPairCache, solver, collisionConfiguration);
}

void gipBulletPhysics::setGravity(float gravityValue) {
	dynamicsWorld->setGravity(btVector3 (0, gravityValue, 0));
}

void gipBulletPhysics::addRigidBody(btRigidBody* rb) {
	dynamicsWorld->addRigidBody(rb);
}

void gipBulletPhysics::create2dBoxObject(gImage img, float x, float y, float objMass) {
	btTransform box2dTransform;

	btCollisionShape* box2dShape = new btBoxShape(btVector3(img.getWidth(), img.getHeight(), 1.0f));
	collisionShapes.push_back(box2dShape);

	box2dTransform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 */
	box2dTransform.setOrigin(btVector3(x, -(y + img.getHeight()), 0));
	btScalar mass(objMass);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		box2dShape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(box2dTransform);
	btRigidBody::btRigidBodyConstructionInfo box2dRbInfo(mass, myMotionState, box2dShape, localInertia);
	btRigidBody* box2dRigidBody = new btRigidBody(box2dRbInfo);

	addRigidBody(box2dRigidBody);

	// gLogi("box") << float(box2dTransform.getOrigin().getX()) << " " << float(box2dTransform.getOrigin().getY());
}

void gipBulletPhysics::create2dCircleObject(gImage img, float x, float y, float objMass) {
	btTransform circle2dTransform;

	// parameter is circle radius
	btCollisionShape* circle2dShape = new btSphereShape(img.getWidth() / 2);
	collisionShapes.push_back(circle2dShape);

	circle2dTransform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the center for circle object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getWidth() / 2 and +img.getHeight() / 2).
	 */
	circle2dTransform.setOrigin(btVector3(x + (img.getWidth() / 2), -(y + img.getHeight() / 2), 0));

	btScalar mass(objMass);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		circle2dShape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(circle2dTransform);
	btRigidBody::btRigidBodyConstructionInfo circle2dRbInfo(mass, myMotionState, circle2dShape, localInertia);
	btRigidBody* circle2dRigidBody = new btRigidBody(circle2dRbInfo);

	addRigidBody(circle2dRigidBody);

	// gLogi("circle") << float(circle2dTransform.getOrigin().getX()) << " " << float(circle2dTransform.getOrigin().getY());
}

int gipBulletPhysics::stepSimulation(btScalar timeStep, int maxSubSteps , btScalar fixedTimeStep) {
	return dynamicsWorld->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
}

int gipBulletPhysics::getNumCollisionObjects() {
	return dynamicsWorld->getNumCollisionObjects();
}

btCollisionObjectArray& gipBulletPhysics::getCollisionObjectArray() {
	return dynamicsWorld->getCollisionObjectArray();
}

btVector3& gipBulletPhysics::getOrigin(btTransform* trans) {
	return trans->getOrigin();
}

// convert bullet3 positions to Glist Engine positions and return for circle objects.
btVector3 gipBulletPhysics::getCircle2dObjectPosition(btTransform trans, float imgWidth, float imgHeight) {
	// gLogi("circle (x,y)") << trans.getOrigin().getX() - (imgWidth / 2) << " " << -(trans.getOrigin().getY() + imgHeight / 2);
	return btVector3(
			trans.getOrigin().getX() - (imgWidth / 2),
			-(trans.getOrigin().getY() + imgHeight / 2),
			0.0f
	);
}

// convert bullet3 positions to Glist Engine positions and return for box objects.
btVector3 gipBulletPhysics::getBox2dObjectPosition(btTransform trans, float imgWidth, float imgHeight) {
	// gLogi("box (x,y)") << trans.getOrigin().getX() << " " << -(trans.getOrigin().getY() + imgHeight);
	return btVector3(
			trans.getOrigin().getX(),
			-(trans.getOrigin().getY() + imgHeight),
			0.0f
	);
}

void gipBulletPhysics::clean() {
	delete collisionConfiguration;
	delete dispatcher;
	delete overlappingPairCache;
	delete solver;
	delete dynamicsWorld;
}


