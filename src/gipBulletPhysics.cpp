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
	collisionconfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionconfiguration);
	overlappingpaircache = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver ;
	dynamicsworld = new btDiscreteDynamicsWorld (dispatcher, overlappingpaircache, solver, collisionconfiguration);
}

void gipBulletPhysics::setGravity(float gravityValue) {
	dynamicsworld->setGravity(btVector3 (0, gravityValue, 0));
}

void gipBulletPhysics::createRigidBody(btRigidBody* rigidBody) {
	dynamicsworld->addRigidBody(rigidBody);
}

int gipBulletPhysics::createBox2dObject(gImageGameObject* imgObject, float objMass) {
	btTransform box2dtransform;

	btCollisionShape* box2dshape = new btBoxShape(btVector3(imgObject->image.getWidth(), imgObject->image.getHeight(), 1.0f));
	collisionshapes.push_back(box2dshape);

	box2dtransform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 */
	box2dtransform.setOrigin(btVector3(imgObject->positionx, -(imgObject->positiony + imgObject->image.getHeight()), 0));
	btScalar mass(objMass);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		box2dshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(box2dtransform);
	btRigidBody::btRigidBodyConstructionInfo box2drbinfo(mass, mymotionstate, box2dshape, localInertia);
	btRigidBody* box2drigidbody = new btRigidBody(box2drbinfo);

	createRigidBody(box2drigidbody);

	gameobjectid.push_back(gameobjectid.size());
	// gLogi("box") << float(box2dTransform.getOrigin().getX()) << " " << float(box2dTransform.getOrigin().getY());

	return gameobjectid.back();
}

int gipBulletPhysics::createCircle2dObject(gImageGameObject* imgObject, float objMass) {
	btTransform circle2dtransform;

	// parameter is circle radius
	btCollisionShape* circle2dshape = new btSphereShape(imgObject->image.getWidth() / 2);
	collisionshapes.push_back(circle2dshape);

	circle2dtransform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the center for circle object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getWidth() / 2 and +img.getHeight() / 2).
	 */
	circle2dtransform.setOrigin(
			btVector3(imgObject->positionx + (imgObject->image.getWidth() / 2),
			-(imgObject->positiony + imgObject->image.getHeight() / 2), 0)
	);

	btScalar mass(objMass);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isdynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isdynamic)
		circle2dshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(circle2dtransform);
	btRigidBody::btRigidBodyConstructionInfo circle2drbinfo(mass, mymotionstate, circle2dshape, localInertia);
	btRigidBody* circle2drigidbody = new btRigidBody(circle2drbinfo);

	createRigidBody(circle2drigidbody);

	gameobjectid.push_back(gameobjectid.size());
	// gLogi("circle") << float(circle2dTransform.getOrigin().getX()) << " " << float(circle2dTransform.getOrigin().getY());

	return gameobjectid.back();
}

int gipBulletPhysics::stepSimulation(btScalar timeStep, int maxSubSteps , btScalar fixedTimeStep) {
	return dynamicsworld->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
}

int gipBulletPhysics::getNumCollisionObjects() {
	return dynamicsworld->getNumCollisionObjects();
}

btCollisionObjectArray& gipBulletPhysics::getCollisionObjectArray() {
	return dynamicsworld->getCollisionObjectArray();
}

glm::vec2 gipBulletPhysics::getOrigin2d(int gameObjectNo) {
	btCollisionObject* gameobject = getCollisionObjectArray()[gameObjectNo];
	btTransform transform = gameobject->getWorldTransform();
	return glm::vec2 (
			transform.getOrigin().getX(),
			transform.getOrigin().getY()
	);
}

glm::vec2 gipBulletPhysics::getCircle2dObjectPosition(int gameObjectNo, gImageGameObject* imgObject) {
	// gLogi("circle (x,y)") << trans.getOrigin().getX() - (imgWidth / 2) << " " << -(trans.getOrigin().getY() + imgHeight / 2);
	btCollisionObject* gameobject = getCollisionObjectArray()[gameObjectNo];
	btTransform transform = gameobject->getWorldTransform();
	return glm::vec2 (
			transform.getOrigin().getX() - (imgObject->image.getWidth() / 2),
			-(transform.getOrigin().getY() + imgObject->image.getHeight() / 2)
	);
}

glm::vec2 gipBulletPhysics::getBox2dObjectPosition(int gameObjectNo, gImageGameObject* imgObject) {
	// gLogi("box (x,y)") << trans.getOrigin().getX() << " " << -(trans.getOrigin().getY() + imgHeight);
	btCollisionObject* gameobject = getCollisionObjectArray()[gameObjectNo];
	btTransform transform = gameobject->getWorldTransform();
	return glm::vec2 (
			transform.getOrigin().getX(),
			-(transform.getOrigin().getY() + imgObject->image.getHeight())
	);
}

void gipBulletPhysics::clean() {
	delete collisionconfiguration;
	delete dispatcher;
	delete overlappingpaircache;
	delete solver;
	delete dynamicsworld;
}


