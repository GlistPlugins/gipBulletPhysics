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

void gipBulletPhysics::setGravity(float gravityvalue) {
	dynamicsworld->setGravity(btVector3 (0, gravityvalue, 0));
}

void gipBulletPhysics::addRigidBody(btRigidBody* rb) {
	dynamicsworld->addRigidBody(rb);
}

void gipBulletPhysics::create2dBoxObject(gImageGameObject* imgobject, float objmass) {
	btTransform box2dtransform;

	btCollisionShape* box2dshape = new btBoxShape(btVector3(imgobject->image.getWidth(), imgobject->image.getHeight(), 1.0f));
	collisionshapes.push_back(box2dshape);

	box2dtransform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 */
	box2dtransform.setOrigin(btVector3(imgobject->positionx, -(imgobject->positiony + imgobject->image.getHeight()), 0));
	btScalar mass(objmass);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		box2dshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(box2dtransform);
	btRigidBody::btRigidBodyConstructionInfo box2drbinfo(mass, mymotionstate, box2dshape, localInertia);
	btRigidBody* box2drigidbody = new btRigidBody(box2drbinfo);

	addRigidBody(box2drigidbody);

	// gLogi("box") << float(box2dTransform.getOrigin().getX()) << " " << float(box2dTransform.getOrigin().getY());
}

void gipBulletPhysics::create2dCircleObject(gImageGameObject* imgobject, float objmass) {
	btTransform circle2dtransform;

	// parameter is circle radius
	btCollisionShape* circle2dshape = new btSphereShape(imgobject->image.getWidth() / 2);
	collisionshapes.push_back(circle2dshape);

	circle2dtransform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the center for circle object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getWidth() / 2 and +img.getHeight() / 2).
	 */
	circle2dtransform.setOrigin(
			btVector3(imgobject->positionx + (imgobject->image.getWidth() / 2),
			-(imgobject->positiony + imgobject->image.getHeight() / 2), 0)
	);

	btScalar mass(objmass);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isdynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isdynamic)
		circle2dshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(circle2dtransform);
	btRigidBody::btRigidBodyConstructionInfo circle2drbinfo(mass, mymotionstate, circle2dshape, localInertia);
	btRigidBody* circle2drigidbody = new btRigidBody(circle2drbinfo);

	addRigidBody(circle2drigidbody);

	// gLogi("circle") << float(circle2dTransform.getOrigin().getX()) << " " << float(circle2dTransform.getOrigin().getY());
}

int gipBulletPhysics::stepSimulation(btScalar timestep, int maxsubsteps , btScalar fixedtimestep) {
	return dynamicsworld->stepSimulation(timestep, maxsubsteps, fixedtimestep);
}

int gipBulletPhysics::getNumCollisionObjects() {
	return dynamicsworld->getNumCollisionObjects();
}

btCollisionObjectArray& gipBulletPhysics::getCollisionObjectArray() {
	return dynamicsworld->getCollisionObjectArray();
}

glm::vec2 gipBulletPhysics::getOrigin2d(btTransform* trans) {
	btVector3& position = trans->getOrigin();
	return glm::vec2 (
			position.getX(),
			position.getY()
	);
}

// convert bullet3 positions to Glist Engine positions and return for circle objects.
glm::vec2 gipBulletPhysics::getCircle2dObjectPosition(btTransform transform, float imgwidth, float imgheight) {
	// gLogi("circle (x,y)") << trans.getOrigin().getX() - (imgWidth / 2) << " " << -(trans.getOrigin().getY() + imgHeight / 2);
	return glm::vec2 (
			transform.getOrigin().getX() - (imgwidth / 2),
			-(transform.getOrigin().getY() + imgheight / 2)
	);
}

// convert bullet3 positions to Glist Engine positions and return for box objects.
glm::vec2 gipBulletPhysics::getBox2dObjectPosition(btTransform trans, float imgwidth, float imgheight) {
	// gLogi("box (x,y)") << trans.getOrigin().getX() << " " << -(trans.getOrigin().getY() + imgHeight);
	return glm::vec2 (
			trans.getOrigin().getX(),
			-(trans.getOrigin().getY() + imgheight)
	);
}

void gipBulletPhysics::clean() {
	delete collisionconfiguration;
	delete dispatcher;
	delete overlappingpaircache;
	delete solver;
	delete dynamicsworld;
}


