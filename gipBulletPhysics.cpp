/*
 *  gipBulletPhysics.cpp
 *
 *  Created on: 5 Aug 2022
 *      Author: Faruk Aygün
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
	gLogi();
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

/**@brief Return the origin vector translation */
void gipBulletPhysics::getOrigin(btTransform* trans) {
	trans->getOrigin();
}

// Delete initialized objects
void gipBulletPhysics::remove() {
	delete collisionConfiguration;
	delete dispatcher;
	delete overlappingPairCache;
	delete solver;
	delete dynamicsWorld;
}


