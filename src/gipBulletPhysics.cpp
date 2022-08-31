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

void gipBulletPhysics::initializeRigidWorld() {
	collisionconfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionconfiguration);
	overlappingpaircache = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver;
	dynamicsworld = new btDiscreteDynamicsWorld (dispatcher, overlappingpaircache, solver, collisionconfiguration);
}

void gipBulletPhysics::initializeSoftRigidWorld() {
	collisionconfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionconfiguration);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver;
	constraintsolver = solver;
	dynamicsworld = new btDiscreteDynamicsWorld(dispatcher, broadphase, constraintsolver, collisionconfiguration);

	dynamicsworld->getSolverInfo().m_erp2 = 0.f;
	dynamicsworld->getSolverInfo().m_globalCfm = 0.f;
	dynamicsworld->getSolverInfo().m_numIterations = 3;
	dynamicsworld->getSolverInfo().m_solverMode = SOLVER_SIMD;  // | SOLVER_RANDMIZE_ORDER;
	dynamicsworld->getSolverInfo().m_splitImpulse = false;
}

void gipBulletPhysics::setGravity(glm::vec3 gravityValue) {
	dynamicsworld->setGravity(btVector3 (gravityValue.x, gravityValue.y, gravityValue.z));
}

void gipBulletPhysics::setCentralForce(gImageGameObject* imgObject, glm::vec3 forceValue) {
	btCollisionObject* gameobject = getCollisionObjectArray()[imgObject->id];
	btRigidBody* rb = btRigidBody::upcast(gameobject);

	rb->applyCentralForce(btVector3(forceValue.x, forceValue.y, forceValue.z));
}

void gipBulletPhysics::setCentralImpulse(gImageGameObject* imgObject, glm::vec3 impulseValue) {
	btCollisionObject* gameobject = getCollisionObjectArray()[imgObject->id];
	btRigidBody* rb = btRigidBody::upcast(gameobject);

	rb->applyCentralImpulse(btVector3(impulseValue.x, impulseValue.y, impulseValue.z));
}

void gipBulletPhysics::setForce(gImageGameObject* imgObject, glm::vec3 forceValue, glm::vec3 relPos) {
	btCollisionObject* gameobject = getCollisionObjectArray()[imgObject->id];
	btRigidBody* rb = btRigidBody::upcast(gameobject);

	rb->applyForce(btVector3(forceValue.x, forceValue.y, forceValue.z), btVector3(relPos.x, relPos.y, relPos.z));
}

void gipBulletPhysics::setImpulse(gImageGameObject* imgObject, glm::vec3 impulseValue, glm::vec3 relPos) {
	btCollisionObject* gameobject = getCollisionObjectArray()[imgObject->id];
	btRigidBody* rb = btRigidBody::upcast(gameobject);

	rb->applyImpulse(btVector3(impulseValue.x, impulseValue.y, impulseValue.z), btVector3(relPos.x, relPos.y, relPos.z));
}

void gipBulletPhysics::setTorque(gImageGameObject* imgObject, glm::vec3 torqueValue) {
	btCollisionObject* gameobject = getCollisionObjectArray()[imgObject->id];
	btRigidBody* rb = btRigidBody::upcast(gameobject);

	rb->applyTorque(btVector3(torqueValue.x, torqueValue.y, torqueValue.z));
}

void gipBulletPhysics::setTorqueImpulse(gImageGameObject* imgObject, glm::vec3 torqueValue) {
	btCollisionObject* gameobject = getCollisionObjectArray()[imgObject->id];
	btRigidBody* rb = btRigidBody::upcast(gameobject);

	rb->applyTorqueImpulse(btVector3(torqueValue.x, torqueValue.y, torqueValue.z));
}

int gipBulletPhysics::createBox2dObject(gImageGameObject* imgObject) {
	btTransform box2dtransform;
	btCollisionShape* box2dshape = new btBoxShape(btVector3(imgObject->image.getWidth(), imgObject->image.getHeight(), 1.0f));
	// TODO: btBox2dShape* box2dshape = new btBox2dShape(btVector3(imgObject->image.getWidth(), imgObject->image.getHeight(), 0.0f));
	collisionshapes.push_back(box2dshape);

	box2dtransform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 */
	box2dtransform.setOrigin(
			btVector3(
					imgObject->positionx,
					-(imgObject->positiony + imgObject->image.getHeight()),
					0
			)
	);

	btScalar mass(imgObject->mass);
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		box2dshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(box2dtransform);
	btRigidBody::btRigidBodyConstructionInfo box2drbinfo(mass, mymotionstate, box2dshape, localInertia);
	btRigidBody* box2drigidbody = new btRigidBody(box2drbinfo);

	dynamicsworld->addRigidBody(box2drigidbody);

	gameobjectid.push_back(gameobjectid.size());
	// gLogi("box") << float(box2dTransform.getOrigin().getX()) << " " << float(box2dTransform.getOrigin().getY());

	return gameobjectid.back();
}

int gipBulletPhysics::createCircle2dObject(gImageGameObject* imgObject) {
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
			btVector3(
					imgObject->positionx + (imgObject->image.getWidth() / 2),
					-(imgObject->positiony + imgObject->image.getHeight() / 2),
					0
			)
	);

	btScalar mass(imgObject->mass);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isdynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isdynamic)
		circle2dshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(circle2dtransform);
	btRigidBody::btRigidBodyConstructionInfo circle2drbinfo(mass, mymotionstate, circle2dshape, localInertia);
	btRigidBody* circle2drigidbody = new btRigidBody(circle2drbinfo);

	dynamicsworld->addRigidBody(circle2drigidbody);

	gameobjectid.push_back(gameobjectid.size());
	// gLogi("circle") << float(circle2dTransform.getOrigin().getX()) << " " << float(circle2dTransform.getOrigin().getY());

	return gameobjectid.back();
}

// increase stiffness, reduce dumping for harder floor.
int gipBulletPhysics::createSoftContactBox2dObject(gImageGameObject* imgObject, float stiffness, float damping) {
	btTransform softbox2dtransform;
	btCollisionShape* softbox2dshape = new btBoxShape(btVector3(imgObject->image.getWidth(), imgObject->image.getHeight(), 0.0f));

	collisionshapes.push_back(softbox2dshape);

	softbox2dtransform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 */
	softbox2dtransform.setOrigin(
			btVector3(
					imgObject->positionx,
					-(imgObject->positiony + imgObject->image.getHeight()),
					0
			)
	);

	btScalar mass(imgObject->mass);
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		softbox2dshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(softbox2dtransform);
	btRigidBody::btRigidBodyConstructionInfo softbox2drbinfo(mass, mymotionstate, softbox2dshape, localInertia);
	btRigidBody* softbox2drigidbody = new btRigidBody(softbox2drbinfo);

	dynamicsworld->addRigidBody(softbox2drigidbody);

	softbox2drigidbody->setContactStiffnessAndDamping(stiffness, damping);
	softbox2drigidbody->setFriction(1);

	gameobjectid.push_back(gameobjectid.size());
	// gLogi("soft contact box") << float(softbox2dtransform.getOrigin().getX()) << " " << float(softbox2dtransform.getOrigin().getY());

	return gameobjectid.back();
}

int gipBulletPhysics::createSoftCircle2dObject(gImageGameObject* imgObject) {
	btCollisionShape* softball2dchildshape = new btSphereShape(imgObject->image.getWidth() / 2); // child shape
	btCompoundShape* softball2dcolshape = new btCompoundShape(); // parent shape

	softball2dcolshape->addChildShape(btTransform::getIdentity(), softball2dchildshape);
	collisionshapes.push_back(softball2dcolshape);

	btTransform softball2dTransform;
	softball2dTransform.setIdentity();

	// ballTransform.setRotation(btQuaternion(btVector3(1.0f, 1.0f, 1.0f), SIMD_PI / 10.0));
	btScalar mass(imgObject->mass);
	bool isDynamic = (mass != 0.0f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		softball2dcolshape->calculateLocalInertia(mass, localInertia);

	for (int k = 0; k < 1; k++)
	{
		for (int i = 0; i < 1; i++)
		{
			for (int j = 0; j < 1; j++)
			{
				/*
				 * The Glist Engine references the top left corner for object positions;
				 * but the bullet3 library references the bottom left for object positions.
				 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
				 */
				softball2dTransform.setOrigin(
						btVector3(
								imgObject->positionx + imgObject->image.getWidth() / 2,
								-(imgObject->positiony + imgObject->image.getHeight() / 2),
								0
						)
				);

				bool isDynamic = (mass != 0.f);

				btVector3 localInertia(0, 0, 0);
				if (isDynamic)
					softball2dcolshape->calculateLocalInertia(mass, localInertia);

				//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* mymotionstate = new btDefaultMotionState(softball2dTransform);
				btRigidBody::btRigidBodyConstructionInfo softball2drbinfo(mass, mymotionstate, softball2dcolshape, localInertia);
				btRigidBody* softball2drb = new btRigidBody(softball2drbinfo);

				softball2drb->setUserIndex(-1);

				dynamicsworld->addRigidBody(softball2drb);
			}
		}
	}
	gameobjectid.push_back(gameobjectid.size());
	// gLogi("box") << float(softball2dTransform.getOrigin().getX()) << " " << float(softball2dTransform.getOrigin().getY());

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

glm::vec2 gipBulletPhysics::getCircle2dObjectPosition(gImageGameObject* imgObject) {
	// gLogi("circle (x,y)") << trans.getOrigin().getX() - (imgWidth / 2) << " " << -(trans.getOrigin().getY() + imgHeight / 2);
	btCollisionObject* gameobject = getCollisionObjectArray()[imgObject->id];
	btTransform transform = gameobject->getWorldTransform();
	return glm::vec2 (
			transform.getOrigin().getX() - (imgObject->image.getWidth() / 2),
			-(transform.getOrigin().getY() + imgObject->image.getHeight() / 2)
	);
}

glm::vec2 gipBulletPhysics::getBox2dObjectPosition(gImageGameObject* imgObject) {
	// gLogi("box (x,y)") << trans.getOrigin().getX() << " " << -(trans.getOrigin().getY() + imgHeight);
	btCollisionObject* gameobject = getCollisionObjectArray()[imgObject->id];
	btTransform transform = gameobject->getWorldTransform();
	return glm::vec2 (
			transform.getOrigin().getX(),
			-(transform.getOrigin().getY() + imgObject->image.getHeight())
	);
}

//cleanup in the reverse order of creation/initialization
void gipBulletPhysics::clean() {
	//remove the rigidbodies from the dynamics world and delete them
	if(dynamicsworld) {
		int i;
		for (i = dynamicsworld->getNumConstraints() - 1; i >= 0; i--) {
			dynamicsworld->removeConstraint(dynamicsworld->getConstraint(i));
		}
		for (i = dynamicsworld->getNumCollisionObjects() - 1; i >= 0; i--) {
			btCollisionObject* obj = getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState()) {
				delete body->getMotionState();
			}
			dynamicsworld->removeCollisionObject(obj);
			delete obj;
		}
	}

	// delete collision shapes
	for (int j = 0; j < collisionshapes.size(); j++)
	{
		btCollisionShape* shape = collisionshapes[j];
		delete shape;
	}
	collisionshapes.clear();

	delete dynamicsworld;
	dynamicsworld = 0;
	delete constraintsolver;
	constraintsolver = 0;
	delete solver;
	solver = 0;
	delete broadphase;
	broadphase = 0;
	delete overlappingpaircache;
	overlappingpaircache = 0;
	delete dispatcher;
	dispatcher = 0;
	delete collisionconfiguration;
	collisionconfiguration = 0;
}
