/*
 *  gipBulletPhysics.cpp
 *
 *  Created on: 5 Aug 2022
 *      Author: Faruk Aygun
 *      		Emirhan Limon
 *
 *  Edited 		: 16.02.2023
 *  	Author 	: Remzi ����
 */

#include "gipBulletPhysics.h"
#include "gDebugDraw.h"

gipBulletPhysics::gipBulletPhysics() {
}

gipBulletPhysics::~gipBulletPhysics() {
	clean();
}

void gipBulletPhysics::update() {
}

void gipBulletPhysics::initializeWorld(int type) {
	collisionconfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionconfiguration);
	overlappingpaircache = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver;
	gDebugDraw* debugDrawer = new gDebugDraw();

	if (type == rigidWorld) {
		dynamicsworld = new btDiscreteDynamicsWorld (dispatcher, overlappingpaircache, solver, collisionconfiguration);
	}
	else if (type == softRigidWorld) {
		broadphase = new btDbvtBroadphase();
		constraintsolver = solver;
		dynamicsworld = new btDiscreteDynamicsWorld(dispatcher, broadphase, constraintsolver, collisionconfiguration);
	}


    /*Create custom debug drawer*/
    gDebugDraw *draw   =   new gDebugDraw;
    draw->clearLines();
    draw->setDebugMode( draw->getDebugMode()
          | btIDebugDraw::DBG_DrawWireframe );

    dynamicsworld->setDebugDrawer(draw);
}

void gipBulletPhysics::setErp2(float value) {
	dynamicsworld->getSolverInfo().m_erp2 = value;
}

void gipBulletPhysics::setglobalCfm(float value) {
	dynamicsworld->getSolverInfo().m_globalCfm = value;

}

void gipBulletPhysics::setNumIterations(int value) {
	dynamicsworld->getSolverInfo().m_numIterations = value;

}

void gipBulletPhysics::setSolverMode(int solverMode) {
	dynamicsworld->getSolverInfo().m_solverMode = solverMode;  // | SOLVER_RANDMIZE_ORDER;
}

void gipBulletPhysics::setSplitImpulse(int splitImpulse) {
	dynamicsworld->getSolverInfo().m_splitImpulse = splitImpulse;
}

void gipBulletPhysics::setGravity(glm::vec3 gravityValue) {
	dynamicsworld->setGravity(btVector3 (gravityValue.x, gravityValue.y, gravityValue.z));
}

btRigidBody* gipBulletPhysics::getRigidBody(gImageGameObject* imgObject) {
	btCollisionObject* gameobject = dynamicsworld->getCollisionObjectArray()[imgObject->getId()];

	return btRigidBody::upcast(gameobject);
}

btTransform gipBulletPhysics::getTransform(int imgObjectId) {
	btCollisionObject* gameobject = dynamicsworld->getCollisionObjectArray()[imgObjectId];

	return gameobject->getWorldTransform();
}

void gipBulletPhysics::setFriction(gImageGameObject* imgObject, float frictionValue) {
	auto rb = getRigidBody(imgObject);
	rb->setFriction(frictionValue);
}

void gipBulletPhysics::setRollingFriction(gImageGameObject* imgObject, float frictionValue) {
	auto rb = getRigidBody(imgObject);
	rb->setRollingFriction(frictionValue);
}

void gipBulletPhysics::setSpinningFriction(gImageGameObject* imgObject, float frictionValue) {
	auto rb = getRigidBody(imgObject);
	rb->setSpinningFriction(frictionValue);
}

void gipBulletPhysics::setAnisotropicFriction(gImageGameObject* imgObject, glm::vec3 frictionValue, int frictionMode) {
	auto rb = getRigidBody(imgObject);
	rb->setAnisotropicFriction(btVector3(frictionValue.x, frictionValue.y, frictionValue.z), frictionMode);
}

void gipBulletPhysics::applyCentralForce(gImageGameObject* imgObject, glm::vec3 forceValue) {
	auto rb = getRigidBody(imgObject);
	rb->applyCentralForce(btVector3(forceValue.x, forceValue.y, forceValue.z));
}

void gipBulletPhysics::applyCentralImpulse(gImageGameObject* imgObject, glm::vec3 impulseValue) {
	auto rb = getRigidBody(imgObject);
	rb->applyCentralImpulse(btVector3(impulseValue.x, impulseValue.y, impulseValue.z));
}

void gipBulletPhysics::applyForce(gImageGameObject* imgObject, glm::vec3 forceValue, glm::vec3 relPos) {
	auto rb = getRigidBody(imgObject);
	rb->applyForce(btVector3(forceValue.x, forceValue.y, forceValue.z), btVector3(relPos.x, relPos.y, relPos.z));
}

void gipBulletPhysics::applyImpulse(gImageGameObject* imgObject, glm::vec3 impulseValue, glm::vec3 relPos) {
	auto rb = getRigidBody(imgObject);
	rb->applyImpulse(btVector3(impulseValue.x, impulseValue.y, impulseValue.z), btVector3(relPos.x, relPos.y, relPos.z));
}

void gipBulletPhysics::applyTorque(gImageGameObject* imgObject, glm::vec3 torqueValue) {
	auto rb = getRigidBody(imgObject);
	rb->applyTorque(btVector3(torqueValue.x, torqueValue.y, torqueValue.z));
}

void gipBulletPhysics::applyTorqueImpulse(gImageGameObject* imgObject, glm::vec3 torqueValue) {
	auto rb = getRigidBody(imgObject);
	rb->applyTorqueImpulse(btVector3(torqueValue.x, torqueValue.y, torqueValue.z));
}

float gipBulletPhysics::getErp2() {
	return dynamicsworld->getSolverInfo().m_erp2;
}

float gipBulletPhysics::getglobalCfm() {
	return dynamicsworld->getSolverInfo().m_globalCfm;
}

int gipBulletPhysics::getNumIterations() {
	return dynamicsworld->getSolverInfo().m_numIterations;
}

int gipBulletPhysics::getSolverMode() {
	return dynamicsworld->getSolverInfo().m_solverMode;
}

int gipBulletPhysics::getSplitImpulse() {
	return dynamicsworld->getSolverInfo().m_splitImpulse;
}

void gipBulletPhysics::printObjectTransform() {
	std::vector<btCollisionObject*> colobjarray;
	std::vector<btRigidBody*> rbarray;

	for (auto object : gameobjects) {
		int id = object->getId();
		// get created obj and add to vector
		colobjarray.push_back(dynamicsworld->getCollisionObjectArray()[id]);
		// create rb for obj and add to vector
		rbarray.push_back(btRigidBody::upcast(colobjarray[id]));

		btTransform transform;
		btRigidBody* rb = rbarray[id];

		if (rb && rb->getMotionState()) {
			rb->getMotionState()->getWorldTransform(transform);
		}
		else {
			transform = colobjarray[id]->getWorldTransform();
		}

		if (id == 1) {
			gLogi("gipBulletPhysics") << "id: " << id
					<< "\n\t\t\t pos(x,y,z): "
					<< " (" << float(transform.getOrigin().getX())
					<< ", " << float(transform.getOrigin().getY())
					<< ", " << float(transform.getOrigin().getZ()) << ")"
					<< "\n\t\t\t rot(x,y,z,w): "
					<< " (" << float(transform.getRotation().getX())
					<< ", " << float(transform.getRotation().getY())
					<< ", " << float(transform.getRotation().getZ())
					<< ", " << float(transform.getRotation().getW()) << ")"
					<< "\n\t\t\t radian angle: "
					<< transform.getRotation().getAngle()
					<< "\n\t\t\t degree angle: "
					<< gRadToDeg(transform.getRotation().getAngle())
					<< "\n";
		}
	}
}
/*
 * rotation is degree format
 */
int gipBulletPhysics::createBox2dObject(gImageGameObject* imgObject, float rotation, glm::vec2 size) {
	btTransform box2dtransform;
	//You need divide source size by 2 for convertin physic metrics correctly
	btCollisionShape* box2dshape = new btBoxShape(btVector3(imgObject->getWidth() * 0.5f * size.x, imgObject->getHeight() * 0.5f * size.y, 0.0f));
	collisionshapes.push_back(box2dshape);

	box2dtransform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 * You need set collision center to upright of half size of source
	 */
	box2dtransform.setOrigin(
			btVector3(
					imgObject->getPosition().x + imgObject->getWidth() / 2,
					-(imgObject->getPosition().y + imgObject->getHeight() / 2),
					0
			)
	);
	if(rotation > 0.0f) {
		btQuaternion objQuat = box2dtransform.getRotation();
		objQuat.setRotation(btVector3(0.0f, 0.0f, 1.0f),-glm::radians(rotation));
		box2dtransform.setRotation(objQuat);

	}
	btScalar mass(imgObject->getMass());
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		box2dshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(box2dtransform);
	btRigidBody::btRigidBodyConstructionInfo box2drbinfo(mass, mymotionstate, box2dshape, localInertia);
	btRigidBody* box2drigidbody = new btRigidBody(box2drbinfo);

	box2drigidbody->setCollisionFlags(box2drigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

	dynamicsworld->addRigidBody(box2drigidbody);

	gameobjects.push_back(imgObject);
	imgObject->setId(gameobjects.size() - 1);
	box2drigidbody->setUserIndex(imgObject->getId());
	// gLogi("box") << float(box2dTransform.getOrigin().getX()) << " " << float(box2dTransform.getOrigin().getY());

	return imgObject->getId();
}

int gipBulletPhysics::createCircle2dObject(gImageGameObject* imgObject, float rotation, float radius) {
	btTransform circle2dtransform;
	/*  parameter is circle radius
	*   You need divide source size by 2 for convertin physic metrics correctly
	*	Half size of source equal to diameter of collision object
	*/
	btCollisionShape* circle2dshape = new btSphereShape(imgObject->getWidth() * 0.5f * radius);
	collisionshapes.push_back(circle2dshape);

	circle2dtransform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the center for circle object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getWidth() / 2 and +img.getHeight() / 2).
	 */
	circle2dtransform.setOrigin(
			btVector3(
					imgObject->getPosition().x + (imgObject->getWidth() / 2),
					-(imgObject->getPosition().y + imgObject->getHeight() / 2),
					0
			)
	);
	if(rotation > 0.0f) {
		btQuaternion objQuat = circle2dtransform.getRotation();
		objQuat.setRotation(btVector3(0.0f, 0.0f, 1.0f),-glm::radians(rotation));
		circle2dtransform.setRotation(objQuat);

	}
	btScalar mass(imgObject->getMass());

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isdynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isdynamic)
		circle2dshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(circle2dtransform);
	btRigidBody::btRigidBodyConstructionInfo circle2drbinfo(mass, mymotionstate, circle2dshape, localInertia);
	btRigidBody* circle2drigidbody = new btRigidBody(circle2drbinfo);


	circle2drigidbody->setCollisionFlags(circle2drigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

	dynamicsworld->addRigidBody(circle2drigidbody);

	gameobjects.push_back(imgObject);
	imgObject->setId(gameobjects.size() - 1);
	circle2drigidbody->setUserIndex(imgObject->getId());
	// gLogi("circle") << float(circle2dTransform.getOrigin().getX()) << " " << float(circle2dTransform.getOrigin().getY());

	return imgObject->getId();
}

int gipBulletPhysics::createSoftContactBox2dObject(gImageGameObject* imgObject, float stiffness, float damping, float rotation, glm::vec2 size) {
	btTransform softbox2dtransform;

	btCollisionShape* softbox2dshape = new btBoxShape(btVector3(imgObject->getWidth() * 0.5f * size.x, imgObject->getHeight() * 0.5f * size.y, 0.0f));
	collisionshapes.push_back(softbox2dshape);

	softbox2dtransform.setIdentity();


	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 */
	softbox2dtransform.setOrigin(
			btVector3(
					imgObject->getPosition().x + imgObject->getWidth() * 0.5f,
					-(imgObject->getPosition().y + imgObject->getHeight() * 0.5f),
					0
			)
	);

	if(rotation > 0.0f) {
		btQuaternion objQuat = softbox2dtransform.getRotation();
		//objQuat.setRotation(btVector3(0.0f,0.0f,1.0f), -glm::radians(rotation));
		//objQuat.setEuler(0.0f, 0.0f, -glm::radians(rotation));
		objQuat.setRotation(btVector3(0.0f, 0.0f, 1.0f),-glm::radians(rotation));
		softbox2dtransform.setRotation(objQuat);

	}


	btScalar mass(imgObject->getMass());
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);



	if (isDynamic)
		softbox2dshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(softbox2dtransform);
	btRigidBody::btRigidBodyConstructionInfo softbox2drbinfo(mass, mymotionstate, softbox2dshape, localInertia);
	btRigidBody* softbox2drigidbody = new btRigidBody(softbox2drbinfo);




	softbox2drigidbody->setCollisionFlags(softbox2drigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	dynamicsworld->addRigidBody(softbox2drigidbody);

	softbox2drigidbody->setContactStiffnessAndDamping(stiffness, damping);

	gameobjects.push_back(imgObject);
	imgObject->setId(gameobjects.size() - 1);
	softbox2drigidbody->setUserIndex(imgObject->getId());
	// gLogi("soft contact box") << float(softbox2dtransform.getOrigin().getX()) << " " << float(softbox2dtransform.getOrigin().getY());

	return imgObject->getId();
}

int gipBulletPhysics::createSoftCircle2dObject(gImageGameObject* imgObject,float rotation, float radius) {
	btCollisionShape* softball2dcolshape = new btSphereShape(imgObject->getWidth() * 0.5f * radius); // child shape
	//btCompoundShape* softball2dcolshape = new btCompoundShape(); // parent shape

	//softball2dcolshape->addChildShape(btTransform::getIdentity(), softball2dchildshape);
	collisionshapes.push_back(softball2dcolshape);

	btTransform softball2dTransform;
	softball2dTransform.setIdentity();

	// ballTransform.setRotation(btQuaternion(btVector3(1.0f, 1.0f, 1.0f), SIMD_PI / 10.0));
	btScalar mass(imgObject->getMass());
	bool isDynamic = (mass != 0.0f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		softball2dcolshape->calculateLocalInertia(mass, localInertia);

	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 */
	softball2dTransform.setOrigin(
			btVector3(
					imgObject->getPosition().x + imgObject->getWidth() / 2,
					-(imgObject->getPosition().y + imgObject->getHeight() / 2),
					0
			)
	);
	if(rotation > 0.0f) {
		btQuaternion objQuat = softball2dTransform.getRotation();
		objQuat.setRotation(btVector3(0.0f, 0.0f, 1.0f),-glm::radians(rotation));
		softball2dTransform.setRotation(objQuat);

	}
	if (isDynamic)
		softball2dcolshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(softball2dTransform);
	btRigidBody::btRigidBodyConstructionInfo softball2drbinfo(mass, mymotionstate, softball2dcolshape, localInertia);
	btRigidBody* softball2drb = new btRigidBody(softball2drbinfo);
	softball2drb->setCollisionFlags(softball2drb->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	dynamicsworld->addRigidBody(softball2drb);


	gameobjects.push_back(imgObject);
	imgObject->setId(gameobjects.size() - 1);
	softball2drb->setUserIndex(imgObject->getId());
	// gLogi("box") << float(softball2dTransform.getOrigin().getX()) << " " << float(softball2dTransform.getOrigin().getY());

	return imgObject->getId();
}

int gipBulletPhysics::stepSimulation(btScalar timeStep, int maxSubSteps , btScalar fixedTimeStep) {

	// Physics calculations doing here.
	int step = dynamicsworld->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);

	// update objects position
	for (auto object : gameobjects) {
		btCollisionObject* colobj = dynamicsworld->getCollisionObjectArray()[object->getId()];
		btTransform transform = colobj->getWorldTransform();
		btQuaternion rotation = transform.getRotation();
		/*
		 * Box = 0
		 * Sphere = 8
		 * Compound = 31
		 */
		int shapetype = colobj->getCollisionShape()->getShapeType();

		// don't update if object is static rb.
		if (!colobj->isStaticObject()) {
			if (shapetype == 0) { // box
				object->setPosition(getBox2dObjectPosition(object));
			} else if (shapetype == 8) { // sphere
				object->setPosition(getCircle2dObjectPosition(object));
			}
		}
		// update rotation
		object->setRotationAngle(gRadToDeg(-rotation.getAxis().getZ() * rotation.getAngle()));
	}

	//Call cehck collision for each frame
	collisionCallbackFunction();
	// For tests:
	//printObjectTransform();

	return step;
}

glm::vec2 gipBulletPhysics::getOrigin2d(int imgObjectId) {
	btTransform transform = getTransform(imgObjectId);

	return glm::vec2 (
			transform.getOrigin().getX(),
			transform.getOrigin().getY()
	);
}

glm::vec2 gipBulletPhysics::getCircle2dObjectPosition(gImageGameObject* imgObject) {
	// gLogi("circle (x,y)") << trans.getOrigin().getX() - (imgWidth / 2) << " " << -(trans.getOrigin().getY() + imgHeight / 2);
	btTransform transform = getTransform(imgObject->getId());

	return glm::vec2 (
			transform.getOrigin().getX() - (imgObject->getWidth() / 2),
			-(transform.getOrigin().getY() + imgObject->getHeight() / 2)
	);
}

glm::vec2 gipBulletPhysics::getBox2dObjectPosition(gImageGameObject* imgObject) {
	// gLogi("box (x,y)") << trans.getOrigin().getX() << " " << -(trans.getOrigin().getY() + imgHeight);
	btTransform transform = getTransform(imgObject->getId());

	return glm::vec2 (
			transform.getOrigin().getX(),
			-(transform.getOrigin().getY() + imgObject->getHeight())
	);
}

glm::vec3 gipBulletPhysics::get2dObjectRotation(gImageGameObject* imgObject) {
	btTransform transform = getTransform(imgObject->getId());
	btQuaternion rotation = transform.getRotation();

	return glm::vec3 (
			rotation.getX(),
			rotation.getY(),
			rotation.getZ()
	);
}

void gipBulletPhysics::collisionCallbackFunction() {

    //Count of collision
    int numManifolds = dynamicsworld->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
        btPersistentManifold* contactManifold =  dynamicsworld->getDispatcher()->getManifoldByIndexInternal(i);


        int numContacts = contactManifold->getNumContacts();
        for (int j=0;j<numContacts;j++)
        {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            if (pt.getDistance() < 0.f)
            {
            	//Collision positions
              //  const btVector3& ptA = pt.getPositionWorldOnA();
              //  const btVector3& ptB = pt.getPositionWorldOnB();
              //  const btVector3& normalOnB = pt.m_normalWorldOnB;

            	gameobjects[contactManifold->getBody0()->getUserIndex()]->onCollided(contactManifold->getBody1()->getUserIndex());
            	gameobjects[contactManifold->getBody1()->getUserIndex()]->onCollided(contactManifold->getBody0()->getUserIndex());
            }
        }
    }


}

void gipBulletPhysics::drawDebug(){
	dynamicsworld->debugDrawWorld();
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
			btCollisionObject* obj = dynamicsworld->getCollisionObjectArray()[i];
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

	//delete gameobjects
	for (int j = 0; j < gameobjects.size(); j++)
	{
		gImageGameObject* gameobject = gameobjects[j];
		delete gameobject;
	}
	gameobjects.clear();

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
