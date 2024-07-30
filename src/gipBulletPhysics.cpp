/*
 *  gipBulletPhysics.cpp
 *
 *  Edited 		: 16.02.2023
 *  	Author 	: Remzi ISCI
 */

#include "gipBulletPhysics.h"
#include "gipDebugDraw.h"
#include "gipBaseGameObject.h"

gipBulletPhysics* gipBulletPhysics::plugin;

gipBulletPhysics::gipBulletPhysics(WORLDCOORDINATETYPE worldcoordinate, WORLDTYPE worldType) {
	plugin = this;
	initializeWorld(worldcoordinate, worldType);
}

gipBulletPhysics::~gipBulletPhysics() {
	clean();
}

void gipBulletPhysics::initializeWorld(WORLDCOORDINATETYPE worldcoordinate, WORLDTYPE worldType) {
	if(_isworldinitiliazed) {
        return;
	}
    collisionconfiguration = new btDefaultCollisionConfiguration();
    collisiondispatcher = new btCollisionDispatcher(collisionconfiguration);
    overlappingpaircache = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver;

    if (worldType == WORLDTYPE::WORLDTYPE_RIGIDWORLD) {
        _dynamicsworld = new btDiscreteDynamicsWorld (collisiondispatcher, overlappingpaircache, solver, collisionconfiguration);
    }
    else if (worldType == WORLDTYPE::WORLDTYPE_SOFTWORLD) {
        broadphase = new btDbvtBroadphase();
        constraintsolver = solver;
        _dynamicsworld = new btDiscreteDynamicsWorld(collisiondispatcher, broadphase, constraintsolver, collisionconfiguration);
    }

    /*
     * THis property is for fast object, When seeting this property false then fast object can pass through wall
     * When choosin true will gice better collision dtetection but will cost more performance
     */
    _dynamicsworld->getDispatchInfo().m_useContinuous = true;

    /*
     *2D coordinate and 3D coordinate have different y axis way
     */
    _dynamicsworld->setGravity(btVector3(0.0f, worldcoordinate == WORLDCOORDINATETYPE::WORLD2D ? -9.81f : 9.81f, 0.0f));
    _dynamicsworld->applyGravity();

    _dynamicsworld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

    /*Create custom debug drawer*/
    gipDebugDraw *draw   = new gipDebugDraw((int)worldcoordinate);
    draw->clearLines();
    draw->setDebugMode( draw->getDebugMode()
                        | btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE);

    _dynamicsworld->setDebugDrawer(draw);

    _isworldinitiliazed = true;

	_dynamicsworld->setInternalTickCallback(internalTick);
    //   this->_maxsubsteps = worldcoordinate == WORLDCOORDINATETYPE::WORLD2D ? 10 : 1;
}

/*
 * Set owner target layer
 * Set target layers whic you want collide with owner object
 * 0 means dont collide
 */
void gipBulletPhysics::addPhysicObject(gipBaseGameObject* targetobject, int objectlayer, int masklayer) {
	if(targetobject->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
        _dynamicsworld->addRigidBody(targetobject->getRigidBody(), objectlayer, masklayer);
		targetobject->getRigidBody()->setUserPointer(targetobject);
    } else {
        _dynamicsworld->addCollisionObject(targetobject->_ghostobject, objectlayer, masklayer);
		targetobject->_ghostobject->setUserPointer(targetobject);
    }
	targetobject->setSelfIndex(_objects.size());
	_objects.push_back(targetobject);
}

//This function doesnt work, need to rewrite, use gGhostGameObject3D or gGhostGameObject2D for ray
bool gipBulletPhysics::raycastHit(glm::vec3 from, glm::vec3 to, int masklayers, gipRaycastResult* result) {
	gLogi("raycast ") << "called";

	btVector3 _start = btVector3(from.x, from.y, from.z);
	btVector3 _end = btVector3(to.x, to.y, to.z);
    btCollisionWorld::AllHitsRayResultCallback rayCallback(_start, _end);


    //For debug ray
 //   _dynamicsworld->getDebugDrawer()->drawLine(_start, _end, btVector4(1, 1, 0, 1));
 //   _dynamicsworld->getDebugDrawer()->drawSphere(_start, 20.f, btVector4(1, 1, 0, 1));

    rayCallback.m_flags |= btTriangleRaycastCallback::kF_KeepUnflippedNormal;
    rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseSubSimplexConvexCastRaytest;
    rayCallback.m_flags &= ~btTriangleRaycastCallback::kF_FilterBackfaces;
	_dynamicsworld->performDiscreteCollisionDetection();
	_dynamicsworld->updateAabbs();
	overlappingpaircache->calculateOverlappingPairs(collisiondispatcher);

	// Set the collision filter for the raycast callback
	     rayCallback.m_collisionFilterGroup = 0;  // Raycast is in its own group
	    rayCallback.m_collisionFilterMask = masklayers;  // Check collisions with specified layer mask

	 // Perform the raycast on the dynamics world

	_dynamicsworld->rayTest(_start, _end, rayCallback);
	_dynamicsworld->performDiscreteCollisionDetection();
	_dynamicsworld->updateAabbs();
	overlappingpaircache->calculateOverlappingPairs(collisiondispatcher);

	    // Check if the raycast hit anything
	    if (rayCallback.hasHit()) {
	        // Get the hit object and point
	        const btRigidBody* hitBody = btRigidBody::upcast(rayCallback.m_collisionObjects[0]);
	        btVector3 hitPoint;// = rayCallback.m_hitPointWorld;
	       	gLogi("raycast") << "ray hitted";
	        // Do something with the hit object and point (e.g., apply a force to the object)
	        if (hitBody != nullptr) {

	        	glm::vec3 hp = glm::vec3(hitPoint.x(), hitPoint.y(), hitPoint.z());
	        	result->hitpoint = hp;
	        	result->hittedobject = static_cast<gipBaseGameObject*>(hitBody->getUserPointer());
	        	gLogi("raycast") << "ray hitted";
	        	return true;
	        }

	    }

	    return false;

}

// need to be called from game each update
void gipBulletPhysics::runPhysicWorldStep(float deltatime) {
	// Physics calculations doing here.
	_dynamicsworld->stepSimulation(deltatime, 0);
	// update objects position
	for (auto& object : this->_objects) {
		// don't update if object is static rb.
		if (object->_collsionobjecttype == COLLISIONOBJECTTYPE_RIGIDBODY && !object->_rigidbody->isStaticObject()) {
			object->updatePositionVariable();
			object->updateRotationVariable();
		}
	}
	// For tests:
	//printObjectTransform();
}

void gipBulletPhysics::checkCollisions() {
    //Count of collision
    int numManifolds = _dynamicsworld->getDispatcher()->getNumManifolds();

    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold =  _dynamicsworld->getDispatcher()->getManifoldByIndexInternal(i);
        //Count of point between collided two objects
        int numContacts = contactManifold->getNumContacts();

        for (int j = numContacts - 1; j >= 0; j--) {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            //Collision positions
			const btVector3& ptA = pt.getPositionWorldOnA();
			const btVector3& ptB = pt.getPositionWorldOnB();
			glm::vec3 colonobjA = glm::vec3((float)(ptA.x()), (float)(ptA.y()), (float)(ptA.z()));
			glm::vec3 colonobjB = glm::vec3((float)(ptB.x()), (float)(ptB.y()), (float)(ptB.z()));
			//  const btVector3& normalOnB = pt.m_normalWorldOnB;
			gipBaseGameObject* obj1 = static_cast<gipBaseGameObject*>(contactManifold->getBody0()->getUserPointer());
			gipBaseGameObject* obj2 = static_cast<gipBaseGameObject*>(contactManifold->getBody1()->getUserPointer());
			obj1->warnCollided(obj2, colonobjA, colonobjB);
			obj2->warnCollided(obj1, colonobjB, colonobjA);
			break;
        }
    }
}

void gipBulletPhysics::internalTick(btDynamicsWorld *world, btScalar timeStep) {
	plugin->checkCollisions();
}

void gipBulletPhysics::setGravity(glm::vec3 gravityValue) {
	_dynamicsworld->setGravity(btVector3 (gravityValue.x, gravityValue.y, gravityValue.z));
	_dynamicsworld->applyGravity();
}

glm::vec3 gipBulletPhysics::getGravity() {
	btVector3 tempvec = _dynamicsworld->getGravity();
	return glm::vec3(tempvec.x(), tempvec.y(), tempvec.z());
}

/*
 * Call this function in the GamaCanvas draw method to draw physic bounds and debug views
 */
void gipBulletPhysics::drawDebug() {
	_dynamicsworld->debugDrawWorld();
}

void gipBulletPhysics::removeObject(gipBaseGameObject* object) {
	if (object->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
        _dynamicsworld->removeCollisionObject(object->getRigidBody());
    } else {
        _dynamicsworld->removeCollisionObject(object->_ghostobject);
    }
	size_t previousindex = object->_selfindex;
	_objects.erase(_objects.begin() + previousindex);
	for (size_t i = previousindex; i < _objects.size(); ++i) {
		_objects[i]->setSelfIndex(i);
	}
}

void gipBulletPhysics::setErp2(float value) {
	_dynamicsworld->getSolverInfo().m_erp2 = value;
}

void gipBulletPhysics::setglobalCfm(float value) {
	_dynamicsworld->getSolverInfo().m_globalCfm = value;

}

void gipBulletPhysics::setNumIterations(int value) {
	_dynamicsworld->getSolverInfo().m_numIterations = value;
}

void gipBulletPhysics::setSolverMode(int solverMode) {
	_dynamicsworld->getSolverInfo().m_solverMode = solverMode;  // | SOLVER_RANDMIZE_ORDER;
}

void gipBulletPhysics::setSplitImpulse(int splitImpulse) {
	_dynamicsworld->getSolverInfo().m_splitImpulse = splitImpulse;
}

float gipBulletPhysics::getErp2() {
	return _dynamicsworld->getSolverInfo().m_erp2;
}

float gipBulletPhysics::getglobalCfm() {
	return _dynamicsworld->getSolverInfo().m_globalCfm;
}

int gipBulletPhysics::getNumIterations() {
	return _dynamicsworld->getSolverInfo().m_numIterations;
}

int gipBulletPhysics::getSolverMode() {
	return _dynamicsworld->getSolverInfo().m_solverMode;
}

int gipBulletPhysics::getSplitImpulse() {
	return _dynamicsworld->getSolverInfo().m_splitImpulse;
}

//cleanup in the reverse order of creation/initialization
void gipBulletPhysics::clean() {
	//remove the rigidbodies from the dynamics world and delete them
	if(_dynamicsworld) {
		for (int i = _dynamicsworld->getNumConstraints() - 1; i >= 0; i--) {
			_dynamicsworld->removeConstraint(_dynamicsworld->getConstraint(i));
		}
		for (int i = _dynamicsworld->getNumCollisionObjects() - 1; i >= 0; i--) {
			btCollisionObject* obj = _dynamicsworld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState()) {
				delete body->getMotionState();
			}
			_dynamicsworld->removeCollisionObject(obj);
			delete obj;
		}
	}

	//delete gameobjects
	for (int j = 0; j < _objects.size(); j++) {
		gipBaseGameObject* gameobject = _objects[j];
		delete gameobject;
	}
	_objects.clear();

	delete _dynamicsworld;
	_dynamicsworld = nullptr;
	delete constraintsolver;
	constraintsolver = nullptr;
	delete solver;
	solver = nullptr;
	delete softwolrdbroadphase;
	softwolrdbroadphase = nullptr;
	delete overlappingpaircache;
	overlappingpaircache = nullptr;
	delete collisiondispatcher;
	collisiondispatcher = nullptr;
	delete collisionconfiguration;
	collisionconfiguration = nullptr;
}
