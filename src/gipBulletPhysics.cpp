/*
 *  gipBulletPhysics.cpp
 *
 *  Edited 		: 16.02.2023
 *  	Author 	: Remzi ISCI
 */

#include "gipBulletPhysics.h"
#include "gipDebugDraw.h"
#include "gipBaseGameObject.h"

gipBulletPhysics::gipBulletPhysics(WORLDCOORDINATETYPE worldcoordinate, WORLDTYPE worldType) {
    objectidcounter = 0;
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
    //   this->_maxsubsteps = worldcoordinate == WORLDCOORDINATETYPE::WORLD2D ? 10 : 1;
}

/*
 * Set owner target layer
 * Set target layers whic you want collide with owner object
 * 0 means dont collide
 */
ObjectId gipBulletPhysics::addPhysicObject(gipBaseGameObject* targetobject, int objectlayer, int masklayer) {
    targetobject->_id = objectidcounter++;
    _objectlist[targetobject->_id] = targetobject;
	if(targetobject->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
        _dynamicsworld->addRigidBody(targetobject->getRigidBody(), objectlayer, masklayer);
    } else {
        _dynamicsworld->addCollisionObject(targetobject->_ghostobject, objectlayer, masklayer);
    }

	return targetobject->_id;
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
	        	result->hittedobject = _objectlist[hitBody->getUserIndex()];
	        	gLogi("raycast") << "ray hitted";
	        	return true;
	        }

	    }

	    return false;

}

gipBaseGameObject* gipBulletPhysics::getObject(ObjectId id) {
	return this->_objectlist[id];
}

// need to be called from game each update
void gipBulletPhysics::runPhysicWorldStep(float deltatime) {
	// Physics calculations doing here.
	_dynamicsworld->stepSimulation(deltatime, 10);
	// update objects position
	for (auto& entry : this->_objectlist) {
        ObjectId id = entry.first;
        gipBaseGameObject* object = entry.second;
		// don't update if object is static rb.
		if (object->_collsionobjecttype == COLLISIONOBJECTTYPE_RIGIDBODY && !object->_rigidbody->isStaticObject()) {
			object->updatePositionVariable();
			object->updateRotationVariable();
		}
	}


	//Call cehck collision for each frame
	checkCollisions();
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

        for (int j = 0; j < numContacts; j++) {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            if (pt.getDistance() <= 0.0f) {
              //Collision positions
            	const btVector3& ptA = pt.getPositionWorldOnA();
                const btVector3& ptB = pt.getPositionWorldOnB();
                glm::vec3 colonobjA = glm::vec3((float)(ptA.x()), (float)(ptA.y()), (float)(ptA.z()));
                glm::vec3 colonobjB = glm::vec3((float)(ptB.x()), (float)(ptB.y()), (float)(ptB.z()));
              //  const btVector3& normalOnB = pt.m_normalWorldOnB;
                _objectlist[contactManifold->getBody0()->getUserIndex()]->warnCollided(contactManifold->getBody1()->getUserIndex(), colonobjA, colonobjB);
                _objectlist[contactManifold->getBody1()->getUserIndex()]->warnCollided(contactManifold->getBody0()->getUserIndex(), colonobjB, colonobjA);
                if (contactManifold->getNumContacts() != numContacts) {
                    break; // break if an object is removed from the list
                }
            }
        }
    }
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

void gipBulletPhysics::updateSingleAabb(ObjectId id) {
	if(_objectlist[id]->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
        _dynamicsworld->updateSingleAabb(_objectlist[id]->_rigidbody);
    } else {
        _dynamicsworld->updateSingleAabb(_objectlist[id]->_ghostobject);
    }
}

/*
 * Call this function for setting object mask layers
 * LAYER0 means dont collide
 */
void gipBulletPhysics::updateObjectlayers(ObjectId objectid) {
	_dynamicsworld->removeRigidBody(_objectlist[objectid]->_rigidbody);
	if(_objectlist[objectid]->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
        _dynamicsworld->addRigidBody(_objectlist[objectid]->_rigidbody, (int)_objectlist[objectid]->_objectlayers, (int)_objectlist[objectid]->_masklayers);
    } else {
        _dynamicsworld->addCollisionObject(_objectlist[objectid]->_ghostobject, (int)_objectlist[objectid]->_objectlayers, (int)_objectlist[objectid]->_masklayers);
    }
}

void gipBulletPhysics::setMass(gipBaseGameObject* targetobject, float newmass) {
	if(targetobject->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
		_dynamicsworld->removeRigidBody(targetobject->_rigidbody);
		btVector3 _interna = btVector3(0.0f, 0.0f, 0.0f);
		targetobject->_rigidbody->getCollisionShape()->calculateLocalInertia(newmass, _interna);
		targetobject->_rigidbody->setMassProps(newmass, _interna);

		if(newmass != 0.0f) {
			targetobject->_rigidbody->setFlags(targetobject->_rigidbody->getFlags() & !btCollisionObject::CF_STATIC_OBJECT);
			targetobject->_isstatic = false;
		} else {
			targetobject->_rigidbody->setFlags(targetobject->_rigidbody->getFlags() | btCollisionObject::CF_STATIC_OBJECT);
			targetobject->_isstatic = true;
		}

		_dynamicsworld->addRigidBody(targetobject->_rigidbody, (int)targetobject->_objectlayers, (int)targetobject->_masklayers);
	}
}

void gipBulletPhysics::printObjectTransform() {
	std::vector<btCollisionObject*> colobjarray;
	std::vector<btRigidBody*> rbarray;

	for(auto& entry : this->_objectlist) {
        ObjectId id = entry.first;
        gipBaseGameObject* object = entry.second;
		btTransform transform;
		btRigidBody* rb = object->getRigidBody();
		if (rb && rb->getMotionState()) {
			rb->getMotionState()->getWorldTransform(transform);
		} else {
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

void gipBulletPhysics::removeObject(ObjectId id) {
	if(_objectlist[id]->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
        _dynamicsworld->removeCollisionObject(_objectlist[id]->getRigidBody());
    } else {
        _dynamicsworld->removeCollisionObject(_objectlist[id]->_ghostobject);
    }
    _objectlist.erase(id);
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
	for (int j = 0; j < _objectlist.size(); j++) {
		gipBaseGameObject* gameobject = _objectlist[j];
		delete gameobject;
	}
	_objectlist.clear();

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
