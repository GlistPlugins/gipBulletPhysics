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
	initializeWorld(worldcoordinate, worldType);
}

gipBulletPhysics::~gipBulletPhysics() {
	clean();
}

void gipBulletPhysics::initializeWorld(WORLDCOORDINATETYPE worldcoordinate, WORLDTYPE worldType) {

	if(_isworldinitiliazed == false){
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
		_dynamicsworld->getDispatchInfo().m_useContinuous=true;

		/*
		 *2D coordinate and 3D coordinate have different y axis way
		 */
		_dynamicsworld->setGravity(btVector3(0.0f, worldcoordinate == WORLDCOORDINATETYPE::WORLD2D ? -9.81f : 9.81f, 0.0f));
		_dynamicsworld->applyGravity();
	    /*Create custom debug drawer*/
	    gipDebugDraw *draw   = new gipDebugDraw((int)worldcoordinate);
	    draw->clearLines();
	    draw->setDebugMode( draw->getDebugMode()
	          | btIDebugDraw::DBG_DrawWireframe );

	    _dynamicsworld->setDebugDrawer(draw);

	    _isworldinitiliazed = true;
	}

}

/*
 * Set owner target layer
 * Set target layers whic you want collide with owner object
 * 0 means dont collide
 */
int gipBulletPhysics::addPhysicObect(gipBaseGameObject* targetobject, int objectlayer, int masklayer) {
	this->_objectlist.push_back(targetobject);
	this->_dynamicsworld->addRigidBody(targetobject->getRigidBody(), objectlayer, masklayer);


	return this->_objectlist.size() - 1;
}

gipBaseGameObject* gipBulletPhysics::getObject(int id) {
	return this->_objectlist[id];
}

//need to be called from game each update
// Return step count fot world worked
int gipBulletPhysics::runPhysicWorldStep() {
	// Physics calculations doing here.
	int step = _dynamicsworld->stepSimulation(this->_timestep, this->_maxsubsteps, this->_fixedtimestep);
	// update objects position
	for (auto object : this->_objectlist) {

		// don't update if object is static rb.
		if (!object->_rigidbody->isStaticObject()) {
			object->updatePositionVariable();
			object->updateRotationVariable();

		}
	}

	//Call cehck collision for each frame
	checkCollisions();
	// For tests:
	//printObjectTransform();

	return step;
}

void gipBulletPhysics::checkCollisions() {
    //Count of collision
    int numManifolds = _dynamicsworld->getDispatcher()->getNumManifolds();

    for (int i=0;i<numManifolds;i++)
    {
        btPersistentManifold* contactManifold =  _dynamicsworld->getDispatcher()->getManifoldByIndexInternal(i);
        //Count of point between collided two objects
        int numContacts = contactManifold->getNumContacts();
        for (int j=0;j<numContacts;j++)
        {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            if (pt.getDistance() <= 0.f) //Maybe can use <=
            {
              //Collision positions
            	const btVector3& ptA = pt.getPositionWorldOnA();
                const btVector3& ptB = pt.getPositionWorldOnB();
                glm::vec3 colonobjA = glm::vec3((float)(ptA.x()), (float)(ptA.y()), (float)(ptA.z()));
                glm::vec3 colonobjB = glm::vec3((float)(ptB.x()), (float)(ptB.y()), (float)(ptB.z()));
              //  const btVector3& normalOnB = pt.m_normalWorldOnB;
                this->_objectlist[contactManifold->getBody0()->getUserIndex()]->warnCollided(contactManifold->getBody1()->getUserIndex(), colonobjA, colonobjB);
                this->_objectlist[contactManifold->getBody1()->getUserIndex()]->warnCollided(contactManifold->getBody0()->getUserIndex(), colonobjB, colonobjA);

            }
        }
    }
}

void gipBulletPhysics::setGravity(glm::vec3 gravityValue) {
	_dynamicsworld->setGravity(btVector3 (gravityValue.x, gravityValue.y, gravityValue.z));
	_dynamicsworld->applyGravity();
}

glm::vec3 gipBulletPhysics::getGravity() {
	btVector3 tempvec =  _dynamicsworld->getGravity();
	return glm::vec3(tempvec.x(), tempvec.y(), tempvec.z());
}

/*
 * Call this function in the GamaCanvas draw method to draw physic bounds and debug views
 */
void gipBulletPhysics::drawDebug() {
	_dynamicsworld->debugDrawWorld();
}

void gipBulletPhysics::updateSingleAabb(btCollisionObject* rigidbody) {
	_dynamicsworld->updateSingleAabb(rigidbody);
}

void gipBulletPhysics::setMass(gipBaseGameObject* targetobject, float newmass) {
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

void gipBulletPhysics::printObjectTransform() {
	std::vector<btCollisionObject*> colobjarray;
	std::vector<btRigidBody*> rbarray;

	for (auto object : this->_objectlist) {
		int id = object->getID();
		btTransform transform;
		btRigidBody* rb = object->getRigidBody();
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

void gipBulletPhysics::removeObject(int id) {
	_dynamicsworld->removeCollisionObject(_objectlist[id]->getRigidBody());
	_objectlist.erase(_objectlist.begin() + id);
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
		int i;
		for (i = _dynamicsworld->getNumConstraints() - 1; i >= 0; i--) {
			_dynamicsworld->removeConstraint(_dynamicsworld->getConstraint(i));
		}
		for (i = _dynamicsworld->getNumCollisionObjects() - 1; i >= 0; i--) {
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
	for (int j = 0; j < _objectlist.size(); j++)
	{
		gipBaseGameObject* gameobject = _objectlist[j];
		delete gameobject;
	}
	_objectlist.clear();

	delete _dynamicsworld;
	_dynamicsworld = 0;
	delete constraintsolver;
	constraintsolver = 0;
	delete solver;
	solver = 0;
	delete softwolrdbroadphase;
	softwolrdbroadphase = 0;
	delete overlappingpaircache;
	overlappingpaircache = 0;
	delete collisiondispatcher;
	collisiondispatcher = 0;
	delete collisionconfiguration;
	collisionconfiguration = 0;
}
