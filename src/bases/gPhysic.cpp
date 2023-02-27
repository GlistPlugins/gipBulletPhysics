/*
 * gPhysic.cpp
 *
 *  Created on: 19 Þub 2023
 *      Author: Remzi ÝÞÇÝ
 */


#include "gPhysic.h"


gPhysic::gPhysic() {

}

gPhysic* gPhysic::Instance() {
	if(!m_physic) {
		m_physic = new gPhysic();
	}
	return gPhysic::m_physic;
}

void gPhysic::startWorld(float timestep)	{
	m_physic->setTimeStep(timestep);
	m_physic->initializeWorld();
}
/*
 * Set owner target layer
 * Set target layers whic you want collide with owner object
 * 0 means dont collide
 */
int gPhysic::addPhysicObect(gPhysicObject* object, int objectlayer, int masklayer) {
	physicobjects.push_back(object);
	dynamicsworld->addRigidBody(object->getRigidBody(), objectlayer, masklayer);
	return physicobjects.size() - 1;
}


//Initiliaze world, calls from constructor doesn need manuel call
void gPhysic::initializeWorld(int worldtype) {

	//Construct needed variables
	collisionconfiguration = new btDefaultCollisionConfiguration();
	collisiondispatcher = new btCollisionDispatcher(collisionconfiguration);
	overlappingpaircache = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver;


	softwolrdbroadphase = new btDbvtBroadphase();
	constraintsolver = solver;
	dynamicsworld = new btDiscreteDynamicsWorld(collisiondispatcher, softwolrdbroadphase, constraintsolver, collisionconfiguration);


	dynamicsworld->getSolverInfo().m_erp2 = 0.0f;
	dynamicsworld->getSolverInfo().m_globalCfm = 0.0f;
	dynamicsworld->getSolverInfo().m_numIterations = 10;
	dynamicsworld->getSolverInfo().m_solverMode = SOLVER_SIMD;  // | SOLVER_RANDMIZE_ORDER;
	dynamicsworld->setGravity(btVector3(0.0f, -9.81f, 0.0f));
	//dynamicsworld->applyGravity();
	//Setting up debugDrawer
	gDebugDraw* debugDrawer = new gDebugDraw();
	debugDrawer->clearLines();
	debugDrawer->setDebugMode( debugDrawer->getDebugMode()
          | btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb);
    dynamicsworld->setDebugDrawer(debugDrawer);

}

//need to be called from game each update
// Return step count fot world worked
int gPhysic::runPhysicWorldStep() {
	// Physics calculations doing here.
	int step = dynamicsworld->stepSimulation(_timestep, maxsubsteps, fixedtimestep);

	int count = physicobjects.size();
	for(int i = count - 1; i > -1; i--) {


		// don't update if object is static rb.
		if (!physicobjects[i]->getRigidBody()->isStaticObject()) {

				physicobjects[i]->updatePositionVariable();
				physicobjects[i]->updateRotationVariable();
		}

	}

	//Call cehck collision for each frame
	checkCollisions();


	// Uncomment print to get object transforms on debug for tests:
	//printObjectTransform();

	return step;
}

void gPhysic::checkCollisions() {
    //Count of collision
    int numManifolds = dynamicsworld->getDispatcher()->getNumManifolds();

    for (int i=0;i<numManifolds;i++)
    {
        btPersistentManifold* contactManifold =  dynamicsworld->getDispatcher()->getManifoldByIndexInternal(i);
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

            	physicobjects[contactManifold->getBody0()->getUserIndex()]->onCollided(contactManifold->getBody1()->getUserIndex(), colonobjA, colonobjB);
            	physicobjects[contactManifold->getBody1()->getUserIndex()]->onCollided(contactManifold->getBody0()->getUserIndex(), colonobjB, colonobjA);
            }
        }
    }
}



void gPhysic::setGravity(glm::vec3 newgravity) {
	dynamicsworld->setGravity(btVector3((btScalar)newgravity.x, (btScalar)newgravity.y, (btScalar)newgravity.z));
	dynamicsworld->applyGravity();
}

btVector3 gPhysic::getGravity() {
	return dynamicsworld->getGravity();
}

/*
 * Call this function in the GamaCanvas draw method to draw physic bounds and debug views
 */
void gPhysic::drawDebug() {
	dynamicsworld->debugDrawWorld();
}

void gPhysic::printObjectTransform() {
	std::vector<btCollisionObject*> colobjarray;
	std::vector<btRigidBody*> rbarray;

	for (auto object : physicobjects) {
		int id = object->getID();
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

void gPhysic::updateSingleAabb(btCollisionObject* targetcollisionobject) {
	dynamicsworld->updateSingleAabb(targetcollisionobject);
}

void gPhysic::setTimeStep(float timestep) {
	_timestep = (btScalar)timestep;
}

//cleanup in the reverse order of creation/initialization
void gPhysic::clean() {
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
	int count = physicobjects.size();
	for (int j = 0; j < count; j++)
	{
		//TODO: Buraya tekrar bak
		//delete physicobjects[j];

	}
	physicobjects.clear();

	delete dynamicsworld;
	dynamicsworld = 0;
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



gPhysic::~gPhysic() {
	clean();
}
