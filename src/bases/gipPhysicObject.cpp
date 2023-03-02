/*
 * gPhysicObject.cpp
 *
 *  Created on: 19 �ub 2023
 *      Author: remzi
 */

#include "gipPhysicObject.h"
#include "gImage.h"
#include "bases/gipBulletPhysics.h"
#include "gModel.h"

gipPhysicObject::gipPhysicObject() {
}


gipPhysicObject::~gipPhysicObject() {
	gipBulletPhysics::Instance()->removeObject(this->_id);
	delete _image;
	delete _model;
	delete _collisionshape;
	delete _rigidbody;
	_image = nullptr;
	_model = nullptr;
	_collisionshape = nullptr;
	_rigidbody = nullptr;
}


int gipPhysicObject::getWidth() {
	return _width;
}

int gipPhysicObject::getHeight() {
	return _height;
}

/*
 * for size need become between 0.4 and 100000
 */
void gipPhysicObject::setSize(float x, float y, float z) {
	this->_size = glm::vec3(x, y, z > 0 ? z : _depth);
	//setRendererObjectSize(); // if you want chain size of physic object and rendrer object you can uncomment this line
	//setRendererObjectPosition();
	_rigidbody->getCollisionShape()->setLocalScaling(btVector3(x, y, z));
	gipBulletPhysics::Instance()->updateSingleAabb(_rigidbody);

}


glm::vec3 gipPhysicObject::getSize() {
	return this->_size;
}

std::string gipPhysicObject::getName() {
	return objectname;
}

void gipPhysicObject::setName(std::string newname) {
	this->objectname = newname;
}

int gipPhysicObject::getID() {
	return this->_id;
}


glm::vec3 gipPhysicObject::getPosition() {
	return this->_position;
}


void gipPhysicObject::setPosition(float x, float y, float z) {
	this->_position = glm::vec3(x, y, z);

	if(_renderobjecttype == OBJECTRENDERTYPE_IMAGE) {
		this->_transform.setOrigin(
				btVector3(
						x + _width * 0.5f,
						-(y + _height * 0.5f),
						0.0f
				)
		);
	} else if(_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		this->_transform.setOrigin(
				btVector3(
						x + _width,
						y + _height,
						z + _depth
				)
		);
	} else if(_renderobjecttype == OBJECTRENDERTYPE_NONE) {
		if(z == 0.0f) {
			this->_transform.setOrigin(
					btVector3(
							x + _width * 0.5f,
							-(y + _height * 0.5f),
							0.0f
					)
			);
		} else {
			this->_transform.setOrigin(
					btVector3(
							x + _width,
							y + _height,
							z + _depth
					)
			);
		}

	}


	_rigidbody->setWorldTransform(_transform);
	gipBulletPhysics::Instance()->updateSingleAabb(_rigidbody);
	setRendererObjectPosition();


}

void gipPhysicObject::updatePositionVariable() {
	transformtype type = (transformtype)_collisionshape->getShapeType();
	glm::vec3 temp_pos;

	_rigidbody->getMotionState()->getWorldTransform(_transform);


	/*
	 * Box = 0
	 * Sphere = 8
	 * Compound = 31
	 */

	if(is2d) {
		if(type == transformtype::TRANSFORMTYPE_BOX) {
			this->_position =  glm::vec3 (
					_transform.getOrigin().getX() - _width * 0.5f,
					-(_transform.getOrigin().getY() + _height * 0.5f),
					0.0f
			);
		} else if(type == transformtype::TRANSFORMTYPE_SPHERE) {
			this->_position =  glm::vec3 (
					_transform.getOrigin().getX() - (_height * 0.5f),
					-(_transform.getOrigin().getY() + (_height * 0.5f)),
					0.0f
			);
		} else {
			this->_position = glm::vec3 (
					_transform.getOrigin().getX() + _width,
					-(_transform.getOrigin().getY() + _height),
					0.0f
			);
		}
	} else {
		if(type == transformtype::TRANSFORMTYPE_BOX) {
			this->_position =  glm::vec3 (
					_transform.getOrigin().getX() - _width * 0.5f,
					-(_transform.getOrigin().getY() + _height * 0.5f),
					_transform.getOrigin().getZ()
			);
		} else if(type == transformtype::TRANSFORMTYPE_SPHERE) {
			this->_position =  glm::vec3 (
					_transform.getOrigin().getX() - (_height * 0.5f),
					-(_transform.getOrigin().getY() + (_height * 0.5f)),
					_transform.getOrigin().getZ()
			);
		} else {
			this->_position = glm::vec3 (
					_transform.getOrigin().getX() + _width,
					-(_transform.getOrigin().getY() + _height),
					_transform.getOrigin().getZ()
			);
		}
	}

	setRendererObjectPosition();
}

void gipPhysicObject::updateRotationVariable() {
	_rigidbody->getMotionState()->getWorldTransform(_transform);

	this->_rotation = _transform.getRotation();
	setRendererObjectRotation();
}
glm::vec3 gipPhysicObject::getOrigin() {
	btVector3 tempor = _transform.getOrigin();
	return {tempor.x(), tempor.y(), tempor.z()};
}


btQuaternion gipPhysicObject::getRotation() {
	return _rotation;
}


void gipPhysicObject::setRotation(btQuaternion newrotation) {
	this->_rotation = newrotation;
	_transform.setRotation(newrotation);
	setRendererObjectRotation();
	gipBulletPhysics::Instance()->updateSingleAabb(_rigidbody);
}
void gipPhysicObject::setRotation(float degrex, float degrey, float degrez) {
	this->_rotation.setEulerZYX(-gDegToRad(degrez), gDegToRad(degrey), gDegToRad(degrex));
	_transform.setRotation(this->_rotation);
	this->_rigidbody->setWorldTransform(this->_transform);
	setRendererObjectRotation();
	gipBulletPhysics::Instance()->updateSingleAabb(_rigidbody);
}


void gipPhysicObject::setRotation(float degrez) {
	this->_rotation.setEulerZYX(-gDegToRad(degrez), gDegToRad(0.0f), gDegToRad(0.0f));
	_transform.setRotation(this->_rotation);
	this->_rigidbody->setWorldTransform(this->_transform);
	setRendererObjectRotation();
	gipBulletPhysics::Instance()->updateSingleAabb(_rigidbody);
}

void gipPhysicObject::setMass(glm::vec3 newmassdirection, float newmass) {
	this->_mass = newmass;
	this->_massdirection = newmassdirection;
	_rigidbody->setMassProps(newmass, btVector3(newmassdirection.x, newmassdirection.y, newmassdirection.z));

	gipBulletPhysics::Instance()->updateSingleAabb(_rigidbody);
}


glm::vec3 gipPhysicObject::getMassDirection() {
	return _massdirection;
}


void gipPhysicObject::onCollided(int targetobjectid, glm::vec3 selfcollpos, glm::vec3 targetcollpos) {
	if(_isOnCollidedFuncSetted)	_onColl(targetobjectid, selfcollpos, targetcollpos);
}



void gipPhysicObject::setOnCollided(std::function<void(int, glm::vec3, glm::vec3)> onColl) {
	_isOnCollidedFuncSetted = true;
	this->_onColl = onColl;
}


void gipPhysicObject::setFriction(float newvalue) {
	this->_friction = newvalue;
	_rigidbody->setFriction(newvalue);

	gipBulletPhysics::Instance()->updateSingleAabb(_rigidbody);
}


float gipPhysicObject::getFriction() {
	return _friction;
}


void gipPhysicObject::setSpinningFriction(float newvalue) {
	this->_spinningFriction = newvalue;
	_rigidbody->setSpinningFriction(newvalue);

	gipBulletPhysics::Instance()->updateSingleAabb(_rigidbody);
}


float gipPhysicObject::getSpinnigFriction() {
	return this->_spinningFriction;
}


void gipPhysicObject::setRollingFriction(float newvalue) {
	this->_rollingfriction = newvalue;
	_rigidbody->setRollingFriction(newvalue);

	gipBulletPhysics::Instance()->updateSingleAabb(_rigidbody);
}


btScalar gipPhysicObject::getRollingFriction() {
	return _rollingfriction;
}


void gipPhysicObject::setAnisotropicFriction(glm::vec3 newvalue, int anisotropicfrictionmode) {
	this->_anisotropicfriction = newvalue;
	this->_anistropicfrictionmode = anisotropicfrictionmode;
	_rigidbody->setAnisotropicFriction(btVector3(newvalue.x, newvalue.y, newvalue.z), anisotropicfrictionmode);

	gipBulletPhysics::Instance()->updateSingleAabb(_rigidbody);
}

void gipPhysicObject::setBounce(float newvalue) {
	if(newvalue > 0.0f) {

		this->_rigidbody->setRestitution(newvalue);
	}
}


void gipPhysicObject::applyCentralForce(glm::vec3 forcevalue) {
	_rigidbody->applyCentralForce(btVector3(forcevalue.x, forcevalue.y, forcevalue.z));
}


void gipPhysicObject::applyCentralImpulse(glm::vec3 impulsevalue) {
	_rigidbody->applyCentralImpulse(btVector3(impulsevalue.x, impulsevalue.y, impulsevalue.z));
}


void gipPhysicObject::applyForce(glm::vec3 forcevalue,glm::vec3 forcepos) {
	_rigidbody->applyForce(btVector3(forcevalue.x, forcevalue.y, forcevalue.z), btVector3(forcepos.x, forcepos.y, forcepos.z));
}



void gipPhysicObject::applyImpulse(glm::vec3 impulsevalue,glm::vec3 impulsepos) {
	_rigidbody->applyImpulse(btVector3(impulsevalue.x, impulsevalue.y, impulsevalue.z), btVector3(impulsepos.x, impulsepos.y, impulsepos.z));
}


void gipPhysicObject::applyTorque(glm::vec3 torquevalue) {
	_rigidbody->applyTorque(btVector3(torquevalue.x, torquevalue.y, torquevalue.z));
}



void gipPhysicObject::applyTorqueImpulse(glm::vec3 torquevalue) {
	_rigidbody->applyTorqueImpulse(btVector3(torquevalue.x, torquevalue.y, torquevalue.z));
}

btCollisionShape* gipPhysicObject::getCollisionShape() {
	return _collisionshape;
}

btRigidBody* gipPhysicObject::getRigidBody() {
	return _rigidbody;
}

btTransform*  gipPhysicObject::getTransform() {
	return &_transform;
}

void gipPhysicObject::setTag(int newtag) {
	this->_tag = newtag;
}
int gipPhysicObject::getTag() {
	return _tag;
}

void gipPhysicObject::destroy() {
	gipBulletPhysics::Instance()->removeObject(this->_id);
	delete this;
}