/*
 * gPhysicObject.cpp
 *
 *  Created on: 19 Þub 2023
 *      Author: remzi
 */

#include "gPhysicObject.h"
#include "gImage.h"
#include "gPhysic.h"
#include "gModel.h"

gPhysicObject::gPhysicObject() {
}


gPhysicObject::~gPhysicObject() {
	_image = nullptr;
	_model = nullptr;
	_collisionshape = nullptr;
	_rigidbody = nullptr;
	delete _image;
	delete _model;
	delete _collisionshape;
	delete _rigidbody;
}


int gPhysicObject::getWidth() {
	return _width;
}

int gPhysicObject::getHeight() {
	return _height;
}

void gPhysicObject::setSize(glm::vec3 newvalue) {
	this->_size = newvalue;
	setRendererObjectSize();
	_rigidbody->getCollisionShape()->setLocalScaling(btVector3(newvalue.x, newvalue.y, newvalue.z));
	gPhysic::world()->updateSingleAabb(_rigidbody);
}


glm::vec3 gPhysicObject::getSize() {
	return this->_size;
}


int gPhysicObject::getID() {
	return this->_id;
}


glm::vec3 gPhysicObject::getPosition() {
	return this->_position;
}


void gPhysicObject::setPosition(glm::vec3 newposition) {
	this->_position = newposition;
	setRendererObjectPosition();
	this->_transform.setOrigin(
			btVector3(
					newposition.x + _width * 0.5f,
					-(newposition.y + _height * 0.5f),
					newposition.z + _depth * 0.5f
			)
	);
	_rigidbody->setWorldTransform(_transform);
	gPhysic::world()->updateSingleAabb(_rigidbody);


}
void gPhysicObject::updatePositionVariable() {
	transformtype type = (transformtype)_collisionshape->getShapeType();
	glm::vec3 temp_pos;

	//temp_rigidbody->getMotionState()->getWorldTransform(temp_transform);
	_rigidbody->getMotionState()->getWorldTransform(_transform);


	/*
	 * Box = 0
	 * Sphere = 8
	 * Compound = 31
	 */


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

void gPhysicObject::updateRotationVariable() {
	_rigidbody->getMotionState()->getWorldTransform(_transform);
	this->_rotation = _transform.getRotation();
}
glm::vec3 gPhysicObject::getOrigin() {
	btVector3 tempor = _transform.getOrigin();
	return {tempor.x(), tempor.y(), tempor.z()};
}


btQuaternion gPhysicObject::getRotation() {
	return _rotation;
}


void gPhysicObject::setRotation(btQuaternion newrotation) {
	this->_rotation = newrotation;
	_transform.setRotation(newrotation);
	setRendererObjectRotation();
	gPhysic::world()->updateSingleAabb(_rigidbody);
}


void gPhysicObject::setMass(glm::vec3 newmassdirection, float newmass) {
	this->_mass = newmass;
	this->_massdirection = newmassdirection;
	_rigidbody->setMassProps(newmass, btVector3(newmassdirection.x, newmassdirection.y, newmassdirection.z));

	gPhysic::world()->updateSingleAabb(_rigidbody);
}


glm::vec3 gPhysicObject::getMassDirection() {
	return _massdirection;
}


void gPhysicObject::onCollided(int targetobjectid, glm::vec3 selfcollpos, glm::vec3 targetcollpos) {
	if(_isOnCollidedFuncSetted)	_onColl(targetobjectid, selfcollpos, targetcollpos);
}



void gPhysicObject::setOnCollided(std::function<void(int, glm::vec3, glm::vec3)> onColl) {
	_isOnCollidedFuncSetted = true;
	this->_onColl = onColl;
}


void gPhysicObject::setFriction(float newvalue) {
	this->_friction = newvalue;
	_rigidbody->setFriction(newvalue);

	gPhysic::world()->updateSingleAabb(_rigidbody);
}


float gPhysicObject::getFriction() {
	return _friction;
}


void gPhysicObject::setSpinningFriction(float newvalue) {
	this->_spinningFriction = newvalue;
	_rigidbody->setSpinningFriction(newvalue);

	gPhysic::world()->updateSingleAabb(_rigidbody);
}


float gPhysicObject::getSpinnigFriction() {
	return this->_spinningFriction;
}


void gPhysicObject::setRollingFriction(float newvalue) {
	this->_rollingfriction = newvalue;
	_rigidbody->setRollingFriction(newvalue);

	gPhysic::world()->updateSingleAabb(_rigidbody);
}


btScalar gPhysicObject::getRollingFriction() {
	return _rollingfriction;
}


void gPhysicObject::setAnisotropicFriction(glm::vec3 newvalue, int anisotropicfrictionmode) {
	this->_anisotropicfriction = newvalue;
	this->_anistropicfrictionmode = anisotropicfrictionmode;
	_rigidbody->setAnisotropicFriction(btVector3(newvalue.x, newvalue.y, newvalue.z), anisotropicfrictionmode);

	gPhysic::world()->updateSingleAabb(_rigidbody);
}


void gPhysicObject::applyCentralForce(glm::vec3 forcevalue) {
	_rigidbody->applyCentralForce(btVector3(forcevalue.x, forcevalue.y, forcevalue.z));
}


void gPhysicObject::applyCentralImpulse(glm::vec3 impulsevalue) {
	_rigidbody->applyCentralImpulse(btVector3(impulsevalue.x, impulsevalue.y, impulsevalue.z));
}


void gPhysicObject::applyForce(glm::vec3 forcevalue,glm::vec3 forcepos) {
	_rigidbody->applyForce(btVector3(forcevalue.x, forcevalue.y, forcevalue.z), btVector3(forcepos.x, forcepos.y, forcepos.z));
}



void gPhysicObject::applyImpulse(glm::vec3 impulsevalue,glm::vec3 impulsepos) {
	_rigidbody->applyImpulse(btVector3(impulsevalue.x, impulsevalue.y, impulsevalue.z), btVector3(impulsepos.x, impulsepos.y, impulsepos.z));
}


void gPhysicObject::applyTorque(glm::vec3 torquevalue) {
	_rigidbody->applyTorque(btVector3(torquevalue.x, torquevalue.y, torquevalue.z));
}



void gPhysicObject::applyTorqueImpulse(glm::vec3 torquevalue) {
	_rigidbody->applyTorqueImpulse(btVector3(torquevalue.x, torquevalue.y, torquevalue.z));
}

btCollisionShape* gPhysicObject::getCollisionShape() {
	return _collisionshape;
}

btRigidBody* gPhysicObject::getRigidBody() {
	return _rigidbody;
}

btTransform*  gPhysicObject::getTransform() {
	return &_transform;
}
