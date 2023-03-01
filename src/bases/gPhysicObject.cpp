/*
 * gPhysicObject.cpp
 *
 *  Created on: 19 �ub 2023
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

/*
 * for size need become between 0.4 and 100000
 */
void gPhysicObject::setSize(float x, float y, float z) {
	this->_size = glm::vec3(x, y, z > 0 ? z : _depth);
	std::cout << _rigidbody->getWorldTransform().getOrigin().z();
	//setRendererObjectSize(); // if you want chain size of physic object and rendrer object you can uncomment this line
	//setRendererObjectPosition();
	_rigidbody->getCollisionShape()->setLocalScaling(btVector3(x, y, z));
	gPhysic::Instance()->updateSingleAabb(_rigidbody);

}


glm::vec3 gPhysicObject::getSize() {
	return this->_size;
}

std::string gPhysicObject::getName() {
	return objectname;
}

void gPhysicObject::setName(std::string newname) {
	this->objectname = newname;
}

int gPhysicObject::getID() {
	return this->_id;
}


glm::vec3 gPhysicObject::getPosition() {
	return this->_position;
}


void gPhysicObject::setPosition(float x, float y, float z) {
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
	gPhysic::Instance()->updateSingleAabb(_rigidbody);
	setRendererObjectPosition();


}

void gPhysicObject::updatePositionVariable() {
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

void gPhysicObject::updateRotationVariable() {
	_rigidbody->getMotionState()->getWorldTransform(_transform);

	this->_rotation = _transform.getRotation();
	setRendererObjectRotation();
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
	gPhysic::Instance()->updateSingleAabb(_rigidbody);
}
void gPhysicObject::setRotation(float degrex, float degrey, float degrez) {
	this->_rotation.setEulerZYX(-gDegToRad(degrez), gDegToRad(degrey), gDegToRad(degrex));
	_transform.setRotation(this->_rotation);
	this->_rigidbody->setWorldTransform(this->_transform);
	setRendererObjectRotation();
	gPhysic::Instance()->updateSingleAabb(_rigidbody);
}


void gPhysicObject::setRotation(float degrez) {
	this->_rotation.setEulerZYX(-gDegToRad(degrez), gDegToRad(0.0f), gDegToRad(0.0f));
	_transform.setRotation(this->_rotation);
	this->_rigidbody->setWorldTransform(this->_transform);
	setRendererObjectRotation();
	gPhysic::Instance()->updateSingleAabb(_rigidbody);
}

void gPhysicObject::setMass(glm::vec3 newmassdirection, float newmass) {
	this->_mass = newmass;
	this->_massdirection = newmassdirection;
	_rigidbody->setMassProps(newmass, btVector3(newmassdirection.x, newmassdirection.y, newmassdirection.z));

	gPhysic::Instance()->updateSingleAabb(_rigidbody);
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

	gPhysic::Instance()->updateSingleAabb(_rigidbody);
}


float gPhysicObject::getFriction() {
	return _friction;
}


void gPhysicObject::setSpinningFriction(float newvalue) {
	this->_spinningFriction = newvalue;
	_rigidbody->setSpinningFriction(newvalue);

	gPhysic::Instance()->updateSingleAabb(_rigidbody);
}


float gPhysicObject::getSpinnigFriction() {
	return this->_spinningFriction;
}


void gPhysicObject::setRollingFriction(float newvalue) {
	this->_rollingfriction = newvalue;
	_rigidbody->setRollingFriction(newvalue);

	gPhysic::Instance()->updateSingleAabb(_rigidbody);
}


btScalar gPhysicObject::getRollingFriction() {
	return _rollingfriction;
}


void gPhysicObject::setAnisotropicFriction(glm::vec3 newvalue, int anisotropicfrictionmode) {
	this->_anisotropicfriction = newvalue;
	this->_anistropicfrictionmode = anisotropicfrictionmode;
	_rigidbody->setAnisotropicFriction(btVector3(newvalue.x, newvalue.y, newvalue.z), anisotropicfrictionmode);

	gPhysic::Instance()->updateSingleAabb(_rigidbody);
}

void gPhysicObject::setBounce(float newvalue) {
	if(newvalue > 0.0f) {

		this->_rigidbody->setRestitution(newvalue);
	}
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

void gPhysicObject::setTag(int newtag) {
	this->_tag = newtag;
}
int gPhysicObject::getTag() {
	return _tag;
}
