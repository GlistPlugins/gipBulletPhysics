/*
 * gipBaseGameObject.cpp
 *
 *  Created on: 5 Mar 2023
 *      Author: remzi
 */

#include "gipBaseGameObject.h"
#include "gipBulletPhysics.h"

gipBaseGameObject::gipBaseGameObject() {
}

gipBaseGameObject::~gipBaseGameObject() {
	this->_physicworld->removeObject(this->_id);
	delete _image;
	delete _model;
	delete _collisionshape;
	delete _rigidbody;
	_image = nullptr;
	_model = nullptr;
	_collisionshape = nullptr;
	_rigidbody = nullptr;
	_physicworld = nullptr;
}


	/*
	 * Use this function for setting Oncollision fu
	 * use std::bind for parameter
	 */
	void gipBaseGameObject::setOnCollided(std::function<void(int, glm::vec3, glm::vec3)> onColl){
		_isOnCollidedFuncSetted = true;
		this->_onColl = onColl;
	}
	//get with of content(image, model etc)
	int gipBaseGameObject::getWidth() {
		return this->_width;
	}

	//get height of conten(image, model etc)
	int gipBaseGameObject::getHeight() {
		return this->_height;
	}

	void gipBaseGameObject::setTag(int newtag) {
		this->_tag = newtag;
	}
	int gipBaseGameObject::getTag() {
		return this->_tag;
	}

	std::string gipBaseGameObject::getName() {
		return this->_objectname;
	}
	void gipBaseGameObject::setName(std::string newname) {
		this->_objectname = newname;
	}

	int gipBaseGameObject::getID() {
		return this->_id;
	}


	glm::vec3 gipBaseGameObject::getPosition() {
		return this->_position;
	}
	void gipBaseGameObject::setPosition(float x, float y, float z) {
		this->_position = glm::vec3(x, y, z);

		if(_coordinatetype == COORDINATE2D) {
			this->_transform.setOrigin(
					btVector3(
							x + (_sizecollider.x * 0.5f) + this->_colliderofset.x,
							-(y + (_sizecollider.y * 0.5f) + this->_colliderofset.y),
							0.0f
					)
			);
		} else if(_coordinatetype == COORDINATE3D) {
			this->_transform.setOrigin(
					btVector3(
							x + this->_colliderofset.x,
							y + this->_colliderofset.y,
							z + this->_colliderofset.z
					)
			);
		}
		_rigidbody->setWorldTransform(_transform);
		this->_physicworld->updateSingleAabb(_rigidbody);

		if(_isrenderobjectloaded && _renderobjecttype == OBJECTRENDERTYPE_MODEL) {
			_model->setPosition(_position.x,
					-_position.y,
					_position.z);
		}
	}

	//degree
	glm::vec3 gipBaseGameObject::getRotation() {
		glm::vec3 temprot;
		this->_rotation.getEulerZYX(temprot.z, temprot.y, temprot.x);
		temprot.x = gRadToDeg(temprot.x);
		temprot.y = gRadToDeg(temprot.y);
		temprot.z = -gRadToDeg(temprot.z);
		return temprot;
	}
	//Bu kýsým düzeltilecek
	void gipBaseGameObject::setRotation(float degrex, float degrey, float degrez) {
		this->_rotation.setEulerZYX(-gDegToRad(degrez), gDegToRad(degrey), gDegToRad(degrex));
		_transform.setRotation(this->_rotation);
		this->_rigidbody->setWorldTransform(this->_transform);
		this->_physicworld->updateSingleAabb(_rigidbody);
		if(_isrenderobjectloaded && _renderobjecttype == OBJECTRENDERTYPE_MODEL) {
			glm::vec3 newor;
			newor.x = gRadToDeg(_rotation.getAxis().getX() * _rotation.getAngle());
			newor.y = gRadToDeg(_rotation.getAxis().getY() * _rotation.getAngle());
			newor.z = -gRadToDeg(_rotation.getAxis().getZ() * _rotation.getAngle());
			_model->setOrientation(newor);
		}
	}
	//degree
	//This for 2d object and world
	void gipBaseGameObject::setRotation(float degrez) {
		if(_coordinatetype == COORDINATE2D) {
			this->_rotation.setEulerZYX(-gDegToRad(degrez), gDegToRad(0.0f), gDegToRad(0.0f));
			_transform.setRotation(this->_rotation);
			this->_rigidbody->setWorldTransform(this->_transform);
			this->_physicworld->updateSingleAabb(_rigidbody);
		}

	}

	//Set offset collider fromorigin of object
	void gipBaseGameObject::setColliderOffset(float offsetx, float offsety, float offsetz) {
		_colliderofset.x = offsetx;
		_colliderofset.y = offsety;
		_colliderofset.z = offsetz;
		this->setPosition(this->_position.x, this->_position.y, this->_position.z);
	}

	glm::vec3 gipBaseGameObject::getColliderOffset() {
		return _colliderofset;
	}

	void gipBaseGameObject::setIsSizeLocked(bool islocked) {
		this->_isrenderersizelocked = islocked;
	}
	bool gipBaseGameObject::getIsSizeLocked() {
		return this->_isrenderersizelocked;
	}

	/*
	 * set size of object 1 is default
	 * size need to be between 0.04 and 100000
	 */
	void gipBaseGameObject::setColliderSize(float x, float y, float z) {
		if(this->_isrenderersizelocked) {
			_width *= x / _sizecollider.x;
			_height *= y / _sizecollider.y;
			if(_coordinatetype == COORDINATE3D) {
				_depth *= z / _sizecollider.z;
				if(_isrenderobjectloaded) {
					this->_model->scale(_width, _height, _depth);
				}
			}
		}
		_rigidbody->getCollisionShape()->setLocalScaling(btVector3(x / _sizecollider.x, y / _sizecollider.y, (z > 0 ? z : _sizecollider.z)  / _sizecollider.z));
		this->_sizecollider = glm::vec3(x, y, z > 0 ? z : _sizecollider.z);
		this->_physicworld->updateSingleAabb(_rigidbody);
	}

	glm::vec3 gipBaseGameObject::getColliderSize() {
		return this->_sizecollider;
	}
	void gipBaseGameObject::setObjectSize(float width, float height, float depth) {
		if(_isrenderobjectloaded && this->_isrenderersizelocked) {
			_sizecollider.x *= width / _width;
			_sizecollider.y *= height / _height;
			if(_coordinatetype == COORDINATE2D) {
				_rigidbody->getCollisionShape()->setLocalScaling(btVector3((width / _width) * 0.5f ,  (height / _height) * 0.5f, 1.0f));
			}
			else {
				if(_depth > 0) _sizecollider.z *= depth / _depth;
				else _sizecollider.z *= depth;
				_rigidbody->getCollisionShape()->setLocalScaling(btVector3((width / _width) * 0.5f ,  (height / _height) * 0.5f, _depth > 0 ? depth / _depth : depth));
			}



			this->_physicworld->updateSingleAabb(_rigidbody);
		}
		_width = width;
		_height = height;
		_depth = depth > 0 ? depth : _depth;
		if(_isrenderobjectloaded && _renderobjecttype == OBJECTRENDERTYPE_MODEL) {
			_model->scale(width, height, depth);
		}
		setPosition(this->_position.x, this->_position.y, this->_position.z);
	}
	glm::vec3 gipBaseGameObject::getObjectSize() {
		return glm::vec3(_width, _height, _depth);
	}


	glm::vec3 gipBaseGameObject::getOrigin() {
		btVector3 tempor = _transform.getOrigin();
		return {tempor.x(), -tempor.y(), tempor.z()};
	}

	//Call this function for changing shape type
	void gipBaseGameObject::setShapeType(SHAPETYPE shapetype) {
		if(this->_shapetype != shapetype) {
			this->_shapetype = shapetype;
			if(shapetype == SHAPETYPE::SHAPETYPE_BOX){
				this->_collisionshape =new btBoxShape(btVector3(_sizecollider.x, _sizecollider.y,_sizecollider.z));
			} else if(shapetype == SHAPETYPE::SHAPETYPE_SPHERE){
				this->_collisionshape = new btSphereShape(((_sizecollider.x + _sizecollider.y) * 0.5f) * 0.5f * _sizecollider.z);
			} else if(shapetype == SHAPETYPE::SHAPETYPE_CYLINDER){
				this->_collisionshape = new btCylinderShape(btVector3(_sizecollider.x, _sizecollider.y,_sizecollider.z));
			} else if(shapetype == SHAPETYPE::SHAPETYPE_CAPSULE){
				this->_collisionshape = new btCapsuleShape(_sizecollider.x, _sizecollider.y);
			} else if(shapetype == SHAPETYPE::SHAPETYPE_CONE){
				this->_collisionshape = new btConeShape(_sizecollider.x, _sizecollider.y);
			}
			this->_transform.setIdentity();
			/*
			 * The Glist Engine references the top left corner for object positions;
			 * but the bullet3 library references the center of transform.
			 * Glist engine Y axis is opposite to bullet physic y axis nned to convert y axis by multiply -1
			 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
			 */
			if(_coordinatetype == COORDINATE3D) {
				this->_transform.setOrigin(
						btVector3(
								_position.x,
								-_position.y,
								_position.z
						)
				);
			} else {
				this->_transform.setOrigin(
							btVector3(
									_position.x + _sizecollider.x * 0.5f,
									-(_position.y + _sizecollider.y * 0.5f),
									0.0f
							)
					);
			}
			btVector3 localInertia(0, 0, 0);
			this->_collisionshape->calculateLocalInertia(_mass, localInertia);
			this->_rigidbody->setCollisionShape(this->_collisionshape);
			this->_rigidbody->setWorldTransform(_transform);
			this->_physicworld->updateSingleAabb(_rigidbody);
		}

	}
	int gipBaseGameObject::getShapeType() {
		return (int)this->_shapetype;
	}



	//Set base physic methods ---------------------------------------------------------------
	//Object own layers, layer are bitwise
	void gipBaseGameObject::setObjectLayers(int layers) {
		this->_objectlayers = layers;
		this->_physicworld->updateObjectlayers(this->_id);
	}
	//Set target layers whic we want collide with this object, layers asre bitewise
	void gipBaseGameObject::setMaskLayers(int masklayers) {
		this->_masklayers = masklayers;
		this->_physicworld->updateObjectlayers(this->_id);
	}


	void gipBaseGameObject::setMass(float newmass) {
		if(this->_mass != newmass) {
			this->_mass = newmass;
			this->_physicworld->setMass(this, newmass);
 		}

	}

	float gipBaseGameObject::getMass() {
		return this->_mass;
	}

	void gipBaseGameObject::setFriction(float newvalue) {
		this->_friction = newvalue;
		_rigidbody->setFriction(newvalue);
		this->_physicworld->updateSingleAabb(_rigidbody);
	}

	float gipBaseGameObject::getFriction() {
		return _friction;
	}

	void gipBaseGameObject::setRollingFriction(float newvalue) {
		this->_rollingfriction = newvalue;
		_rigidbody->setRollingFriction(newvalue);
		this->_physicworld->updateSingleAabb(_rigidbody);
	}

	float gipBaseGameObject::getRollingFriction() {
		return _rollingfriction;
	}

	void gipBaseGameObject::setSpinningFriction(float newvalue) {
		this->_spinningFriction = newvalue;
		_rigidbody->setSpinningFriction(newvalue);
		this->_physicworld->updateSingleAabb(_rigidbody);
	}

	float gipBaseGameObject::getSpinnigFriction() {
		return this->_spinningFriction;
	}

	void gipBaseGameObject::setAnisotropicFriction(glm::vec3 newvalue, int anisotropicfrictionmode) {
		this->_anisotropicfriction = newvalue;
		this->_anistropicfrictionmode = anisotropicfrictionmode;
		_rigidbody->setAnisotropicFriction(btVector3(newvalue.x, newvalue.y, newvalue.z), anisotropicfrictionmode);
		this->_physicworld->updateSingleAabb(_rigidbody);
	}


	//value should become between 0 and 1
	void gipBaseGameObject::setBounce(float newvalue) {
		if(newvalue > 0.0f) {
			this->_rigidbody->setRestitution(newvalue);
			this->_physicworld->updateSingleAabb(_rigidbody);
		}
	}


	// These apply methods should be used in update method.
	void gipBaseGameObject::applyCentralForce(glm::vec3 forcevalue) {
		_rigidbody->applyCentralForce(btVector3(forcevalue.x, forcevalue.y, forcevalue.z));
	}

	void gipBaseGameObject::applyCentralImpulse(glm::vec3 impulsevalue) {
		_rigidbody->applyCentralImpulse(btVector3(impulsevalue.x, impulsevalue.y, impulsevalue.z));
	}

	void gipBaseGameObject::applyForce(glm::vec3 forcevalue,glm::vec3 forcepos) {
		_rigidbody->applyForce(btVector3(forcevalue.x, forcevalue.y, forcevalue.z), btVector3(forcepos.x, forcepos.y, forcepos.z));
	}

	void gipBaseGameObject::applyImpulse(glm::vec3 impulsevalue,glm::vec3 impulsepos) {
		_rigidbody->applyImpulse(btVector3(impulsevalue.x, impulsevalue.y, impulsevalue.z), btVector3(impulsepos.x, impulsepos.y, impulsepos.z));
	}
	void gipBaseGameObject::applyTorque(glm::vec3 torquevalue) {
		_rigidbody->applyTorque(btVector3(torquevalue.x, torquevalue.y, torquevalue.z));
	}
	void gipBaseGameObject::applyTorqueImpulse(glm::vec3 torquevalue) {
		_rigidbody->applyTorqueImpulse(btVector3(torquevalue.x, torquevalue.y, torquevalue.z));
	}

	//----------------------------------------------------------------------------------------


	btTransform* gipBaseGameObject::getTransform() {
		return &this->_transform;
	}
	btCollisionShape* gipBaseGameObject::getCollisionShape() {
		return _collisionshape;
	}
	btRigidBody* gipBaseGameObject::getRigidBody() {
		return _rigidbody;
	}

	void gipBaseGameObject::destroy() {
		this->_physicworld->removeObject(this->_id);
		delete this;
	}

	/*
	 * This function will be called when object collided with another object
	 *
	 * !!!
	 *  Dont call this function manuel
	 *  This function will be used by physic engine
	 */
	void gipBaseGameObject::warnCollided(int targetobjectid, glm::vec3 selfcollpos, glm::vec3 targetcollpos) {

		if(this->_isOnCollidedFuncSetted == true) {
			this->_onColl(targetobjectid, selfcollpos, targetcollpos);
		}
	}

	/*
	 * This function is for physic engine dont use manualy
	 */
	void gipBaseGameObject::updateRotationVariable() {
		_transform = _rigidbody->getWorldTransform();

		this->_rotation = _transform.getRotation();
	}

	/*
	 * This function is for physic engine dont use manualy
	 */
	void gipBaseGameObject::updatePositionVariable() {
		TRANSFORMTYPE type = (TRANSFORMTYPE)_collisionshape->getShapeType();
		_transform = _rigidbody->getWorldTransform();
		/*
		 * Box = 0
		 * Sphere = 8
		 * Compound = 31
		 */

		if(_coordinatetype == COORDINATE::COORDINATE2D) {
			if(type == TRANSFORMTYPE::TRANSFORMTYPE_BOX) {
				this->_position =  glm::vec3 (
						_transform.getOrigin().getX() - (_width * 0.5f) - this->_colliderofset.x,
						-(_transform.getOrigin().getY() + (_height * 0.5f) - this->_colliderofset.y),
						0.0f
				);
			} else if(type == TRANSFORMTYPE::TRANSFORMTYPE_SPHERE) {
				this->_position =  glm::vec3 (
						_transform.getOrigin().getX() - (_height * 0.5f) - this->_colliderofset.x,
						-(_transform.getOrigin().getY() + (_height * 0.5f) - this->_colliderofset.y),
						0.0f
				);
			} else {
				this->_position = glm::vec3 (
						_transform.getOrigin().getX() - _width * 0.5f - this->_colliderofset.x,
						-(_transform.getOrigin().getY() + _height * 0.5f - this->_colliderofset.y),
						0.0f
				);
			}
		} else {
			if(type == TRANSFORMTYPE::TRANSFORMTYPE_BOX) {
				this->_position =  glm::vec3 (
						_transform.getOrigin().getX() - this->_colliderofset.x,
						-(_transform.getOrigin().getY() - this->_colliderofset.y),
						_transform.getOrigin().getZ() - this->_colliderofset.z
				);
			} else if(type == TRANSFORMTYPE::TRANSFORMTYPE_SPHERE) {
				this->_position =  glm::vec3 (
						_transform.getOrigin().getX() - this->_colliderofset.x,
						-(_transform.getOrigin().getY() - this->_colliderofset.y),
						_transform.getOrigin().getZ() - this->_colliderofset.z
				);
			} else {
				this->_position = glm::vec3 (
						_transform.getOrigin().getX() - this->_colliderofset.x,
						-(_transform.getOrigin().getY() - this->_colliderofset.y),
						_transform.getOrigin().getZ() - this->_colliderofset.x
				);
			}
		}
		if(_isrenderobjectloaded && _renderobjecttype == OBJECTRENDERTYPE_MODEL) {
			_model->setPosition(_position.x,
					-_position.y,
					_position.z);
		}

	}

	bool gipBaseGameObject::getIsStatic() {
		return this->_isstatic;
	}


