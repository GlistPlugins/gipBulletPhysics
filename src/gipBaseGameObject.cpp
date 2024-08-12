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
	this->_physicworld->removeObject(this);
}

	/*
	 * Use this function to set Oncollision functions
	 * use std::bind for parameter
	 */
	void gipBaseGameObject::setOnCollided(OnCollidedFunction func) {
		collidedcallback = func;
	}

	//get width of content(image, model etc)
	int gipBaseGameObject::getWidth() {
		return this->_width;
	}

	//get height of content(image, model etc)
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
							-(y + this->_colliderofset.y),
							z + this->_colliderofset.z
					)
			);
		}

		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY)	{
			_rigidbody->setWorldTransform(_transform);
		} else {
			this->_ghostobject->setWorldTransform(_transform);
		}
		updateSingleAABB();

		if(_isrenderobjectloaded) {
			if(_renderobjecttype == OBJECTRENDERTYPE_MODEL){
				_model->setPosition(_position.x,
						_position.y,
						_position.z);
			} else if(_renderobjecttype == OBJECTRENDERTYPE_MESH){
				_mesh->setPosition(_position.x,
						_position.y,
						_position.z);
			}
		}
	}

	//This uses degrees
	glm::vec3 gipBaseGameObject::getRotation() {
		glm::vec3 temprot;
		this->_rotation.getEulerZYX(temprot.z, temprot.y, temprot.x);
		temprot.x = gRadToDeg(temprot.x);
		temprot.y = gRadToDeg(temprot.y);
		temprot.z = -gRadToDeg(temprot.z);
		return temprot;
	}


	void gipBaseGameObject::setRotation(float degrex, float degrey, float degrez) {
		glm::vec3 newor;
		newor.x = gDegToRad(degrex);
		newor.y = gDegToRad(degrey);
		newor.z = gDegToRad(degrez);
		if(_isrenderobjectloaded) {
			if(_coordinatetype == COORDINATE3D) {

				if(_renderobjecttype == OBJECTRENDERTYPE_MODEL){
					_model->setOrientation(_resetquat);
//					_model->setOrientation(glm::vec3(-newor.x, newor.y, -newor.z));
					if(newor.z != 0) _model->roll(-newor.z);
					if(newor.y != 0) _model->pan(newor.y);
					if(newor.x != 0) _model->tilt(-newor.x) ;
				}
				else if(_renderobjecttype == OBJECTRENDERTYPE_MESH){
					_mesh->setOrientation(_resetquat);
//					_mesh->setOrientation(glm::vec3(-newor.x, newor.y, -newor.z));
					if(newor.z != 0) _mesh->roll(-newor.z);
					if(newor.y != 0) _mesh->pan(newor.y);
					if(newor.x != 0) _mesh->tilt(-newor.x) ;

				}
			}
		}

		this->_rotation.setEulerZYX(newor.z, newor.y, newor.x);
		_transform.setRotation(this->_rotation);
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
			this->_rigidbody->setWorldTransform(this->_transform);
		} else {
			this->_ghostobject->setWorldTransform(this->_transform);
		}
		updateSingleAABB();
	}

	//This uses degrees
	//This is for 2d objects and 2d world
	void gipBaseGameObject::setRotation2D(float degrez) {
		if(_coordinatetype == COORDINATE2D) {
			this->_rotation.setEulerZYX(gDegToRad(degrez), gDegToRad(0.0f), gDegToRad(0.0f));
			_transform.setRotation(this->_rotation);

			if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
				this->_rigidbody->setWorldTransform(this->_transform);
			} else {
				this->_ghostobject->setWorldTransform(this->_transform);
			}
			updateSingleAABB();
		}
	}

	//Set offset of collider from origin point of object
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
	 * set the size of object, 1 is default
	 * size needs to be between 0.04 and 100000
	 */
	void gipBaseGameObject::setColliderSize(float x, float y, float z) {
		if(this->_isrenderersizelocked) {
			_width *= x / _sizecollider.x;
			_height *= y / _sizecollider.y;
			if(_coordinatetype == COORDINATE3D) {
				_depth *= z / _sizecollider.z;
				if(_isrenderobjectloaded) {
					if(_renderobjecttype == OBJECTRENDERTYPE_MODEL) this->_model->setScale(_width, _height, _depth);
					else if(_renderobjecttype == OBJECTRENDERTYPE_MESH) this->_mesh->setScale(_width, _height, _depth);

				}
			}
		}

		if(_coordinatetype == COORDINATE2D) {
			if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY)
				_rigidbody->getCollisionShape()->setLocalScaling(btVector3(x / _sizecollider.x, y / _sizecollider.y, (z > 0 ? z : _sizecollider.z)  / _sizecollider.z));
			else
				_ghostobject->getCollisionShape()->setLocalScaling(btVector3(x / _sizecollider.x, y / _sizecollider.y, (z > 0 ? z : _sizecollider.z)  / _sizecollider.z));
		}
		else if(_coordinatetype == COORDINATE3D) {
			if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY)
				_rigidbody->getCollisionShape()->setLocalScaling(btVector3(x, y, z > 0 ? z : _sizecollider.z));
			else
				_ghostobject->getCollisionShape()->setLocalScaling(btVector3(x, y, z > 0 ? z : _sizecollider.z));
		}
		this->_sizecollider = glm::vec3(x, y, z > 0 ? z : _sizecollider.z);
		updateSingleAABB();
	}

	glm::vec3 gipBaseGameObject::getColliderSize() {
		return this->_sizecollider;
	}
	void gipBaseGameObject::setObjectSize(float width, float height, float depth) {
		if(_isrenderobjectloaded) {
			 if(this->_isrenderersizelocked) {
				_sizecollider.x *= width / _width;
				_sizecollider.y *= height / _height;
				if(_coordinatetype == COORDINATE2D) {
					_rigidbody->getCollisionShape()->setLocalScaling(btVector3((width / _width) * 0.5f ,  (height / _height) * 0.5f, 1.0f));
				}
				else {
					if(_depth > 0) _sizecollider.z *= depth / _depth;
					else _sizecollider.z *= depth;
					if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY)
						_rigidbody->getCollisionShape()->setLocalScaling(btVector3(_sizecollider.x, _sizecollider.y, _sizecollider.z));
					else
						_ghostobject->getCollisionShape()->setLocalScaling(btVector3(_sizecollider.x, _sizecollider.y, _sizecollider.z));
				}

				updateSingleAABB();
			}
			_width = width;
			_height = height;
			_depth = depth > 0 ? depth : _depth;
			if(_renderobjecttype == OBJECTRENDERTYPE_MODEL) _model->setScale(width, height, depth);
			else if(_renderobjecttype == OBJECTRENDERTYPE_MESH) _mesh->setScale(width, height, depth);
			setPosition(this->_position.x, this->_position.y, this->_position.z);
		}

	}
	glm::vec3 gipBaseGameObject::getObjectSize() {
		return glm::vec3(_width, _height, _depth);
	}


	glm::vec3 gipBaseGameObject::getOrigin() {
		btVector3 tempor = _transform.getOrigin();
		return {tempor.x(), -tempor.y(), tempor.z()};
	}

	//Call this function to change shape type
	void gipBaseGameObject::setShapeType(SHAPETYPE shapetype) {
		if(this->_shapetype != shapetype) {
			this->_shapetype = shapetype;
			if(shapetype == SHAPETYPE::SHAPETYPE_BOX){
				if(_coordinatetype == COORDINATE2D)	this->_collisionshape = new btBoxShape(btVector3(_sizecollider.x, _sizecollider.y,_sizecollider.z));
				else if(_coordinatetype == COORDINATE3D) this->_collisionshape = new btBoxShape(btVector3(_sizecollider.x * 0.0025f, _sizecollider.y * 0.0025f, _sizecollider.z));
			} else if(shapetype == SHAPETYPE::SHAPETYPE_SPHERE){
				if(_coordinatetype == COORDINATE2D)	this->_collisionshape = new btSphereShape(((_sizecollider.x + _sizecollider.y) * 0.5f) * 0.5f * _sizecollider.z);
				else if(_coordinatetype == COORDINATE3D)	this->_collisionshape = new btSphereShape(((_sizecollider.x + _sizecollider.y) * 0.5f) * 0.5f * _sizecollider.z * 0.0025f);
			} else if(shapetype == SHAPETYPE::SHAPETYPE_CYLINDER){
				if(_coordinatetype == COORDINATE2D) this->_collisionshape = new btCylinderShape(btVector3(_sizecollider.x, _sizecollider.y,_sizecollider.z));
				else if(_coordinatetype == COORDINATE3D) this->_collisionshape = new btCylinderShape(btVector3(_sizecollider.x * 0.0025f, _sizecollider.y * 0.0025f,_sizecollider.z * 0.0025f));
			} else if(shapetype == SHAPETYPE::SHAPETYPE_CAPSULE){
				if(_coordinatetype == COORDINATE2D) this->_collisionshape = new btCapsuleShape(_sizecollider.x, _sizecollider.y);
				else if(_coordinatetype == COORDINATE3D) this->_collisionshape = new btCapsuleShape(_sizecollider.x * 0.0025f, _sizecollider.y * 0.0025f);
			} else if(shapetype == SHAPETYPE::SHAPETYPE_CONE){
				if(_coordinatetype == COORDINATE2D) this->_collisionshape = new btConeShape(_sizecollider.x, _sizecollider.y);
				else if(_coordinatetype == COORDINATE3D) this->_collisionshape = new btConeShape(_sizecollider.x * 0.0025f, _sizecollider.y * 0.0025f);
			}

			/*
			 * The Glist Engine references the top left corner for object positions;
			 * but the bullet3 library references the center of transform.
			 * Glist engine Y axis is opposite to bullet physics y axis we need to convert y axis by multiplying by -1
			 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
			 */
			if(_coordinatetype == COORDINATE3D) {
				this->_transform.setOrigin(
						btVector3(
								_position.x + this->_colliderofset.x,
								-(_position.y + this->_colliderofset.y),
								_position.z + this->_colliderofset.z
						)
				);
			} else {
				this->_transform.setOrigin(
							btVector3(
									_position.x  + this->_colliderofset.x + _sizecollider.x * 0.5f,
									-(_position.y + this->_colliderofset.y + _sizecollider.y * 0.5f),
									0.0f
							)
					);
			}
			btVector3 localInertia(0, 0, 0);
			this->_collisionshape->calculateLocalInertia(_mass, localInertia);
			if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
				this->_rigidbody->setCollisionShape(this->_collisionshape);
				this->_rigidbody->setWorldTransform(_transform);
			} else {
				this->_ghostobject->setCollisionShape(this->_collisionshape);
				this->_ghostobject->setWorldTransform(_transform);
			}

			updateSingleAABB();
		}

	}
	int gipBaseGameObject::getShapeType() {
		return (int)this->_shapetype;
	}



	//Set base physic methods ---------------------------------------------------------------
	//Object own layers, layer are bitwise
	void gipBaseGameObject::setObjectLayers(int layers) {
		this->_objectlayers = layers;
		updateObjectLayers();
	}

	/*
	 * Set target layers which you want owner object to collide with
	 * Layer 0 means do not collide
	 * layers are bitwise
	 */
	void gipBaseGameObject::setMaskLayers(int masklayers) {
		this->_masklayers = masklayers;
		updateObjectLayers();
	}


	void gipBaseGameObject::setMass(float newmass) {
		if(this->_mass != newmass && this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
			this->_mass = newmass;
			_physicworld->_dynamicsworld->removeRigidBody(_rigidbody);
			btVector3 _interna = btVector3(0.0f, 0.0f, 0.0f);
			_rigidbody->getCollisionShape()->calculateLocalInertia(newmass, _interna);
			_rigidbody->setMassProps(newmass, _interna);

			if(newmass != 0.0f) {
				_rigidbody->setFlags(_rigidbody->getFlags() & !btCollisionObject::CF_STATIC_OBJECT);
				_isstatic = false;
			} else {
				_rigidbody->setFlags(_rigidbody->getFlags() | btCollisionObject::CF_STATIC_OBJECT);
				_isstatic = true;
			}

			_physicworld->_dynamicsworld->addRigidBody(_rigidbody, (int) _objectlayers, (int) _masklayers);
 		}
	}

	float gipBaseGameObject::getMass() {
		return this->_mass;
	}

	void gipBaseGameObject::setFriction(float newvalue) {
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
			this->_friction = newvalue;
			_rigidbody->setFriction(newvalue);
			updateSingleAABB();
		}
	}

	float gipBaseGameObject::getFriction() {
		return _friction;
	}

	void gipBaseGameObject::setRollingFriction(float newvalue) {
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
			this->_rollingfriction = newvalue;
			_rigidbody->setRollingFriction(newvalue);
			updateSingleAABB();
		}
	}

	float gipBaseGameObject::getRollingFriction() {
		return _rollingfriction;
	}

	void gipBaseGameObject::setSpinningFriction(float newvalue) {
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
			this->_spinningFriction = newvalue;
			_rigidbody->setSpinningFriction(newvalue);
			updateSingleAABB();
		}
	}

	float gipBaseGameObject::getSpinnigFriction() {
		return this->_spinningFriction;
	}

	void gipBaseGameObject::setAnisotropicFriction(glm::vec3 newvalue, int anisotropicfrictionmode) {
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
			this->_anisotropicfriction = newvalue;
			this->_anistropicfrictionmode = anisotropicfrictionmode;
			_rigidbody->setAnisotropicFriction(btVector3(newvalue.x, newvalue.y, newvalue.z), anisotropicfrictionmode);
			updateSingleAABB();
		}
	}


	//value should be between 0 and 1
	void gipBaseGameObject::setBounce(float newvalue) {
		if(newvalue > 0.0f && this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
			this->_rigidbody->setRestitution(newvalue);
			updateSingleAABB();
		}
	}

	// These "apply" methods should only be used in the update method
	void gipBaseGameObject::applyCentralForce(glm::vec3 forcevalue) {
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY)
			_rigidbody->applyCentralForce(btVector3(forcevalue.x, forcevalue.y, forcevalue.z));
	}

	void gipBaseGameObject::applyCentralImpulse(glm::vec3 impulsevalue) {
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY)
			_rigidbody->applyCentralImpulse(btVector3(impulsevalue.x, impulsevalue.y, impulsevalue.z));
	}

	void gipBaseGameObject::applyForce(glm::vec3 forcevalue,glm::vec3 forcepos) {
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY)
			_rigidbody->applyForce(btVector3(forcevalue.x, forcevalue.y, forcevalue.z), btVector3(forcepos.x, forcepos.y, forcepos.z));
	}

	void gipBaseGameObject::applyImpulse(glm::vec3 impulsevalue,glm::vec3 impulsepos) {
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY)
			_rigidbody->applyImpulse(btVector3(impulsevalue.x, impulsevalue.y, impulsevalue.z), btVector3(impulsepos.x, impulsepos.y, impulsepos.z));
	}
	void gipBaseGameObject::applyTorque(glm::vec3 torquevalue) {
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY)
			_rigidbody->applyTorque(btVector3(torquevalue.x, torquevalue.y, torquevalue.z));
	}
	void gipBaseGameObject::applyTorqueImpulse(glm::vec3 torquevalue) {
		if(this->_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY)
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
		this->_physicworld->removeObject(this);

	}

	/*
	 * This function will be called when object collides with another object
	 *
	 * !!!
	 * Don't call this function manually
	 * This function will be used by physics engine
	 */
	void gipBaseGameObject::warnCollided(gipBaseGameObject* target, glm::vec3 selfcollpos, glm::vec3 targetcollpos) {
		if (!collidedcallback) {
			return;
		}
        collidedcallback(target, selfcollpos, targetcollpos);
	}

	/*
	 * This function is for physics engine don't use manually
	 */
	void gipBaseGameObject::updateRotationVariable() {
		_transform = _rigidbody->getWorldTransform();
		glm::vec3 newor;
		this->_rotation = _transform.getRotation();

		if(_isrenderobjectloaded) {
			this->_rotation.getEulerZYX(newor.z, newor.y, newor.x);
			if(_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
				_model->setOrientation(_resetquat);

				if(newor.z != 0) _model->roll(-newor.z);
				if(newor.y != 0) _model->pan(newor.y);
				if(newor.x != 0) _model->tilt(-newor.x) ;
			}else if(_renderobjecttype == OBJECTRENDERTYPE_MESH) {
				_mesh->setOrientation(_resetquat);
				if(newor.z != 0) _mesh->roll(-newor.z);
				if(newor.y != 0) _mesh->pan(newor.y);
				if(newor.x != 0) _mesh->tilt(-newor.x) ;

			}
		}

	}

	void gipBaseGameObject::setSelfIndex(size_t index) {
		_selfindex = index;
		if(_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
			_rigidbody->setUserIndex(_selfindex);
		} else {
			_ghostobject->setUserIndex(_selfindex);
		}
	}

	/*
	 * This function is for physics engine don't use manually
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
						_transform.getOrigin().getY() - this->_colliderofset.y,
						_transform.getOrigin().getZ() - this->_colliderofset.z
				);
			} else if(type == TRANSFORMTYPE::TRANSFORMTYPE_SPHERE) {
				this->_position =  glm::vec3 (
						_transform.getOrigin().getX() - this->_colliderofset.x,
						_transform.getOrigin().getY() - this->_colliderofset.y,
						_transform.getOrigin().getZ() - this->_colliderofset.z
				);
			} else {
				this->_position = glm::vec3 (
						_transform.getOrigin().getX() - this->_colliderofset.x,
						_transform.getOrigin().getY() - this->_colliderofset.y,
						_transform.getOrigin().getZ() - this->_colliderofset.x
				);
			}
		}
		if(_isrenderobjectloaded) {
			if(_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
				_model->setPosition(_position.x  + this->_colliderofset.x,
						-(_position.y + this->_colliderofset.y),
						_position.z + this->_colliderofset.z);
			} else if(_renderobjecttype == OBJECTRENDERTYPE_MESH) {
				_mesh->setPosition(_position.x + this->_colliderofset.x,
						-(_position.y + this->_colliderofset.y),
						_position.z + this->_colliderofset.z);
			}

		}

	}

	bool gipBaseGameObject::getIsStatic() {
		return this->_isstatic;
	}

	void gipBaseGameObject::updateSingleAABB() {
		if(_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
			_physicworld->_dynamicsworld->updateSingleAabb(_rigidbody);
		} else {
			_physicworld->_dynamicsworld->updateSingleAabb(_ghostobject);
		}
	}

	void gipBaseGameObject::updateObjectLayers() {
		_physicworld->_dynamicsworld->removeRigidBody(_rigidbody);
		if(_collsionobjecttype == COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY) {
			_physicworld->_dynamicsworld->addRigidBody(_rigidbody, (int) _objectlayers, (int)_masklayers);
		} else {
			_physicworld->_dynamicsworld->addCollisionObject(_ghostobject, (int) _objectlayers, (int) _masklayers);
		}
	}


