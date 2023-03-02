/*
 * gPhysic2dCircle.cpp
 *
 *  Created on: 21 02 2023
 *      Author: Remzi ISCI
 */

#include <gipPhysic2dCircle.h>

/*
 * rotaion is degree format
 * size need to become between 0.04 and 100.000
 */
gipPhysic2dCircle::gipPhysic2dCircle(gImage* image, bool isstatic, float mass, float radius, int objectlayers, int masklayers) {
	this->_image = image;
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_IMAGE;
	this->_width = _image->getWidth();
	this->_height = _image->getHeight();
	this->_isstatic = isstatic;
	this->_mass = mass;
	this->is2d = true;
	this->_size = glm::vec3(radius, radius, radius);
	if(objectlayers > 0) this->_objectlayers = objectlayers;
	if(masklayers > 0) this->_masklayers = masklayers;
	this->_collisionshape = new btSphereShape(((_width + _height) * 0.5f) * 0.5f * _size.x);

	this->_transform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 * You need set collision center to upright of half size of source
	 */
	this->_transform.setOrigin(
				btVector3(
						_position.x + _width * 0.5f,
						-(_position.y + _height * 0.5f),
						0.0f
				)
		);

	btVector3 localInertia(0, 0, 0);
	this->_collisionshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(this->_transform);
	btRigidBody::btRigidBodyConstructionInfo rigidbodyinfo(mass, mymotionstate, this->_collisionshape, localInertia);
	this->_rigidbody = new btRigidBody(rigidbodyinfo);


	this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	if(isstatic) {
		this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	}else {
		//Will prevent pass through walls but still will pass when get enough hight speed
		this->_rigidbody->setCcdMotionThreshold(0.01f);
	}



	this->_rigidbody->setWorldTransform(_transform);
	this->_rigidbody->setRestitution(0.1f);
	this->_rigidbody->setLinearFactor(btVector3(1.0f, 1.0f, 0.0f));
	this->_rigidbody->setAngularFactor(btVector3(0.0f, 0.0f, 1.0f));
	this->_id = gipBulletPhysics::Instance()->addPhysicObect(this, this->_objectlayers, this->_masklayers);
	this->_rigidbody->setUserIndex(this->_id);
}



gipPhysic2dCircle::gipPhysic2dCircle(bool isstatic, float mass, float radius, int objectlayers, int masklayers) {
	_isrenderobjectloaded = false;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
	this->_width = 200.f;
	this->_height = 200.f;
	this->_depth = 1.0f;
	this->_isstatic = isstatic;
	this->_mass = mass;
	this->is2d = true;

	if(objectlayers > 0) this->_objectlayers = objectlayers;
	if(masklayers > 0) this->_masklayers = masklayers;
	this->_size = glm::vec3(radius, radius, radius);
	//this->_rotation.setRotation(btVector3(0.0f, 0.0f, 1.0f),(btScalar)(-glm::radians(rotation.z)));


	this->_collisionshape = new btSphereShape(_width * 0.5f * _size.x);

	this->_transform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 * You need set collision center to upright of half size of source
	 */
	this->_transform.setOrigin(
			btVector3(
					_position.x + _width * 0.5f,
					-(_position.y + _height * 0.5f),
					0.0f
			)
	);

	btVector3 localInertia(0, 0, 0);
	this->_collisionshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(this->_transform);
	btRigidBody::btRigidBodyConstructionInfo rigidbodyinfo(mass, mymotionstate, this->_collisionshape, localInertia);
	this->_rigidbody = new btRigidBody(rigidbodyinfo);
	this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	if(isstatic) {
		this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	}else {
		//Will prevent pass through walls but still will pass when get enough hight speed
		this->_rigidbody->setCcdMotionThreshold(0.01f);
	}
	this->_rigidbody->setWorldTransform(_transform);
	this->_rigidbody->setRestitution(0.1f);
	this->_rigidbody->setLinearFactor(btVector3(1.0f, 1.0f, 0.0f));
	this->_rigidbody->setAngularFactor(btVector3(0.0f, 0.0f, 1.0f));
	this->_id = gipBulletPhysics::Instance()->addPhysicObect(this, this->_objectlayers, this->_masklayers);
	this->_rigidbody->setUserIndex(this->_id);

}


void gipPhysic2dCircle::draw() {
	//Wont be draw if ther is no any renderer object
	if(this->_renderobjecttype == OBJECTRENDERTYPE_IMAGE) {
		if(_isrenderobjectloaded) {
			_image->draw(_position.x, _position.y, _width, _height, _width * 0.5f, _height * 0.5f, gRadToDeg(-_rotation.getAxis().getZ() * _rotation.getAngle()));
		}
	} else 	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(_isrenderobjectloaded) {
			_model->draw();
 		}
	}
}

void gipPhysic2dCircle::setRendererObjectSize() {
	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(this->_isrenderobjectloaded) {
			_model->setScale(_size);
 		}
	}
}
void gipPhysic2dCircle::setRendererObjectPosition() {
	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(_isrenderobjectloaded) {
			_model->setPosition(_position);
 		}
	}
}
void gipPhysic2dCircle::setRendererObjectRotation() {
	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(_isrenderobjectloaded) {
			glm::vec3 newor;
			_rotation.getEulerZYX(newor.z, newor.y, newor.x);
			_model->setOrientation(newor);
 		}
	}
}

gipPhysic2dCircle::~gipPhysic2dCircle() {

}
