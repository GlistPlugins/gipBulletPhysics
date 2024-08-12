/*
 * gImageGameObject.cpp
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ISCI
 */

#include <gImageGameObject.h>

gImageGameObject::gImageGameObject(gipBulletPhysics* physicworld) {
	this->_physicworld = physicworld;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
	this->_coordinatetype = COORDINATE::COORDINATE2D;
	btTransform temptensform;
	this->_collisionshape = new btBoxShape(btVector3(_sizecollider.x,_sizecollider.y, _sizecollider.z));
	this->_transform.setIdentity();

	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	this->_collisionshape->calculateLocalInertia(_mass, localInertia);

		/*
		 * The Glist Engine references the top left corner for object positions;
		 * but the bullet3 library references the center of transform.
		 * Glist engine Y axis is opposite to bullet physics y axis we need to convert y axis by multiplying by -1
		 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
		 */
	this->_transform.setOrigin(
			btVector3(
				_position.x + _sizecollider.x * 0.5f,
				-(_position.y + _sizecollider.y * 0.5f),
				0.0f
				)
	);

	//Using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(this->_transform);
	btRigidBody::btRigidBodyConstructionInfo rigidbodyinfo(_mass, mymotionstate, this->_collisionshape, localInertia);
	this->_rigidbody = new btRigidBody(rigidbodyinfo);
	this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	//If static is choosen then add static flag to object to improve perfomance
	if(this->_isstatic) {
		this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
	} else {
		//Will prevent passing through walls but will still pass when it gets fast enough
		this->_rigidbody->setCcdMotionThreshold(0.01f);
	}
	this->_rigidbody->setActivationState(4);

	//Setting factors according to 2d world, to make it move in 2d and rotate around only 1 axis
	this->_rigidbody->setLinearFactor(btVector3(1.0f, 1.0f, 0.0f));
	//To make this rotate IN x and y axis, rotate it AROUND z axis
	this->_rigidbody->setAngularFactor(btVector3(0.0f, 0.0f, 1.0f));
	physicworld->addPhysicObject(this, this->_objectlayers, this->_masklayers);
}


void gImageGameObject::loadImage(std::string imagepath) {
	this->_image = new gImage();
	this->_image->loadImage(imagepath);
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_IMAGE;
	setObjectSize(this->_image->getWidth(), this->_image->getHeight());
}

void gImageGameObject::load(std::string fullpath) {
	this->_image = new gImage();
	this->_image->load(fullpath);
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_IMAGE;
	setObjectSize(this->_image->getWidth(), this->_image->getHeight());
}

void gImageGameObject::setImage(gImage* sourceimage) {
	this->_image = sourceimage;
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_IMAGE;
	setObjectSize(this->_image->getWidth(), this->_image->getHeight());
}

void gImageGameObject::clearImage() {
	this->_isrenderobjectloaded = false;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
}


void gImageGameObject::draw() {
	// Will not work if there isn't any renderer object
	if(this->_renderobjecttype == OBJECTRENDERTYPE_IMAGE) {
		if(_isrenderobjectloaded) {
			_image->draw(_position.x, _position.y, _width, _height, _width * 0.5f, _height * 0.5f, gRadToDeg(-_rotation.getAxis().getZ() * _rotation.getAngle()));
		}
	}
}



gImageGameObject::~gImageGameObject() {
	// TODO Auto-generated destructor stub
}
