/*
 * gPhysic2dBox.cpp
 *
 *  Created on: 21 02 2023
 *      Author: Remzi ISCI
 */

#include <gipPhysic2dBox.h>


/*
 * rotaion is degree format
 * size need to become between 0.04 and 100.000
 */
gipPhysic2dBox::gipPhysic2dBox(gImage* image, bool isstatic,float mass, int objectlayers, int masklayers) {
	this->_image = image;
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_IMAGE;
	this->_isstatic = isstatic;
	this->_mass = mass;
	this->_width = image->getWidth();
	this->_height = image->getHeight();
	this->is2d = true;
	if(objectlayers > 0) this->_objectlayers = objectlayers;
	if(masklayers > 0) this->_masklayers = masklayers;

	//this->_rotation.setRotation(btVector3(0.0f, 0.0f, 1.0f),(btScalar)(-glm::radians(newrotation.z)));
	this->_collisionshape = new btBoxShape(btVector3(_width * 0.5f * _size.x, _height * 0.5f * _size.y, _depth));

	this->_transform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the center of transform.
	 * Glist engine Y axis is opposite to bullet physic y axis nned to convert y axis by multiply -1
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
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
	btRigidBody::btRigidBodyConstructionInfo rigidbodyinfo(_mass, mymotionstate, this->_collisionshape, localInertia);
	this->_rigidbody = new btRigidBody(rigidbodyinfo);
	this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	//if choosed static then add static flag to object will improve perfomance
	if(isstatic) {
		this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	}else {
		//Will prevent pass through walls but still will pass when get enough hight speed
		this->_rigidbody->setCcdMotionThreshold(0.01f);
	}

	this->_rigidbody->setWorldTransform(_transform);
	this->_rigidbody->setRestitution(0.1f);
	//Set factor according 2d world can move just 2 axis and rotat 1 axis
	this->_rigidbody->setLinearFactor(btVector3(1.0f, 1.0f, 0.0f));
	this->_rigidbody->setAngularFactor(btVector3(0.0f, 0.0f, 1.0f));

	this->_id = gipBulletPhysics::Instance()->addPhysicObect(this, this->_objectlayers, this->_masklayers);
	this->_rigidbody->setUserIndex(this->_id);

}

gipPhysic2dBox::gipPhysic2dBox( int width, int height, bool isstatic, float mass, int objectlayers, int masklayers) {
	_isrenderobjectloaded = false;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
	this->_width = std::max(1, width);
	this->_height = std::max(1, height);
	this->_isstatic = isstatic;
	this->_mass = mass;
	this->is2d = true;
	if(objectlayers > 0) this->_objectlayers = objectlayers;
	if(masklayers > 0) this->_masklayers = masklayers;

	//this->_rotation.setRotation(btVector3(0.0f, 0.0f, 1.0f),(btScalar)(-glm::radians(rotation.z)));
	this->_collisionshape = new btBoxShape(btVector3(_width * 0.5f * _size.x, _height * 0.5f * _size.y, _depth));

	this->_transform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the center of transform.
	 * Glist engine Y axis is opposite to bullet physic y axis nned to convert y axis by multiply -1
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 */
	this->_transform.setOrigin(
			btVector3(
					_position.x + _width * 0.5f,
					-(_position.y + _height * 0.5f),
					0.0f
			)
	);


	if(_rotation.z() > 0.0f) {
		this->_transform.setRotation(this->_rotation);
	}
	btVector3 localInertia(0, 0, 0);
	this->_collisionshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(this->_transform);
	btRigidBody::btRigidBodyConstructionInfo rigidbodyinfo(mass, mymotionstate, this->_collisionshape, localInertia);
	this->_rigidbody = new btRigidBody(rigidbodyinfo);


	this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	//if choosed static then add static flag to object will improve perfomance
	if(isstatic) {
		this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	} else {
		//Will prevent pass through walls but still will pass when get enough hight speed
		this->_rigidbody->setCcdMotionThreshold(0.01f);
	}

	this->_rigidbody->setWorldTransform(_transform);
	//Set factor according 2d world can move just 2 axis and rotat 1 axis
	this->_rigidbody->setLinearFactor(btVector3(1.0f, 1.0f, 0.0f));
	this->_rigidbody->setAngularFactor(btVector3(0.0f, 0.0f, 1.0f));
	this->_id = gipBulletPhysics::Instance()->addPhysicObect(this, this->_objectlayers, this->_masklayers);
	this->_rigidbody->setUserIndex(this->_id);

}
void gipPhysic2dBox::draw() {
	//Wont be draw if ther is no any renderer object
	if(this->_renderobjecttype == OBJECTRENDERTYPE_IMAGE) {
		if(_isrenderobjectloaded) {
			_image->draw(_position.x, _position.y, _width, _height, _width * 0.5f, _height * 0.5f, gRadToDeg(-_rotation.getAxis().getZ() * _rotation.getAngle()));
		}
	}
}

void gipPhysic2dBox::setRendererObjectSize() {}

void gipPhysic2dBox::setRendererObjectPosition() {}

void gipPhysic2dBox::setRendererObjectRotation() {}

gipPhysic2dBox::~gipPhysic2dBox() {}
