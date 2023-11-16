/*
 * gGhostGameObject2D.cpp
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ISCI
 */

#include <gGhostImageGameObject.h>

gGhostImageGameObject::gGhostImageGameObject(gipBulletPhysics* physicworld) {
	this->_physicworld = physicworld;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
	this->_coordinatetype = COORDINATE::COORDINATE2D;
	this->_collsionobjecttype = COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_GHOST;
	btTransform temptensform;
	this->_collisionshape = new btBoxShape(btVector3(_sizecollider.x,_sizecollider.y, _sizecollider.z));
	this->_transform.setIdentity();

	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	this->_collisionshape->calculateLocalInertia(_mass, localInertia);

		/*
		 * The Glist Engine references the top left corner for object positions;
		 * but the bullet3 library references the center of transform.
		 * Glist engine Y axis is opposite to bullet physic y axis nned to convert y axis by multiply -1
		 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
		 */
	this->_transform.setOrigin(
			btVector3(
				_position.x + _sizecollider.x * 0.5f,
				-(_position.y + _sizecollider.y * 0.5f),
				0.0f
				)
	);
	this->_ghostobject = new btGhostObject();
	this->_ghostobject->setCollisionShape(this->_collisionshape);
	this->_ghostobject->setWorldTransform(this->_transform);
	this->_ghostobject->setCollisionFlags(this->_ghostobject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

	this->_id = physicworld->addPhysicObject(this, this->_objectlayers, this->_masklayers);
	this->_ghostobject->setUserIndex(this->_id);

}


void gGhostImageGameObject::loadImage(std::string imagepath) {
	this->_image = new gImage();
	this->_image->loadImage(imagepath);
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_IMAGE;
	setObjectSize(this->_image->getWidth(), this->_image->getHeight());
}

void gGhostImageGameObject::load(std::string fullpath) {
	this->_image = new gImage();
	this->_image->load(fullpath);
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_IMAGE;
	setObjectSize(this->_image->getWidth(), this->_image->getHeight());
}

void gGhostImageGameObject::setImage(gImage* sourceimage) {
	this->_image = sourceimage;
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_IMAGE;
	setObjectSize(this->_image->getWidth(), this->_image->getHeight());
}

void gGhostImageGameObject::clearImage() {
	this->_isrenderobjectloaded = false;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
}


void gGhostImageGameObject::draw() {
	//Wont be draw if ther is no any renderer object
	if(this->_renderobjecttype == OBJECTRENDERTYPE_IMAGE) {
		if(_isrenderobjectloaded) {
			_image->draw(_position.x, _position.y, _width, _height, _width * 0.5f, _height * 0.5f, gRadToDeg(-_rotation.getAxis().getZ() * _rotation.getAngle()));
		}
	}
}

gGhostImageGameObject::~gGhostImageGameObject() {
	// TODO Auto-generated destructor stub
}


