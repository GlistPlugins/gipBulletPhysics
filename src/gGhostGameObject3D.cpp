/*
 * gGhostGameObject3D.cpp
 *
 *
 *  Created on: 12 Mar 2023
 *      Author: Remzi ISCI
 *
 *  This class inherit from gipPhysicObject
 *  This clas uses 2d coordinate system
 *  Layers are bit wise
 *  Rotaions have been setted according degree format
 */

#include <gGhostGameObject3D.h>

gGhostGameObject3D::gGhostGameObject3D(gipBulletPhysics* physicworld) {
	this->_physicworld = physicworld;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
	this->_coordinatetype = COORDINATE::COORDINATE3D;
	this->_collsionobjecttype = COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_GHOST;
	 _sizecollider = glm::vec3(1.0f, 1.0f, 1.0f);
	 _width = 1.0f;
	 _height = 1.0f;
	 _depth = 1.0f;
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
				_position.x,
				-_position.y,
				_position.z
				)
	);
	this->_ghostobject = new btGhostObject();
	this->_ghostobject->setCollisionShape(this->_collisionshape);
	this->_ghostobject->setWorldTransform(this->_transform);
	this->_ghostobject->setCollisionFlags(this->_ghostobject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

	this->_id = physicworld->addPhysicObect(this, this->_objectlayers, this->_masklayers);
	this->_ghostobject->setUserIndex(this->_id);

}

gGhostGameObject3D::~gGhostGameObject3D() {
	// TODO Auto-generated destructor stub
}
