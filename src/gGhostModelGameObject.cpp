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

#include <gGhostModelGameObject.h>

gGhostModelGameObject::gGhostModelGameObject(gipBulletPhysics* physicworld) {
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

void gGhostModelGameObject::loadModel(std::string modelpath) {
	this->_model = new gModel();
	this->_model->loadModel(modelpath);
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_MODEL;
	 _width = _model->getScale().x;
	 _height = _model->getScale().y;
	 _depth = _model->getScale().z;
	setObjectSize(this->_model->getScale().x, this->_model->getScale().y, this->_model->getScale().z);
}

void gGhostModelGameObject::load(std::string fullpath) {
	this->_model = new gModel();
	this->_model->load(fullpath);
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_MODEL;
	 _width = _model->getScale().x;
	 _height = _model->getScale().y;
	 _depth = _model->getScale().z;
	setObjectSize(this->_model->getScale().x, this->_model->getScale().y, this->_model->getScale().z);
}

void gGhostModelGameObject::setModel(gModel* sourcemodel) {
	this->_model = sourcemodel;
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_MODEL;
	 _width = _model->getScale().x;
	 _height = _model->getScale().y;
	 _depth = _model->getScale().z;
	setObjectSize(this->_model->getScale().x, this->_model->getScale().y, this->_model->getScale().z);
}

void gGhostModelGameObject::setMesh(gMesh* sourcemesh) {
	this->_mesh = sourcemesh;
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_MESH;
	 _width = _mesh->getScale().x;
	 _height = _mesh->getScale().y;
	 _depth = _mesh->getScale().z;
	setObjectSize(this->_mesh->getScale().x, this->_mesh->getScale().y, this->_mesh->getScale().z);
}

void gGhostModelGameObject::clearModel() {
	this->_isrenderobjectloaded = false;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
}

void gGhostModelGameObject::clearMesh() {
	this->_isrenderobjectloaded = false;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
}

void gGhostModelGameObject::draw() {
	//Wont be draw if ther is no any renderer object
	if(_isrenderobjectloaded) {
		if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
			this->_model->draw();
		} else if(this->_renderobjecttype == OBJECTRENDERTYPE_MESH) {
			this->_mesh->draw();
		}
	}
}

gGhostModelGameObject::~gGhostModelGameObject() {
	// TODO Auto-generated destructor stub
}
