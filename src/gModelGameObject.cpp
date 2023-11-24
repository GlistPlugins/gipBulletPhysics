/*
 * gModelGameObject.cpp
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ISCI
 */


#include <gModelGameObject.h>

gModelGameObject::gModelGameObject(gipBulletPhysics* physicworld) {
	this->_physicworld = physicworld;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
	this->_coordinatetype = COORDINATE::COORDINATE3D;
	 _sizecollider = glm::vec3(1.0f, 1.0f, 1.0f);
	 _width = 1.0f;
	 _height = 1.0f;
	 _depth = 1.0f;
	btTransform temptensform;
	this->_collisionshape = new btBoxShape(btVector3(_sizecollider.x, _sizecollider.y, _sizecollider.z));
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
				-(_position.y),
				_position.z
				)
	);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(this->_transform);
	btRigidBody::btRigidBodyConstructionInfo rigidbodyinfo(_mass, mymotionstate, this->_collisionshape, localInertia);
	this->_rigidbody = new btRigidBody(rigidbodyinfo);
	this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );
	//if choosed static then add static flag to object will improve perfomance
	if(this->_isstatic) {
		this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	} else {
		//Will prevent pass through walls but still will pass when get enough hight speed
		this->_rigidbody->setCcdMotionThreshold(0.01f);
	}


	this->_rigidbody->setActivationState(4);
	physicworld->addPhysicObject(this, _objectlayers, _masklayers);
}


void gModelGameObject::loadModel(std::string modelpath) {
	this->_model = new gModel();
	this->_model->loadModel(modelpath);
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_MODEL;
	 _width = _model->getScale().x;
	 _height = _model->getScale().y;
	 _depth = _model->getScale().z;
	setObjectSize(this->_model->getScale().x, this->_model->getScale().y, this->_model->getScale().z);
}

void gModelGameObject::load(std::string fullpath) {
	this->_model = new gModel();
	this->_model->load(fullpath);
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_MODEL;
	 _width = _model->getScale().x;
	 _height = _model->getScale().y;
	 _depth = _model->getScale().z;
	setObjectSize(this->_model->getScale().x, this->_model->getScale().y, this->_model->getScale().z);
}

void gModelGameObject::setModel(gModel* sourcemodel) {
	this->_model = sourcemodel;
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_MODEL;
	 _width = _model->getScale().x;
	 _height = _model->getScale().y;
	 _depth = _model->getScale().z;
	setObjectSize(this->_model->getScale().x, this->_model->getScale().y, this->_model->getScale().z);
}

void gModelGameObject::setMesh(gMesh* sourcemesh) {
	this->_mesh = sourcemesh;
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_MESH;
	 _width = _mesh->getScale().x;
	 _height = _mesh->getScale().y;
	 _depth = _mesh->getScale().z;
	setObjectSize(this->_mesh->getScale().x, this->_mesh->getScale().y, this->_mesh->getScale().z);
}

void gModelGameObject::clearModel() {
	this->_isrenderobjectloaded = false;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
}

void gModelGameObject::clearMesh() {
	this->_isrenderobjectloaded = false;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
}

void gModelGameObject::draw() {
	//Wont be draw if ther is no any renderer object
	if(_isrenderobjectloaded) {
		if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
			this->_model->draw();
		} else if(this->_renderobjecttype == OBJECTRENDERTYPE_MESH) {
			this->_mesh->draw();
		}
	}
}


gModelGameObject::~gModelGameObject() {
	// TODO Auto-generated destructor stub
}


