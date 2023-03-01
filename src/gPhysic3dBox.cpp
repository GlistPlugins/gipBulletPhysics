/*
 * gPhysic3dBox.cpp
 *
 *  Created on: 28 �ub 2023
 *      Author: Remzi ����
 */

#include <gPhysic3dBox.h>


/*
 * rotaion is degree format
 * size need to become between 0.04 and 100.000
 */
gPhysic3dBox::gPhysic3dBox(gMesh* model, bool isstatic, float mass, int objectlayers, int masklayers) {
	this->_model = model;
	_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_MODEL;
	this->_width = model->getScale().x;
	this->_height = model->getScale().y;
	this->_depth = model->getScale().z;
	this->_isstatic = false;
	this->_mass = -mass;
	this->is2d = false;
	if(objectlayers > 0) this->_objectlayers = objectlayers;
	if(masklayers > 0) this->_masklayers = masklayers;
	glm::quat newQat = model->getOrientation();
	this->_rotation = btQuaternion(-gDegToRad(newQat.y), gDegToRad(newQat.x), gDegToRad(newQat.z));
	this->_position = glm::vec3(model->getPosition().x, model->getPosition().y, model->getPosition().z);

	this->_collisionshape = new btBoxShape(btVector3(_width * _size.x, _height * _size.y, _depth * _size.z));

	this->_transform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 * You need set collision center to upright of half size of source
	 */

	this->_transform.setOrigin(
			btVector3(
					_position.x,
					-_position.y,
					_position.z
			)
	);
	if(_rotation.getAngle() != 0.0f) {
		this->_transform.setRotation(this->_rotation);

	}



	btVector3 localInertia(0, 0, 0);
	if(isstatic == false) this->_collisionshape->calculateLocalInertia(this->_mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(this->_transform);
	btRigidBody::btRigidBodyConstructionInfo rigidbodyinfo(this->_mass, mymotionstate, this->_collisionshape, localInertia);
	this->_rigidbody = new btRigidBody(rigidbodyinfo);


	this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	if(isstatic) {
		this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	}
	this->_rigidbody->setWorldTransform(_transform);
	//this->_rigidbody->setRestitution(1.0f);
	//this->_rigidbody->setFriction(0.0f);
	//this->_rigidbody->setRollingFriction(0.f);
	this->_rigidbody->setLinearFactor(btVector3(1.0f, 1.0f, 0.0f));
	this->_rigidbody->setAngularFactor(btVector3(0.0f, 0.0f, 1.0f));
	this->_id = gPhysic::Instance()->addPhysicObect(this, this->_objectlayers, this->_masklayers);
	this->_rigidbody->setUserIndex(this->_id);

}

gPhysic3dBox::gPhysic3dBox( int width, int height, int depth, bool isstatic, float mass, int objectlayers, int masklayers) {

	_isrenderobjectloaded = false;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
	this->_width = std::max(1, width);
	this->_height = std::max(1, height);
	this->_depth = std::max(1, depth);
	this->_isstatic = isstatic;
	this->_mass = mass;
	this->is2d = false;
	if(objectlayers > 0) this->_objectlayers = objectlayers;
	if(masklayers > 0) this->_masklayers = masklayers;

	//this->_rotation.setRotation(btVector3(0.0f, 0.0f, 1.0f),(btScalar)(-glm::radians(rotation.z)));
	this->_collisionshape = new btBoxShape(btVector3(_width * 0.5f * _size.x, _height * 0.5f * _size.y, 1.0f));

	this->_transform.setIdentity();
	/*
	 * The Glist Engine references the top left corner for object positions;
	 * but the bullet3 library references the bottom left for object positions.
	 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
	 * You need set collision center to upright of half size of source
	 */
	this->_transform.setOrigin(
			btVector3(
					_position.x,
					-_position.y,
					_position.z
			)
	);
	if(_rotation.z() > 0.0f) {
		this->_transform.setRotation(this->_rotation);
	}
	btVector3 localInertia(0, 0, 0);
	if(isstatic == false) this->_collisionshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(this->_transform);
	btRigidBody::btRigidBodyConstructionInfo rigidbodyinfo(mass, mymotionstate, this->_collisionshape, localInertia);
	this->_rigidbody = new btRigidBody(rigidbodyinfo);


	this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	if(isstatic) {
		this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	}
	this->_rigidbody->setWorldTransform(_transform);
	this->_rigidbody->setLinearFactor(btVector3(1.0f, 1.0f, 0.0f));
	this->_rigidbody->setAngularFactor(btVector3(0.0f, 0.0f, 1.0f));
	this->_id = gPhysic::Instance()->addPhysicObect(this, this->_objectlayers, this->_masklayers);
	this->_rigidbody->setUserIndex(this->_id);

}
void gPhysic3dBox::draw() {

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

void gPhysic3dBox::setRendererObjectSize() {
	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(_isrenderobjectloaded) {
			_model->setScale(_size);
 		}
	}
}
void gPhysic3dBox::setRendererObjectPosition() {
	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(_isrenderobjectloaded) {
			_model->setPosition(_position.x - + _width * 0.5f,
					-(_position.y - _height * 0.5f),
					(_position.z - _depth * 0.5f));
 		}
	}
}
void gPhysic3dBox::setRendererObjectRotation() {
	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(_isrenderobjectloaded) {
			glm::vec3 newor;
			newor.x = gRadToDeg(_rotation.getAxis().getX() * _rotation.getAngle());
			newor.y = gRadToDeg(_rotation.getAxis().getY() * _rotation.getAngle());
			newor.z = gRadToDeg(-_rotation.getAxis().getZ() * _rotation.getAngle());
			_model->setOrientation(newor);

 		}
	}
}

gPhysic3dBox::~gPhysic3dBox() {
}
