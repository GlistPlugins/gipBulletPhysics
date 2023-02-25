/*
 * gPhysic2dCircle.cpp
 *
 *  Created on: 21 Þub 2023
 *      Author: Remzi ÝÞÇÝ
 */

#include <gPhysic2dCircle.h>

/*
 * rotaion is degree format
 * size need to become between 0.04 and 100.000
 */
gPhysic2dCircle::gPhysic2dCircle(gImage* image, bool isstatic, float mass, float radius, glm::vec3 rotation, glm::vec3 position) {
	this->_image = image;
	this->_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_IMAGE;
	this->_width = _image->getWidth();
	this->_height = _image->getHeight();
	this->_isstatic = isstatic;
	this->_mass = mass;
	this->_size = glm::vec3(radius, radius, radius);
	this->_rotation.setRotation(btVector3(0.0f, 0.0f, 1.0f),(btScalar)(-glm::radians(rotation.z)));
	this->_position = position;

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
						position.x + _width * 0.5f,
						-(position.y + _height * 0.5f),
						0.0f
				)
		);

	if(rotation.z != 0.0f) {
		this->_transform.setRotation(this->_rotation);

	}
	btVector3 localInertia(0, 0, 0);
	if(mass > 0.0f) this->_collisionshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(this->_transform);
	btRigidBody::btRigidBodyConstructionInfo rigidbodyinfo(mass, mymotionstate, this->_collisionshape, localInertia);
	this->_rigidbody = new btRigidBody(rigidbodyinfo);


	this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	if(isstatic) {
		this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	}
	this->_rigidbody->setWorldTransform(_transform);
	//this->_rigidbody->setRestitution(0.81f);
	this->_rigidbody->setLinearFactor(btVector3(1.0f, 1.0f, 0.0f));
	this->_rigidbody->setAngularFactor(btVector3(0.0f, 0.0f, 1.0f));
	this->_id = gPhysic::Instance()->addPhysicObect(this);
	this->_rigidbody->setUserIndex(this->_id);
}


//This constructor need test and work
gPhysic2dCircle::gPhysic2dCircle(gModel* model, bool isstatic, float mass) {
	this->_model = model;
	_isrenderobjectloaded = true;
	this->_renderobjecttype = OBJECTRENDERTYPE_MODEL;
	this->_width = model->getScale().x;
	this->_height = model->getScale().y;
	this->_depth = model->getScale().z;
	this->_isstatic = false;
	this->_mass = mass;
	this->_size = model->getScale();
	glm::quat newQat = model->getOrientation();
	this->_rotation = btQuaternion(-newQat.y, newQat.x, newQat.z);
	this->_position = glm::vec3(model->getPosition().x, -model->getPosition().y, model->getPosition().z);

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
					_position.z + _depth * 0.5f
			)
	);
	if(_rotation.getAngle() != 0.0f) {
		this->_transform.setRotation(this->_rotation);

	}



	btVector3 localInertia(0, 0, 0);
	if(mass > 0.0f) this->_collisionshape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* mymotionstate = new btDefaultMotionState(this->_transform);
	btRigidBody::btRigidBodyConstructionInfo rigidbodyinfo(mass, mymotionstate, this->_collisionshape, localInertia);
	this->_rigidbody = new btRigidBody(rigidbodyinfo);


	this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	if(isstatic) {
		this->_rigidbody->setCollisionFlags(this->_rigidbody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	}
	this->_rigidbody->setWorldTransform(_transform);
	//this->_rigidbody->setRestitution(0.81f);
	this->_rigidbody->setLinearFactor(btVector3(1.0f, 1.0f, 0.0f));
	this->_rigidbody->setAngularFactor(btVector3(0.0f, 0.0f, 1.0f));
	this->_id = gPhysic::Instance()->addPhysicObect(this);
	this->_rigidbody->setUserIndex(this->_id);

}


gPhysic2dCircle::gPhysic2dCircle(bool isstatic, float mass, float radius, glm::vec3 rotation, glm::vec3 position) {
	_isrenderobjectloaded = false;
	this->_renderobjecttype = OBJECTRENDERTYPE_NONE;
	this->_width = radius;
	this->_height = radius;
	this->_depth = radius;
	this->_isstatic = isstatic;
	this->_mass = mass;
	this->_size = glm::vec3(1.0f, 1.0f, 1.0f);
	this->_rotation.setRotation(btVector3(0.0f, 0.0f, 1.0f),(btScalar)(-glm::radians(rotation.z)));
	this->_position = position;

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
					position.x + _width * 0.5f,
					-(position.y + _height * 0.5f),
					0.0f
			)
	);
	if(rotation.z > 0.0f) {
		this->_transform.setRotation(this->_rotation);

	}



	btVector3 localInertia(0, 0, 0);
	if(mass > 0.0f) this->_collisionshape->calculateLocalInertia(mass, localInertia);

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
	this->_id = gPhysic::Instance()->addPhysicObect(this);
	this->_rigidbody->setUserIndex(this->_id);

}


void gPhysic2dCircle::draw() {
	//Wont be draw if ther is no any renderer object
	if(this->_renderobjecttype == OBJECTRENDERTYPE_IMAGE) {
		if(_isrenderobjectloaded) {
			_image->draw(_position.x, _position.y, _width * _size.x, _height * _size.y, _width * 0.5f, _height * 0.5f, gRadToDeg(-_rotation.getAxis().getZ() * _rotation.getAngle()));
		}
	} else 	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(_isrenderobjectloaded) {
			_model->draw();
 		}
	}
}

void gPhysic2dCircle::setRendererObjectSize() {
	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(this->_isrenderobjectloaded) {
			_model->setScale(_size);
 		}
	}
}
void gPhysic2dCircle::setRendererObjectPosition() {
	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(_isrenderobjectloaded) {
			_model->setPosition(_position);
 		}
	}
}
void gPhysic2dCircle::setRendererObjectRotation() {
	if(this->_renderobjecttype == OBJECTRENDERTYPE_MODEL) {
		if(_isrenderobjectloaded) {
			glm::vec3 newor;
			_rotation.getEulerZYX(newor.z, newor.y, newor.x);
			_model->setOrientation(newor);
 		}
	}
}

gPhysic2dCircle::~gPhysic2dCircle() {

}
