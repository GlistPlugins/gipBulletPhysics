/*
 * gipBaseGameObject.h
 *
 *  Created on: 5 03 2023
 *      Author: Remzi ISCI
 */

#ifndef SRC_GIPBASEGAMEOBJECT_H_
#define SRC_GIPBASEGAMEOBJECT_H_

#include "bullet/btBulletCollisionCommon.h"
#include "bullet/btBulletDynamicsCommon.h"
#include "glm/glm.hpp"
#include <functional>
#include "gModel.h"
#include "gMesh.h"
#include "gImage.h"

class gipBulletPhysics;

class gipBaseGameObject;
using OnCollidedFunction = std::function<void(gipBaseGameObject*, glm::vec3, glm::vec3)>;

class gipBaseGameObject {
public:
	friend class gipBulletPhysics;

	enum COORDINATE { COORDINATE2D = 0, COORDINATE3D = 1 };
	/*
	 * Tranform type for collision shapes
	 *
	 * Box = 0
	 * Sphere = 8
	 * Compound = 31
	 */
	enum TRANSFORMTYPE {
		TRANSFORMTYPE_BOX = 0,
		TRANSFORMTYPE_SPHERE = 8,
		TRANSFORMTYPE_COMPOUND = 31,
	};

	enum SHAPETYPE {
		SHAPETYPE_BOX,
		SHAPETYPE_SPHERE,
		SHAPETYPE_CYLINDER,
		SHAPETYPE_CAPSULE,
		SHAPETYPE_CONE
	};

	enum OBJECTRENDERTYPE {
		OBJECTRENDERTYPE_IMAGE,
		OBJECTRENDERTYPE_MODEL,
		OBJECTRENDERTYPE_MESH,
		OBJECTRENDERTYPE_NONE
	};

	enum COLLISIONOBJECTTYPE {
		COLLISIONOBJECTTYPE_RIGIDBODY,
		COLLISIONOBJECTTYPE_GHOST
	};
	/*
	* Layers are bitwise varialbles
	 * You can use multiple layers together
	 * Use | (Bite or operator) to use multiple layer together
	 * etc : LAYER1 | LAYER2 | LAYER12 | LAYER22
	 */
	enum COLLISIONLAYERS {
		LAYERNONMEMBER = -1, //Dont use this, this just for check
		LAYER0 = 1 << 0,	//Dont collide layer
		LAYER1 = 1 << 1,	//Default collide layer
		LAYER2 = 1 << 2,
		LAYER3 = 1 << 3,
		LAYER4 = 1 << 4,
		LAYER5 = 1 << 5,
		LAYER6 = 1 << 6,
		LAYER7 = 1 << 7,
		LAYER8 = 1 << 8,
		LAYER9 = 1 << 9,
		LAYER10 = 1 << 10,
		LAYER11 = 1 << 11,
		LAYER12 = 1 << 12,
		LAYER13 = 1 << 13,
		LAYER14 = 1 << 14,
		LAYER15 = 1 << 15,
		LAYER16 = 1 << 16,
		LAYER17 = 1 << 17,
		LAYER18 = 1 << 18,
		LAYER19 = 1 << 19,
		LAYER20 = 1 << 20,
		LAYER21 = 1 << 21,
		LAYER22 = 1 << 22
	};

	gipBaseGameObject();
	virtual ~gipBaseGameObject();

	/*
	 * Use this function for setting Oncollision fu
	 * use std::bind for parameter
	 */
	void setOnCollided(OnCollidedFunction func);

	//get with of conten(image, model etc)
	int getWidth();

	//get height of conten(image, model etc)
	int getHeight();

	void setTag(int newtag);
	int getTag();

	std::string getName();
	void setName(std::string newname);

	glm::vec3 getPosition();
	void setPosition(float x, float y, float z = 0.0f);

	//degree
	glm::vec3 getRotation();
	void setRotation(float degrex, float degrey, float degrez);

	//degree
	//This for 2d object and world
	void setRotation2D(float degrez);

	//Set offset collider fromorigin of object
	void setColliderOffset(float offsetx, float offsety, float offsetz = 0);
	glm::vec3 getColliderOffset();

	/*
	 * set size of object 1 is default
	 * size need to be between 0.04 and 100000
	 * For BOX : each axis means one egde
	 * For SPHERE : all axis means radius can be same
	 * for Cylinder X means radius, y means height
	 * for Cone x means raadius, y means height
	 */
	void setColliderSize(float x, float y, float z = -1.0f);
	glm::vec3 getColliderSize();
	void setObjectSize(float width, float height, float depth = -1.0f);
	glm::vec3 getObjectSize();

	/*
	 * Set lock size of object and collider
	 */
	void setIsSizeLocked(bool islocked);
	bool getIsSizeLocked();


	//Call this function for changing shape type
	void setShapeType(SHAPETYPE shapetype);
	int getShapeType();

	//You can use COLLISIONLAYERS enum
	//Object own layers, layer are bitwise
	void setObjectLayers(int layers);
	//Set target layers whic we want collide with this object, layers asre bitewise
	void setMaskLayers(int masklayers);

	glm::vec3 getOrigin();

	//Set base physic methods ---------------------------------------------------------------
	float getMass();
	void setMass(float newmass);
	void setFriction(float newvalue);
	float getFriction();
	void setRollingFriction(float newvalue);
	float getRollingFriction();
	void setSpinningFriction(float newvalue);
	float getSpinnigFriction();
	void setAnisotropicFriction(glm::vec3 newvalue, int anisotropicfrictionmode = 1);


	//value should become between 0 and 1
	void setBounce(float newvalue = 0.0f);


	// These apply methods should be used in update method.
	void applyCentralForce(glm::vec3 forcevalue);
	void applyCentralImpulse(glm::vec3 impulsevalue);
	void applyForce(glm::vec3 forcevalue,glm::vec3 forcepos);
	void applyImpulse(glm::vec3 impulsevalue,glm::vec3 impulsepos);
	void applyTorque(glm::vec3 torquevalue);
	void applyTorqueImpulse(glm::vec3 torquevalue);

	//----------------------------------------------------------------------------------------


	btTransform* getTransform();
	btCollisionShape* getCollisionShape();
	btRigidBody* getRigidBody();

	void destroy();

	bool getIsStatic();

	void updateSingleAABB();

	void updateObjectLayers();

protected:

	/*
	 * This function will be called when object collided with another object
	 *
	 * !!!
	 *  Dont call this function manuel
	 *  This function will be used by physic engine
	 */
	void warnCollided(gipBaseGameObject* target, glm::vec3 selfcollpos, glm::vec3 targetcollpos);

	/*
	 * This function is for physic engine dont use manualy
	 */
	void updatePositionVariable();

	/*
	 * This function is for physic engine dont use manualy
	 */
	void updateRotationVariable();

	void setSelfIndex(size_t index);

	/*
	 *This function referans for onCollided func
	 *This referance will connect Canvas function to physic object function
	 *You need use std::bind with setOncollided to using this referance
	 */
	OnCollidedFunction collidedcallback;

	//Referances--------------------------------
	size_t _selfindex = 0;
	gImage* _image;
	gModel* _model;
	gMesh* _mesh;
	gipBulletPhysics* _physicworld;
	btTransform _transform;
	btCollisionShape* _collisionshape;
	btRigidBody* _rigidbody;
	btCollisionObject* _ghostobject;
	//----------------------------------------------


	//Game world properties-------------------------
	int _width = 200;
	int _height = 200;
	int _depth = 1.0f;
	glm::vec3 _sizecollider = glm::vec3(200.0f, 200.0f, 1.0f);
	bool _isrenderersizelocked = true;
	glm::vec3 _position = glm::vec3(0.0f, 0.0f, 0.0f);
	btQuaternion _rotation;
	glm::vec3 _colliderofset = glm::vec3(0.0f, 0.0f, 0.0f);
	//------------------------------------------------

	//Physic properties-------------------------------

	float _mass = 0.0f;
	float _friction = 0.0f;
	float _rollingfriction = 0.0f;
	float _spinningFriction = 0.0f;
	glm::vec3 _anisotropicfriction;
	int _anistropicfrictionmode;
	//-------------------------------------------------



	OBJECTRENDERTYPE _renderobjecttype = OBJECTRENDERTYPE_IMAGE;
	SHAPETYPE _shapetype = SHAPETYPE_BOX;

	//True if renderer object setted
	bool _isrenderobjectloaded = false;

	/*
	 * 0 = 2D
	 * 1 = 3D
	 */
	COORDINATE _coordinatetype = COORDINATE2D;

	/*
	 * This function is for chil object to set rotation
	 */
	bool _isstatic = true;

	/*
	 * Tag is for grouping objects
	 */
	int _tag = 0;

	/*
	 * for naming object
	 */
	std::string _objectname = "new object";

	/*
	 * object layer has
	 *
	 */
	int _objectlayers = 1<<1;

	/*
	 * mask layer, object will collide which object has this layers
	 * Layer is bitwise
	 * All layer  collide for default
	 */
	int _masklayers = 1<<1 |1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7 | 1<<8 | 1<<9 | 1<<10
			| 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15 | 1<<16 | 1<<17 | 1<<18 | 1<<19 | 1<<20
			| 1<<21 | 1<<22;
	COLLISIONOBJECTTYPE _collsionobjecttype = COLLISIONOBJECTTYPE::COLLISIONOBJECTTYPE_RIGIDBODY;

	const glm::quat _resetquat = glm::quatLookAt(glm::vec3(0,0,-1), glm::vec3(0,1,0));
};


#endif /* SRC_GIPBASEGAMEOBJECT_H_ */
