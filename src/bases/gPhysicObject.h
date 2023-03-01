/*
 * gPhysicObject.h
 *
 *  Created on: 19 �ub 2023
 *      Author: Remzi ����
 *
 *  This class is base cals for defined object
 *	This class is abstructed class, don create referance directly from this class,
 *	Inherit from this if you create new object type
 */

#ifndef SRC_BASES_GPHYSICOBJECT_H_
#define SRC_BASES_GPHYSICOBJECT_H_

#include "bullet/btBulletDynamicsCommon.h"
#include "glm/glm.hpp"
#include <Functional>
#include "btBox2dShape.h"
#include "gImage.h"
#include "gModel.h"


class gPhysicObject {
	friend class gPhysic;
public:

	enum OBJECTRENDERTYPE {
		OBJECTRENDERTYPE_IMAGE,
		OBJECTRENDERTYPE_MODEL,
		OBJECTRENDERTYPE_NONE
	};

	/*
	 * Box = 0
	 * Sphere = 8
	 * Compound = 31
	 */
	enum transformtype {
		TRANSFORMTYPE_BOX = 0,
		TRANSFORMTYPE_SPHERE = 8,
		TRANSFORMTYPE_COMPOUND = 31,
	};

	/*
	 *	Call this function in Canvas draw method for showing content
	 */
	virtual void draw() = 0;



	/*
	 * Use this function for setting Oncollision fu
	 * use std::bind for parameter
	 */
	void setOnCollided(std::function<void(int, glm::vec3, glm::vec3)> onColl);

	//get with of conten(image, model etc)
	int getWidth();

	//get height of conten(image, model etc)
	int getHeight();

	/*
	 * set size of object 1 is default
	 * size need to be between 0.04 and 100000
	 */
	void setSize(float x, float y, float z = -1.0f);
	glm::vec3 getSize();

	void setTag(int newtag);
	int getTag();

	std::string getName();
	void setName(std::string newname);

	int getID();

	glm::vec3 getPosition();
	void setPosition(float x, float y, float z = 0.0f);

	glm::vec3 getOrigin();
	btQuaternion getRotation();

	//degree
	void setRotation(float degrex, float degrey, float degrez);
	//degree
	//This for 2d object and world
	void setRotation(float degrez);

	glm::vec3 getMassDirection();
	void setMass(glm::vec3 newmassdirection, float newmass = 1.0f);


	//Set base physic methods
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




	/*
	 * This function will be called when object collided with another object
	 *
	 * !!!
	 *  Dont call this function manuel
	 *  This function will be used by physic engine
	 */
	void onCollided(int targetobjectid, glm::vec3 selfcollpos, glm::vec3 targetcollpos);
	btTransform* getTransform();
	btCollisionShape* getCollisionShape();
	btRigidBody* getRigidBody();




protected:



	//This class is abstracted
	gPhysicObject();
	virtual ~gPhysicObject();

	/*
	 *This function referans for onCollided func
	 *This referance will connect Canvas function to physic object function
	 *You need use std::bind with setOncollided to using this referance
	 */
	std::function<void(int, glm::vec3, glm::vec3)> _onColl;




	gImage* _image;
	gMesh* _model;

	btTransform _transform;
	btCollisionShape* _collisionshape;
	btRigidBody* _rigidbody;

	/*
	 * This function is for physic engine dont use manualy
	 * use radyan
	 */
	void setRotation(btQuaternion newrotation);
	/*
	 * This function is for physic engine dont use manualy
	 */
	void updatePositionVariable();

	/*
	 * This function is for physic engine dont use manualy
	 */
	void updateRotationVariable();

	/*
	 *This function is for chil object to set size
	 */
	virtual void setRendererObjectSize() = 0;

	/*
	 *This function is for chil object to set position
	 */
	virtual void setRendererObjectPosition() = 0;

	/*
	 *This function is for chil object to set rotation
	 */
	virtual void setRendererObjectRotation() = 0;

	//id is comes from physic engine object list id
	int _id = -1;
	int _width = 200;
	int _height = 200;
	int _depth = 1.0f;
	glm::vec3 _size = glm::vec3(1.0f, 1.0f, 1.0f);

	glm::vec3 _position = glm::vec3(0.0f, 0.0f, 0.0f);
	btQuaternion _rotation;
	glm::vec3 _massdirection;
	float _mass = 0.0f;
	float _friction, _rollingfriction, _spinningFriction;
	glm::vec3 _anisotropicfriction;
	int _anistropicfrictionmode;

	bool _isOnCollidedFuncSetted = false;

	OBJECTRENDERTYPE _renderobjecttype = OBJECTRENDERTYPE_IMAGE;

	bool _isrenderobjectloaded = false;

	/*
	 * Is object 2d or 3d
	 * true means 2d
	 * false mean 3d
	 */
	bool is2d = true;


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
	std::string objectname = "new object";

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
	int _masklayers = 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3 | 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7 | 1 << 8 | 1 << 9
			| 1 << 10 | 1 << 11 | 1 << 12 | 1 << 13 | 1 << 14 | 1 << 15 | 1 << 16 | 1 << 17 | 1 << 18 | 1 << 19
			| 1 << 20 | 1 << 21 | 1 << 22;
private:


};
#endif /* SRC_BASES_GPHYSICOBJECT_H_ */
