/*
 * GameCanvas.cpp
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ISCÝ
 */


#ifndef GAMECANVAS_H_
#define GAMECANVAS_H_

#include "gBaseCanvas.h"
#include "gApp.h"
#include "gipBulletPhysics.h"
#include "gImageGameObject.h"
#include "gModelGameObject.h"


class GameCanvas : public gBaseCanvas {
public:
	GameCanvas(gApp* root);
	virtual ~GameCanvas();

	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void charPressed(unsigned int codepoint);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseScrolled(int x, int y);
	void mouseEntered();
	void mouseExited();
	void windowResized(int w, int h);

	void showNotify();
	void hideNotify();

	void onCollided(int targetid);

private:
	int static const RIGIDWORLD = 0;
	int static const SOFTRIGIDWORLD = 1;

	gApp* root;
    gImage sky;
    gImage mountain;
    gImage ground;
    gImage ramp;
    gImage ball;
    gImage gameIcon;
    //gModel ball3d;

    gImageGameObject* groundobject;
    gImageGameObject* rampobject;
    gImageGameObject* ballobject;
    gImageGameObject* softballobject;
    gImageGameObject* gameiconobject;

    //gModelGameObject* ball3dobject;

    float groundX, groundY;
    float rampX, rampY;
    float rampAngle;
    float gameiconangle;
    float ballX, ballY;
    float gameIconX, gameIconY;
    float impulse = 1;

    gipBulletPhysics gBulletObj;

	btCollisionShape* groundCollisionShape;
	btCollisionShape* rampCollisionShape;
	btCollisionShape* ballCollisionShape;
	btCollisionShape* softBallCollisionShape;
	btCollisionObject* groundCollisionObj;
	btCollisionObject* rampCollisionObj;
	btCollisionObject* ballCollisionObj;
	btCollisionObject* softBallCollisionObj;
	btCollisionObject* gameIconCollisionObj;
	btRigidBody* groundRigidBody;
	btRigidBody* rampRigidBody;
	btRigidBody* ballRigidBody;
	btRigidBody* softBallRigidBody;
	btRigidBody* gameIconRigidBody;
	btTransform groundTransform;
	btTransform rampTransform;
	btTransform ballTransform;
	btTransform softBallTransform;
	btTransform gameIconTransform;
	btDefaultMotionState* myMotionState;

	int worldType;

	//void createBoxObject();
	//void createBallObject();
	void startSimulation();
	void startCleanup();
};

#endif /* GAMECANVAS_H_ */
