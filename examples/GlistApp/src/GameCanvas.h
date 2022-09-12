 /*
 * GameCanvas.h
 *
 *  Created on: 12 Sep 2022
 *      Author: Faruk Aygun
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

private:
	int static const RIGIDWORLD = 0;
	int static const SOFTRIGIDWORLD = 1;

	gApp* root;
    gImage sky;
    gImage mountain;
    gImage ground;
    gImage ball;

    gModel ball3d;

    gImageGameObject* groundobject;
    gImageGameObject* ballobject;
    gImageGameObject* softballobject;

    gModelGameObject* ball3dobject;

    float groundX, groundY;
    float ballX, ballY;
    float impulse = 1;

    gipBulletPhysics gBulletObj;

	btCollisionShape* groundShape;
	btCollisionShape* ballShape;
	btCollisionShape* softBallShape;
	btCollisionObject* groundObj;
	btCollisionObject* ballObj;
	btCollisionObject* softBallObj;
	btRigidBody* groundRigidBody;
	btRigidBody* ballRigidBody;
	btRigidBody* softBallRigidBody;
	btTransform groundTransform;
	btTransform ballTransform;
	btTransform softBallTransform;
	btDefaultMotionState* myMotionState;

	int worldType;

	//void createBoxObject();
	//void createBallObject();
	void startSimulation();
	void startCleanup();
};

#endif /* GAMECANVAS_H_ */
