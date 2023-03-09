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

	void onCollidedBall(int targetid);

private:


	gApp* root;

	//images
    gImage sky;
    gImage mountain;
    gImage ground;
    gImage ramp;
    gImage gameIcon;


    gImageGameObject* groundobject;
    gImageGameObject* rampobject;
    gImageGameObject* softballobject;
    gImageGameObject* gameiconobject;
    gImageGameObject* ghostballobject;


    gipBulletPhysics* gBulletObj;


	void startSimulation();
	void startCleanup();
};

#endif /* GAMECANVAS_H_ */
