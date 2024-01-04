/*
 * gCanvas.cpp
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ISCI
 */


#ifndef GCANVAS_H_
#define GCANVAS_H_

#include <gGhostImageGameObject.h>
#include "gBaseCanvas.h"
#include "gApp.h"
#include "gipBulletPhysics.h"
#include "gImageGameObject.h"
#include "gModelGameObject.h"

class gCanvas : public gBaseCanvas {
public:
	gCanvas(gApp* root);
	virtual ~gCanvas();

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

	void onCollidedBall(gipBaseGameObject* target, glm::vec3 pointa, glm::vec3 pointb);

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
    gImageGameObject* seconballobject;

	gGhostImageGameObject* ghostbox;

    gipBulletPhysics* gBulletObj;


	void startSimulation();
	void startCleanup();
};

#endif /* GCANVAS_H_ */
