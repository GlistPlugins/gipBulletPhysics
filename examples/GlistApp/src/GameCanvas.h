/*
 * GameCanvas.cpp
 *
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ÝÞÇÝ
 */


#ifndef GAMECANVAS_H_
#define GAMECANVAS_H_

#include <gPhysic2dBox.h>
#include <gPhysic2dCircle.h>
#include "gBaseCanvas.h"
#include "gApp.h"
#include "bases/gPhysic.h"

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

	gApp* root;
    gImage sky;
    gImage mountain;
    gImage groundimage;
    gImage rampimage;
    gImage ballimage;
    gImage gameIconimage;

    gPhysicObject* groundobject;
    gPhysicObject* groundobjectup;
    gPhysicObject* rampobject;
    gPhysicObject* ballobject;
    gPhysicObject* gameiconobject;



    float groundX, groundY;
    float rampX, rampY;
    float rampAngle;
    float gameiconangle;
    float ballX, ballY;
    float gameIconX, gameIconY;
    float impulse = 1;

};

#endif /* GAMECANVAS_H_ */
