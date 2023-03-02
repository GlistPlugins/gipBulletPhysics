/*
 * GameCanvas3D.h
 *
 *  Created on: 26 �ub 2023
 *      Author: remzi
 */

#ifndef SRC_GAMECANVAS3D_H_
#define SRC_GAMECANVAS3D_H_

#include <gipPhysic3dBox.h>
#include "gBaseCanvas.h"
#include "gApp.h"
#include "gImage.h"
#include "gBox.h"
#include "gLight.h"
#include "gCamera.h"
#include "gModel.h"
#include "gTexture.h"
#include <vector>
#include "bases/gipBulletPhysics.h"
#include "bases/gipPhysicObject.h"


class GameCanvas3D : public gBaseCanvas {
public:
	GameCanvas3D(gApp* root);
	virtual ~GameCanvas3D();

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

	void drawScene();

private:

	const int KEY_W = 2;
	const int KEY_S = 4;
	const int KEY_D = 8;
	const int KEY_A = 16;
	const int KEY_R = 32;
	const int KEY_F = 64;

	gApp* root;
	gImage logo;

	gCamera camera;
	gLight sun1, sun2;
	gSkybox sky;
	gBox ground;
	gTexture texturelight, texturered, texturegreen, texturedark, textureorange;
	std::vector<gBox> boxes;
	gBox box1;
	gBox box2;

	gipPhysicObject* groundobject;
	gipPhysicObject* boxobject;

	glm::vec3 spawnpoint;
	glm::vec3 cameraposition;

	int keystate = 0;
	int mouseoldx = 0;
	int mouseoldy = 0;
};


#endif /* SRC_GAMECANVAS3D_H_ */
