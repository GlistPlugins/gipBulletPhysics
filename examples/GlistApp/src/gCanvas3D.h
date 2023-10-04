/*
 * gCanvas3D.h
 *
 *  Created on: 4 Eki 2023
 *      Author: alper
 */

#ifndef SRC_GCANVAS3D_H_
#define SRC_GCANVAS3D_H_

#include <gGhostModelGameObject.h>
#include "gBaseCanvas.h"
#include "gApp.h"
#include "gipBulletPhysics.h"
#include "gImageGameObject.h"
#include "gModelGameObject.h"
#include "gCamera.h"
#include "gLight.h"
#include "gSkybox.h"
#include "gBox.h"

class gCanvas3D : public gBaseCanvas {
public:
	gCanvas3D(gApp* root);
	virtual ~gCanvas3D();

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

	void onCollidedBall(int targetid);

	gApp* root;
	gImage logo;

	gCamera camera;
	gLight sun1, sun2;
	gSkybox sky;
	gBox* ground;
	gTexture texturelight, texturered, texturegreen, texturedark, textureorange;
	std::vector<gBox> boxes;
	gBox* box1;
	gBox* box2;

	//Physic referances and variables--------------------------
	gipBulletPhysics* physicworld;
	gModelGameObject* box1gameobject;
	gModelGameObject* box2gameobject;
	gModelGameObject* groundobject;

	gGhostModelGameObject* ghostbox;

	glm::vec3 spawnpoint;
	glm::vec3 cameraposition;

	int keystate = 0;
	int mouseoldx = 0;
	int mouseoldy = 0;
};

#endif /* SRC_GCANVAS3D_H_ */
