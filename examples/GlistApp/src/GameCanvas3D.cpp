/*
 * GameCanvas3D.cpp
 *
 *  Created on: 26 Þub 2023
 *      Author: remzi
 */

#include "GameCanvas3D.h"


GameCanvas3D::GameCanvas3D(gApp* root) : gBaseCanvas(root) {
	this->root = root;
}

GameCanvas3D::~GameCanvas3D() {
}

void GameCanvas3D::setup() {
	gipBulletPhysics::Instance()->startWorld(false);

	gLogi("GameCanvas3D") << "setup";
	logo.loadImage("glistengine_logo.png");

	sun1.setPosition(0.0f, 40.0f, -100.0f);
	sun1.setSpecularColor(8, 8, 8);
	sun1.setAmbientColor(180, 180, 172);
	sun2.setPosition(0.0f, 40.0f, -100.0f);
	sun2.setSpecularColor(8, 8, 8);
	sun2.setAmbientColor(180, 180, 172);


	std::vector<std::string> skytextures;
	skytextures.push_back("skymap/right.jpg");
	skytextures.push_back("skymap/left.jpg");
	skytextures.push_back("skymap/top.jpg");
	skytextures.push_back("skymap/bottom.jpg");
	skytextures.push_back("skymap/front.jpg");
	skytextures.push_back("skymap/back.jpg");
	sky.loadTextures(skytextures);




	cameraposition = glm::vec3(0.0f, 60.0f, 100.0f);
	camera.setPosition(cameraposition);
	camera.tilt(-PI * 0.1f);

	texturelight.loadTexture("ground/Light/texture_06.png");
	texturedark.loadTexture("ground/Dark/texture_06.png");
	texturered.loadTexture("ground/Red/texture_06.png");
	texturegreen.loadTexture("ground/Green/texture_06.png");
	textureorange.loadTexture("ground/Orange/texture_06.png");

	ground.scale(100.0f, 1.0f, 100.0f);
	ground.setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
	ground.setTexture(&texturelight);


	box1.setPosition(0.0f, 40.0f, 0.0f);
	box1.setScale(10.0f);
	box1.getMaterial()->setAmbientColor(0, 0, 255);

	box2.setPosition(0.0f, 0.0f, 0.0f);
	box2.setScale(10.0f);
	box2.getMaterial()->setAmbientColor(0, 255, 0);
	boxes.push_back(box1);
	groundobject = new gipPhysic3dBox(&ground);
	boxobject = new gipPhysic3dBox(&box1, false, 1.0f);
	//boxobject->setRotation(0.0f, 0.0f, 40.0f);

	//groundobject->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
	//groundobject->setSize(glm::vec3(0.1f, 0.1f, 0.1f));

}

void GameCanvas3D::update() {
	//gLogi("GameCanvas3D") << "update";
	gipBulletPhysics::Instance()->runPhysicWorldStep();

	if(keystate & KEY_W) {
		camera.dolly(-10.0f);
	} else if(keystate & KEY_S) {
		camera.dolly(10.0f);
	}

	if(keystate & KEY_D) {
		camera.truck(10.0f);
	} else if(keystate & KEY_A) {
		camera.truck(-10.0f);
	}

	sun2.pan((std::rand() % 100) * 0.001f);
	sun2.tilt((std::rand() % 100) * 0.001f);
	sun2.roll((std::rand() % 100) * 0.001f);
}

void GameCanvas3D::draw() {
	//gLogi("GameCanvas3D") << "draw";
	//logo.draw((getWidth() - logo.getWidth()) / 2, (getHeight() - logo.getHeight()) / 2);

	drawScene();




}

void GameCanvas3D::drawScene() {
	camera.begin();
	enableDepthTest();
	sun1.enable();
	//sun2.enable();
	//sky.draw();
	box2.draw();
	groundobject->draw();
	boxobject->draw();
	auto boxcount = boxes.size();
	renderer->setColor(0, 0, 255);
	for(int i = 0; i < boxcount; i++) {
		//boxes[i].draw();
	}
	disableDepthTest();
	renderer->setColor(255, 0, 0);
	gipBulletPhysics::Instance()->drawDebug();
	sun1.disable();
	//sun2.disable();
	camera.end();


}

void GameCanvas3D::keyPressed(int key) {
//	gLogi("GameCanvas") << "keyPressed:" << key;
	if(key == G_KEY_W){
		keystate |= KEY_W;
	} else 	if(key == G_KEY_S){
		keystate |= KEY_S;
	}

	if(key == G_KEY_A){
		keystate |= KEY_A;
	} else 	if(key == G_KEY_D){
		keystate |= KEY_D;
	}

	if(key == G_KEY_R){
		keystate |= KEY_R;
	} else 	if(key == G_KEY_F){
		keystate |= KEY_F;
	}

	if(key == G_KEY_ESC){
		exit(0);
	}
}

void GameCanvas3D::keyReleased(int key) {
//	gLogi("GameCanvas") << "keyReleased:" << key;
	if(key == G_KEY_W){
		keystate &= !KEY_W;
	}
	if(key == G_KEY_S){
		keystate &= !KEY_S;
	}

	if(key == G_KEY_A){
		keystate &= !KEY_A;
	}
	if(key == G_KEY_D){
		keystate &= !KEY_D;
	}

	if(key == G_KEY_R){
		keystate &= !KEY_R;
	}
	if(key == G_KEY_F){
		key &= !KEY_F;
	}
}

void GameCanvas3D::charPressed(unsigned int codepoint) {
//	gLogi("GameCanvas") << "charPressed:" << gCodepointToStr(codepoint);
}

void GameCanvas3D::mouseMoved(int x, int y) {
//	gLogi("GameCanvas") << "mouseMoved" << ", x:" << x << ", y:" << y;
}

void GameCanvas3D::mouseDragged(int x, int y, int button) {
//	gLogi("GameCanvas") << "mouseDragged" << ", x:" << x << ", y:" << y << ", b:" << button;
	if(mouseoldx > x){
		camera.pan(0.04f);
	} else 	if(mouseoldx < x){
		camera.pan(-0.04f);
	}
	if(mouseoldy > y){
		camera.tilt(0.04f);
	} else 	if(mouseoldy < y){
		camera.tilt(-0.04f);
	}
	mouseoldx = x;
	mouseoldy = y;
}

void GameCanvas3D::mousePressed(int x, int y, int button) {
	mouseoldx = x;
	mouseoldy = y;
}

void GameCanvas3D::mouseReleased(int x, int y, int button) {
//	gLogi("GameCanvas") << "mouseReleased" << ", button:" << button;

}

void GameCanvas3D::mouseScrolled(int x, int y) {
//	gLogi("GameCanvas") << "mouseScrolled" << ", x:" << x << ", y:" << y;
}

void GameCanvas3D::mouseEntered() {
}

void GameCanvas3D::mouseExited() {
}

void GameCanvas3D::windowResized(int w, int h) {
}

void GameCanvas3D::showNotify() {

}

void GameCanvas3D::hideNotify() {

}


