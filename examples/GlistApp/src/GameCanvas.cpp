/*
 * GameCanvas.cpp
 *
 *  Created on: 12 Sep 2022
 *      Author: Faruk Aygun
 */


#include "GameCanvas.h"


GameCanvas::GameCanvas(gApp* root) : gBaseCanvas(root) {
	this->root = root;
}

GameCanvas::~GameCanvas() {
	gBulletObj.clean();
}

void GameCanvas::setup() {
	sky.loadImage("layer-1-sky.png");
	mountain.loadImage("layer-2-mountain.png");
	ground.loadImage("layer-3-ground.png");
	ball.loadImage("ball.png");

	groundX = getWidth() / 2 - ground.getWidth() / 2;
	groundY = getHeight() - ground.getHeight();
	ballX   = getWidth() - ball.getWidth();
	ballY   = getHeight() / 2 - ball.getHeight() / 2;

	worldType = SOFTRIGIDWORLD; // 1
	// create world
	gBulletObj.initializeWorld(worldType);
	if (worldType == SOFTRIGIDWORLD) {
		// set default configurations
		gBulletObj.setErp2();
		gBulletObj.setglobalCfm();
		gBulletObj.setNumIterations();
		gBulletObj.setSolverMode();
		gBulletObj.setSplitImpulse();
	}

	gBulletObj.setGravity(glm::vec3(0.0f, -600.8f, 0.0f));

	// create object with image
	groundobject   = new gImageGameObject(ground, 0.0f, glm::vec2(groundX, groundY), 0.0f);
	softballobject = new gImageGameObject(ball, 5.0f, glm::vec2(ballX, ballY), 0.0f);

	// decrease stiffness and increase damping for softer ground. ex: 2000, 2
	gBulletObj.createSoftContactBox2dObject(groundobject, 5000, 0.1f);
	gBulletObj.createSoftCircle2dObject(softballobject);

	// friction methods can call here.
	/*
	gBulletObj.setRollingFriction(softballobject, 100.0f);
	gBulletObj.setRollingFriction(groundobject, 100.0f);
	gBulletObj.setFriction(softballobject, 100.0f);
	gBulletObj.setFriction(groundobject, 100.0f);
	*/
}

void GameCanvas::update() {
	// Physics calculations doing here.
	gBulletObj.stepSimulation(60);
	// force and impulse methods can call here.
	gBulletObj.applyCentralForce(softballobject, glm::vec3(-15.0f, 0.0f, 0.0f));
	/*
	 * Testing friction
	 * Uncomment line 54 to 59 and 73. Comment line 64.
	 * It is expected that the object will not move due to friction.
	 * The ball does not move because the friction force is greater than the impulse force.
	 */
	// if (impulse) gBulletObj.applyCentralImpulse(softballobject, glm::vec3(-15.0f, 0.0f, 0.0f)); impulse = 0;
}

void GameCanvas::draw() {
	sky.draw(getWidth() / 2 - sky.getWidth() / 2, getHeight() - sky.getHeight());
	mountain.draw(getWidth() / 2 - mountain.getWidth() / 2, getHeight() - mountain.getHeight() / 2);
	ground.draw(groundX, groundY);
	// getting position and rotation values from created image object.
	ball.draw(softballobject->getPosition().x,
			  softballobject->getPosition().y,
			  softballobject->getWidth(),
			  softballobject->getHeight(),
			  softballobject->getRotationAngle());
}

void GameCanvas::keyPressed(int key) {
//	gLogi("GameCanvas") << "keyPressed:" << key;
}

void GameCanvas::keyReleased(int key) {
//	gLogi("GameCanvas") << "keyReleased:" << key;
}

void GameCanvas::charPressed(unsigned int codepoint) {
//	gLogi("GameCanvas") << "charPressed:" << gCodepointToStr(codepoint);
}

void GameCanvas::mouseMoved(int x, int y) {
//	gLogi("GameCanvas") << "mouseMoved" << ", x:" << x << ", y:" << y;
}

void GameCanvas::mouseDragged(int x, int y, int button) {
//	gLogi("GameCanvas") << "mouseDragged" << ", x:" << x << ", y:" << y << ", b:" << button;
}

void GameCanvas::mousePressed(int x, int y, int button) {
//	gLogi("GameCanvas") << "mousePressed" << ", button:" << button;
}

void GameCanvas::mouseReleased(int x, int y, int button) {
//	gLogi("GameCanvas") << "mouseReleased" << ", button:" << button;
}

void GameCanvas::mouseScrolled(int x, int y) {
//	gLogi("GameCanvas") << "mouseScrolled" << ", x:" << x << ", y:" << y;
}

void GameCanvas::mouseEntered() {
}

void GameCanvas::mouseExited() {
}

void GameCanvas::windowResized(int w, int h) {
}

void GameCanvas::showNotify() {
}

void GameCanvas::hideNotify() {
}

void GameCanvas::startCleanup() {

}
