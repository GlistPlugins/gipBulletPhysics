/*
 * GameCanvas.cpp
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ÝÞÇÝ
 */


#include "GameCanvas.h"


GameCanvas::GameCanvas(gApp* root) : gBaseCanvas(root) {
	this->root = root;
}

GameCanvas::~GameCanvas() {
}

void GameCanvas::setup() {
	// create and start physic world
	gPhysic::Instance()->startWorld();

	sky.loadImage("layer-1-sky.png");
	mountain.loadImage("layer-2-mountain.png");

	//Load images which we will use for physic objects
	groundimage.loadImage("layer-3-ground.png");
	rampimage.loadImage("ramp.png");
	gameIconimage.loadImage("gameicon/icon.png");
	mountain.loadImage("layer-2-mountain.png");
	ballimage.loadImage("ball.png");

	//load physic objects
	groundobject = new gPhysic2dBox(&groundimage, true, 0.0f, glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(getWidth() * 0.0f, getHeight() * 0.9f, 0.0f));
	groundobjectup = new gPhysic2dBox(true, 0.0f, getWidth(), getHeight() * 0.1f, 1, glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(0.0f, 0.0f,0.0f), glm::vec3(0.0f, 0.0f,0.0f));
	groundobject->setPosition(glm::vec3(getWidth() * 0.5f - groundobject->getWidth() * 0.5f, getHeight() - groundobject->getHeight(), 0.0f));
	rampobject = new gPhysic2dBox(&rampimage, true, 0.0f, glm::vec3(1.0f, 1.0f, 1.0f),glm::vec3(0.0f, 0.0f,135.0f));
	rampobject->setPosition(glm::vec3(getWidth() * 0.7f, getHeight() * 0.6f, 0.0f));
	gameiconobject = new gPhysic2dBox(&gameIconimage, false, 5.0f);
	gameiconobject->setPosition(glm::vec3(0.0f, getHeight() * 0.4f, 0.0f));
	btQuaternion newrot;
	newrot.setRotation(btVector3(0.0f, 0.0f, 1.0f), gDegToRad(90.0f));
	gameiconobject->setRotation(newrot);
	gameiconobject->setOnCollided(std::bind(onCollided, this, std::placeholders::_1));
	ballobject = new gPhysic2dCircle(&ballimage, false, 1.0f, 1.0f, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(getWidth() - ballimage.getWidth(), ballimage.getHeight(), 0.0f));

}

void GameCanvas::update() {
	// Physics calculations doing here.
	gPhysic::Instance()->runPhysicWorldStep();
	// force and impulse methods can call here.
	ballobject->applyCentralForce(glm::vec3(-1.0f, 1.0f, 0.0f));

}

void GameCanvas::draw() {
	//Object without physic
	sky.draw(getWidth() / 2 - sky.getWidth() / 2, getHeight() - sky.getHeight());
	mountain.draw(getWidth() / 2 - mountain.getWidth() / 2, getHeight() - mountain.getHeight() / 2);

	//Object with physic
    groundobject->draw();
    groundobjectup->draw();
    rampobject->draw();
    ballobject->draw();
    gameiconobject->draw();

	renderer->setColor(255, 0, 0);
	//Physic debug renderer
	gPhysic::Instance()->drawDebug();
}

void GameCanvas:: onCollided(int targetid) {
	gLogi("Game Canvas") << "Game icon çarpýþma algýlandý" << targetid;
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

