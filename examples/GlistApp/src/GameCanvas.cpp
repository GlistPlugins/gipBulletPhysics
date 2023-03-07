/*
 * GameCanvas.cpp
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ISCÝ
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
	ramp.loadImage("ramp.png");
	ball.loadImage("ball.png");
	gameIcon.loadImage("gameicon/icon.png");

	groundX = getWidth() / 2 - ground.getWidth() / 2;
	groundY = getHeight() - ground.getHeight();
	rampX = getWidth() * 0.7f;
	rampY = getHeight() * 0.6f;
	rampAngle = 135.0f;
	gameIconX = 0.0f;
	gameIconY = getHeight() * 0.6f;
	gameiconangle = 35.0f;
	ballX   = getWidth() - ball.getWidth();
	ballY   = 0.0f; //getHeight() / 2 - ball.getHeight() / 2;

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
	//groundobject->setOnCollided(std::bind(&GameCanvas::onCollided,this, std::placeholders::_1));

	rampobject   = new gImageGameObject(ramp, 0.0f, glm::vec2(rampX, rampY), rampAngle);
	rampobject->setOnCollided(std::bind(&GameCanvas::onCollided,this, std::placeholders::_1));

	gameiconobject   = new gImageGameObject(gameIcon, 0.0f, glm::vec2(gameIconX, gameIconY), 0.0f);
	gameiconobject->setOnCollided(std::bind(&GameCanvas::onCollided,this, std::placeholders::_1));

	softballobject = new gImageGameObject(ball, 5.0f, glm::vec2(ballX, ballY), 0.0f);
	softballobject->setOnCollided(std::bind(&GameCanvas::onCollided,this, std::placeholders::_1));

	// decrease stiffness and increase damping for softer ground. ex: 2000, 2
	gBulletObj.createSoftContactBox2dObject(groundobject, 5000, 0.1f, 0.0f);
	gBulletObj.createSoftContactBox2dObject(rampobject, 5000, 0.1f, rampAngle);
	gBulletObj.createSoftContactBox2dObject(gameiconobject,5000, 0.1f, gameiconangle, glm::vec2(1.0f, 2.0f));
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
	//gBulletObj.applyCentralForce(softballobject, glm::vec3(-500.0f, 0.0f, 0.0f));
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
	gameIcon.draw(gameIconX, gameIconY, gameIcon.getWidth(), gameIcon.getHeight(), gameIcon.getWidth() * 0.5f, gameIcon.getHeight() * 0.5f, gameiconangle);

	ramp.draw(rampX, rampY, ramp.getWidth(), ramp.getHeight(), ramp.getWidth() * 0.5f, ramp.getHeight() * 0.5f, rampAngle);
	// getting position and rotation values from created image object.
	ball.draw(softballobject->getPosition().x,
			  softballobject->getPosition().y,
			  softballobject->getWidth(),
			  softballobject->getHeight(),
			  softballobject->getRotationAngle());

	renderer->setColor(255, 0, 0);
	gBulletObj.drawDebug();
}

void GameCanvas:: onCollided(int targetid) {
	gLogi("Game Canvas") << "Game canvasta çarpýþma algýlandý";
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
