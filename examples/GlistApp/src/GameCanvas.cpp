/*
 * GameCanvas.cpp
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ISCI
 */


#include "GameCanvas.h"



GameCanvas::GameCanvas(gApp* root) : gBaseCanvas(root) {
	this->root = root;
}

GameCanvas::~GameCanvas() {
}

void GameCanvas::setup() {
	// create and start physic world
	gipBulletPhysics::Instance()->startWorld(true);

	sky.loadImage("layer-1-sky.png");
	mountain.loadImage("layer-2-mountain.png");

	//Load images which we will use for physic objects
	groundimage.loadImage("layer-3-ground.png");
	rampimage.loadImage("ramp.png");
	gameIconimage.loadImage("gameicon/icon.png");
	mountain.loadImage("layer-2-mountain.png");
	ballimage.loadImage("ball.png");


	//load physic objects
	groundobject = new gipPhysic2dBox(&groundimage, true, 0.0f,gipBulletPhysics::LAYER1, gipBulletPhysics::LAYER0|gipBulletPhysics::LAYER1|gipBulletPhysics::COLLISIONLAYERS::LAYER2|gipBulletPhysics::LAYER3);
	groundobject->setPosition(0.0f, getHeight() - groundimage.getHeight());
	//

	wallleft = new gipPhysic2dBox(getWidth() * 0.05f, getHeight());
	wallright = new gipPhysic2dBox(getWidth() * 0.05f, getHeight());
	wallright->setPosition(getWidth() * 0.95f, 0.0f, 0.0f);
	wallup = new gipPhysic2dBox(getWidth(), getHeight() * 0.05f);

	rampobject = new gipPhysic2dBox(&rampimage, true, 0.0f);
	rampobject->setRotation(135.0f);
	rampobject->setPosition(getWidth() * 0.7f, getHeight() * 0.6f);


	gameiconobject = new gipPhysic2dBox(&gameIconimage);
	gameiconobject->setPosition(getWidth() * 0.1f, getHeight() * 0.4f);

	gameiconobject->setOnCollided(std::bind(onCollided, this, std::placeholders::_1));
	gameiconobject->setSize(0.8f, 0.4f);

	//ballobject = new gPhysic2dCircle(&ballimage, false, 1.0f, 1.0f, gPhysic::LAYER3, gPhysic::LAYER0|gPhysic::LAYER1|gPhysic::LAYER2|gPhysic::LAYER3);
	//ballobject->setPosition( glm::vec3(getWidth() - ballimage.getWidth(),  0.0f, 0.0f));
	gipPhysicObject* newball = new gipPhysic2dCircle(&ballimage, false, 0.4f);
	newball->setBounce(10.0f);
	newball->setPosition(getWidth() - ballimage.getWidth() * 4, 0.0f);

	balls.push_back(newball);
}

void GameCanvas::update() {

	// Physics calculations doing here.
	gipBulletPhysics::Instance()->runPhysicWorldStep();
	// force and impulse methods can call here.
	//ballobject->applyCentralForce(glm::vec3(-1.0f, 1.0f, 0.0f));

}

void GameCanvas::draw() {
	//Object without physic
	sky.draw(getWidth() / 2 - sky.getWidth() / 2, getHeight() - sky.getHeight());
	mountain.draw(getWidth() / 2 - mountain.getWidth() / 2, getHeight() - mountain.getHeight() / 2);

	//Object with physic
    groundobject->draw();
    rampobject->draw();
    gameiconobject->draw();
    int ballcount = balls.size();
    for(int i = 0; i < ballcount; i++) {
    	balls[i]->draw();
    }

	renderer->setColor(255, 0, 0);
	//Physic debug renderer
	gipBulletPhysics::Instance()->drawDebug();
}

void GameCanvas:: onCollided(int targetid) {
	//gLogi("Game Canvas") << "Game icon �arp��ma alg�land�" << targetid;
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
	gipPhysicObject* newball = new gipPhysic2dCircle(&ballimage, false, 4.0f);
	//->setBounce((std::rand() % 100) / 100);
	newball->setPosition(x,  y, 0.0f);
	newball->setBounce(10.0f);
	balls.push_back(newball);
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

