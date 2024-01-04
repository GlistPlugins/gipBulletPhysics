/*
 * gCanvas.cpp
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ISCI
 */


#include "gCanvas.h"


gCanvas::gCanvas(gApp* root) : gBaseCanvas(root) {
	this->root = root;
}

gCanvas::~gCanvas() {
	gBulletObj->clean();
}

void gCanvas::setup() {
	gBulletObj = new gipBulletPhysics(gipBulletPhysics::WORLDCOORDINATETYPE::WORLD2D);
	sky.loadImage("layer-1-sky.png");
	mountain.loadImage("layer-2-mountain.png");
	ground.loadImage("layer-3-ground.png");
	ramp.loadImage("ramp.png");
	gameIcon.loadImage("gameicon/icon.png");

	// create object with image referance
	groundobject   = new gImageGameObject(gBulletObj);
	groundobject->setTag(1); //1 for grouping grounds
	groundobject->setName("Ground Object");
	groundobject->setImage(&ground);
	groundobject->setPosition(0.0f, getHeight() - ground.getHeight());
	groundobject->setBounce(0.1f);

	// create object without image referance
	rampobject   = new gImageGameObject(gBulletObj);
	rampobject->setTag(1); //1 for grouping grounds, you can set any integer as you wish
	rampobject->setName("Ramp Object");
	rampobject->setColliderSize(200.0f, 40.0f);
	rampobject->setPosition(getWidth() * 0.8f, getHeight() * 0.7f);
	rampobject->setRotation2D(25.0f);
	rampobject->setBounce(0.1f);
	//rampobject->setFriction(100.f);
	//rampobject->setRollingFriction(100.f);
	//rampobject->setSpinningFriction(100.f);

	// create object with image referance
	gameiconobject   = new gImageGameObject(gBulletObj);
	gameiconobject->setTag(2); //2 for grouping non ground statics, you can set any integer as you wish
	gameiconobject->setImage(&gameIcon);
	gameiconobject->setName("Gameicon Object");
	gameiconobject->setPosition(0.0f, getHeight() * 0.4f);
	gameiconobject->setObjectLayers(gipBaseGameObject::COLLISIONLAYERS::LAYER4);

	// create object with loading image
	softballobject = new gImageGameObject(gBulletObj);
	softballobject->setTag(3); //3 for balls, you can set any integer as you wish
	softballobject->setName("Softball Object");
	softballobject->loadImage("ball.png");
	softballobject->setShapeType(gipBaseGameObject::SHAPETYPE::SHAPETYPE_SPHERE);
	softballobject->setPosition(getWidth() - softballobject->getWidth(), getHeight() * 0.1f);
	softballobject->setMass(40.0f);
	softballobject->setBounce(0.4f);
	//softballobject->setOnCollided(std::bind(&gCanvas::onCollidedBall,this, std::placeholders::_1));

	//create object with loading image
	seconballobject = new gImageGameObject(gBulletObj);
	seconballobject->loadImage("ball.png");
	//you can set image render size
	//ghostballobject->setObjectSize(400.0f, 400.0f);
	seconballobject->setTag(3); //3 for balls, you can set any integer as you wish
	seconballobject->setName("Softball Object");
	//you can lock or unlock size of collider and image
	//ghostballobject->setIsSizeLocked(false);
	seconballobject->setObjectSize(80.0f, 80.0f);
	seconballobject->setColliderSize(120.0f, 120.0f);
	seconballobject->setShapeType(gipBaseGameObject::SHAPETYPE::SHAPETYPE_SPHERE);
	seconballobject->setPosition(0 + seconballobject->getWidth() * 1.2f, getHeight() * 0.1f);
	seconballobject->setMass(400.0f);
	seconballobject->setBounce(10.0f);
	seconballobject->setMaskLayers(gipBaseGameObject::COLLISIONLAYERS::LAYER1 | gipBaseGameObject::COLLISIONLAYERS::LAYER2);
	//ghostballobject->setColliderOffset(100.f, 40.0f);
	//softballobject->setOnCollided(std::bind(&gCanvas::onCollidedBall,this, std::placeholders::_1));


}

void gCanvas::update() {

	// Physics calculations doing here.
	gBulletObj->runPhysicWorldStep();
	// force and impulse methods can call here.
	//ghostballobject->applyTorque(glm::vec3(0.0f, 0.0f, -40.0f));
	//ghostballobject->applyCentralForce(glm::vec3(10.0f, 0.0f, 0.0f));
	//ghostballobject->applyImpulse(glm::vec3(0.0f, 0.0f, -100.0f), glm::vec3(0.0f, 0.0f, 0.0f)); //will add velocity instant
}

void gCanvas::draw() {
	//Draws nimages without colliders
	sky.draw(getWidth() / 2 - sky.getWidth() / 2, getHeight() - sky.getHeight());
	mountain.draw(getWidth() / 2 - mountain.getWidth() / 2, getHeight() - mountain.getHeight() / 2);

	//Draw physic objects
	seconballobject->draw();
	gameiconobject->draw();
	rampobject->draw();
	softballobject->draw();
	groundobject->draw();

	//This line for draw collider, delete or comment this line when get release
	gBulletObj->drawDebug();
}

void gCanvas:: onCollidedBall(gipBaseGameObject* target, glm::vec3 pointa, glm::vec3 pointb) {
	gLogi("Ball Collison") << target->getName() << pointb.x;
}

void gCanvas::keyPressed(int key) {
//	gLogi("gCanvas") << "keyPressed:" << key;

	if(key == G_KEY_R) {
		ghostbox = new gGhostImageGameObject(gBulletObj);
		ghostbox->setPosition(40.0f, getHeight() * 0.4f);
		ghostbox->setColliderSize(120.0f, 0.0f);
		ghostbox->setOnCollided(std::bind(&gCanvas::onCollidedBall,this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		gBulletObj->runPhysicWorldStep();
		delete ghostbox;
	}

}

void gCanvas::keyReleased(int key) {
//	gLogi("gCanvas") << "keyReleased:" << key;
}

void gCanvas::charPressed(unsigned int codepoint) {
//	gLogi("gCanvas") << "charPressed:" << gCodepointToStr(codepoint);
}

void gCanvas::mouseMoved(int x, int y) {
//	gLogi("gCanvas") << "mouseMoved" << ", x:" << x << ", y:" << y;
}

void gCanvas::mouseDragged(int x, int y, int button) {
//	gLogi("gCanvas") << "mouseDragged" << ", x:" << x << ", y:" << y << ", b:" << button;
}

void gCanvas::mousePressed(int x, int y, int button) {
//	gLogi("gCanvas") << "mousePressed" << ", button:" << button;
}

void gCanvas::mouseReleased(int x, int y, int button) {
//	gLogi("gCanvas") << "mouseReleased" << ", button:" << button;
}

void gCanvas::mouseScrolled(int x, int y) {
//	gLogi("gCanvas") << "mouseScrolled" << ", x:" << x << ", y:" << y;
}

void gCanvas::mouseEntered() {
}

void gCanvas::mouseExited() {
}

void gCanvas::windowResized(int w, int h) {
}

void gCanvas::showNotify() {
}

void gCanvas::hideNotify() {
}

void gCanvas::startCleanup() {

}
