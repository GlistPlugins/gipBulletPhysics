/*
 * gCanvas3D.cpp
 *
 *  Created on: 4 Eki 2023
 *      Author: alper
 */

#include <gCanvas3D.h>

gCanvas3D::gCanvas3D(gApp* root) : gBaseCanvas(root) {
	this->root = root;
}

gCanvas3D::~gCanvas3D() {
}


void gCanvas3D::setup() {
	//gipBulletPhysics::getInstance()->startWorld(false);

	gLogi("gCanvas3D") << "setup";
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




	cameraposition = glm::vec3(0.0f, 20.0f, 20.0f);
	camera.setPosition(cameraposition);
	camera.tilt(-PI * 0.2f);

	texturelight.loadTexture("ground/Light/texture_06.png");
	texturedark.loadTexture("ground/Dark/texture_06.png");
	texturered.loadTexture("ground/Red/texture_06.png");
	texturegreen.loadTexture("ground/Green/texture_06.png");
	textureorange.loadTexture("ground/Orange/texture_06.png");


	//Physic referances and variables--------------------------
	physicworld = new gipBulletPhysics(gipBulletPhysics::WORLDCOORDINATETYPE::WORLD3D);


	groundobject = new gModelGameObject(physicworld);
	groundobject->setName("groundobject");
	ground = new gBox();
	ground->setTexture(&texturelight);
	groundobject->setMesh(ground);
	groundobject->setObjectSize(100.0f, 1.0f, 100.0f);
	groundobject->setBounce(0.1f);
	groundobject->setObjectLayers(gipBulletPhysics::COLLISIONLAYERS::LAYER1);
	groundobject->setPosition(0.0f, 0.0f, 0.0f);



	box1gameobject = new gModelGameObject(physicworld);
	box1gameobject->setName("box1gameobject");
	box1gameobject->setPosition(0.0f, 10.0f, 0.0f);
	box1 = new gBox();
	box1->getMaterial()->setAmbientColor(0, 0, 255);
	box1gameobject->setMesh(box1);
	box1gameobject->setMass(1.0f);
	box1gameobject->setBounce(8.0f);
	box1gameobject->setRotation(20.0f, 20.0f, 20.0f);
	//box1gameobject->setColliderSize(14.0f,14.0f, 14.0f);
 	box1gameobject->setObjectSize(10.0f, 2.0f, 10.0f);



	box2gameobject = new gModelGameObject(physicworld);
	box2gameobject->setName("box2gameobject");
	box2 = new gBox();
	box2gameobject->setMesh(box2);
	box2gameobject->setPosition(-20.0f, 4.0f, 0.0f);
	//box2gameobject->setObjectSize(2.0f, 2.0f, 2.0f);
	box2gameobject->setObjectSize(2.0f, 2.0f, 2.0f);
	box2gameobject->setColliderSize(2.0f, 2.0f, 2.0f);
	box2->getMaterial()->setAmbientColor(0, 255, 0);


	ghostbox = new gGhostModelGameObject(physicworld);
	ghostbox->setPosition(0.0f, 20.0f, 0.0f);
	ghostbox->setColliderSize(0.0f, 100.0f, 0.0f);
	ghostbox->setOnCollided(std::bind(&gCanvas3D::onCollidedBall,this, std::placeholders::_1));
	//---------------------------------------------------------
}

void gCanvas3D::onCollidedBall(int targetid) {
	gLogi("Ghostbox collided with ") << physicworld->getObject(targetid)->getName();
}

void gCanvas3D::update() {
	//gLogi("gCanvas3D") << "update";

	physicworld->runPhysicWorldStep();



	if(keystate & KEY_W) {
		camera.dolly(-1.0f);
	} else if(keystate & KEY_S) {
		camera.dolly(1.0f);
	}

	if(keystate & KEY_D) {
		camera.truck(1.0f);
	} else if(keystate & KEY_A) {
		camera.truck(-1.0f);
	}

	sun2.pan((std::rand() % 100) * 0.001f);
	sun2.tilt((std::rand() % 100) * 0.001f);
	sun2.roll((std::rand() % 100) * 0.001f);


}

void gCanvas3D::draw() {
	//gLogi("gCanvas3D") << "draw";
	//logo.draw((getWidth() - logo.getWidth()) / 2, (getHeight() - logo.getHeight()) / 2);

	drawScene();
}

void gCanvas3D::drawScene() {
	camera.begin();
	enableDepthTest();
	sun1.enable();
	//Physic world

	//sun2.enable();
	//sky.draw();
	box2gameobject->draw();
	groundobject->draw();
	renderer->setColor(0, 0, 255);
	box1gameobject->draw();

	physicworld->drawDebug();
	disableDepthTest();
	sun1.disable();
	//sun2.disable();
	camera.end();


}

void gCanvas3D::keyPressed(int key) {
//	gLogi("gCanvas3D") << "keyPressed:" << key;
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

void gCanvas3D::keyReleased(int key) {
//	gLogi("gCanvas3D") << "keyReleased:" << key;
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

void gCanvas3D::charPressed(unsigned int codepoint) {
//	gLogi("gCanvas3D") << "charPressed:" << gCodepointToStr(codepoint);
}

void gCanvas3D::mouseMoved(int x, int y) {
//	gLogi("gCanvas3D") << "mouseMoved" << ", x:" << x << ", y:" << y;
}

void gCanvas3D::mouseDragged(int x, int y, int button) {
//	gLogi("gCanvas3D") << "mouseDragged" << ", x:" << x << ", y:" << y << ", b:" << button;
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

void gCanvas3D::mousePressed(int x, int y, int button) {
//	gLogi("gCanvas3D") << "mousePressed" << ", button:" << button;
	mouseoldx = x;
	mouseoldy = y;
}

void gCanvas3D::mouseReleased(int x, int y, int button) {
//	gLogi("gCanvas3D") << "mouseReleased" << ", button:" << button;
}

void gCanvas3D::mouseScrolled(int x, int y) {
//	gLogi("gCanvas3D") << "mouseScrolled" << ", x:" << x << ", y:" << y;
}

void gCanvas3D::mouseEntered() {
}

void gCanvas3D::mouseExited() {
}

void gCanvas3D::windowResized(int w, int h) {
}

void gCanvas3D::showNotify() {
}

void gCanvas3D::hideNotify() {
}
