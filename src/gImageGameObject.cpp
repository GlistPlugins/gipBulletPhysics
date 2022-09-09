/*
 * gImageGameObject.cpp
 *
 *  Created on: 24 Aug 2022
 *      Author: Faruk Aygun
 */

#include <gImageGameObject.h>

gImageGameObject::gImageGameObject(gImage image, float mass, glm::vec2 position, float rotationAngle) {
	this->image = image;
	this->mass = mass;
	this->position = position;
	this->rotationAngle = rotationAngle;
	width = image.getWidth();
	height = image.getHeight();
}

gImageGameObject::~gImageGameObject() {
	// TODO Auto-generated destructor stub
}

void gImageGameObject::draw() {

}

void gImageGameObject::loadImage(std::string imagePath) {
	image.loadImage(imagePath);
	width = image.getWidth();
	height = image.getHeight();
}

void gImageGameObject::setImage(gImage image) {
	this->image = image;
	width = image.getWidth();
	height = image.getHeight();
}

void gImageGameObject::setId(int id) {
	this->id = id;
}

void gImageGameObject::setMass(float mass) {
	this->mass = mass;
}

void gImageGameObject::setPosition(glm::vec2 position) {
	this->position = position;
}

void gImageGameObject::setRotationAngle(float rotationAngle) {
	this->rotationAngle = rotationAngle;
}


gImage* gImageGameObject::getImage() {
	return &image;
}

int gImageGameObject::getId() {
	return id;
}

float gImageGameObject::getMass() {
	return mass;
}

float gImageGameObject::getWidth() {
	return width;
}

float gImageGameObject::getHeight() {
	return height;
}

float gImageGameObject::getRotationAngle() {
	return rotationAngle;
}

glm::vec2 gImageGameObject::getPosition() {
	return position;
}
