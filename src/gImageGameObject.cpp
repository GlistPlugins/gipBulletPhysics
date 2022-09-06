/*
 * gImageGameObject.cpp
 *
 *  Created on: 24 Aug 2022
 *      Author: Faruk Aygun
 */

#include <gImageGameObject.h>

gImageGameObject::gImageGameObject(gImage image, float mass, glm::vec2 position, glm::vec2 rotation) {
	this->image = image;
	this->mass = mass;
	this->position = position;
	this->rotation = rotation;
}

gImageGameObject::~gImageGameObject() {
	// TODO Auto-generated destructor stub
}

void gImageGameObject::draw() {

}

void gImageGameObject::loadImage(std::string imagePath) {
	this->image.loadImage(imagePath);
}

void gImageGameObject::setImage(gImage image) {
	this->image = image;
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

void gImageGameObject::setRotation(glm::vec2 rotation) {
	this->rotation = rotation;
}

gImage gImageGameObject::getImage() {
	return this->image;
}

int gImageGameObject::getId() {
	return this->id;
}

float gImageGameObject::getMass() {
	return this->mass;
}

float gImageGameObject::getWidth() {
	return this->image.getWidth();
}

float gImageGameObject::getHeight() {
	return this->image.getHeight();
}

glm::vec2 gImageGameObject::getPosition() {
	return this->position;
}

glm::vec2 gImageGameObject::getRotation() {
	return this->rotation;
}
