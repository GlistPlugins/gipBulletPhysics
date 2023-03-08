/*
 * gModelGameObject.cpp
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ISCI
 */


#include <gModelGameObject.h>

gModelGameObject::gModelGameObject(gModel model, float mass, glm::vec3 position, glm::vec3 rotationAngle, float scale) {
	this->model = model;
	this->mass = mass;
	this->position = position;
	this->rotationAngle = rotationAngle;
	this->scale  = scale;
}

gModelGameObject::~gModelGameObject() {
	// TODO Auto-generated destructor stub
}

void gModelGameObject::draw() {
	model.draw();
}

void gModelGameObject::loadModel(std::string modelPath) {
	model.loadModel(modelPath);
}

void gModelGameObject::setId(int id) {
	this->id = id;
}

void gModelGameObject::setMass(float mass) {
	this->mass = mass;
}

void gModelGameObject::setPosition(glm::vec3 position) {
	this->position = position;
	model.setPosition(position);
}

void gModelGameObject::rotateAround(float rad, glm::vec3 axis, glm::vec3 rotationCenter) {
	model.rotateAround(rad, axis, rotationCenter);
}

int gModelGameObject::getId() {
	return id;
}

float gModelGameObject::getMass() {
	return mass;
}

float gModelGameObject::getScale() {
	return scale;
}

glm::vec3 gModelGameObject::getPosition() {
	return position;
}


