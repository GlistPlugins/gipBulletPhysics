/*
 * gImageGameObject.cpp
 *
 *  Created on: 24 Aug 2022
 *      Author: Faruk Aygun
 */

#include <gImageGameObject.h>

gImageGameObject::gImageGameObject(gImage image, float mass, float positionx, float positiony, float rotationx, float rotationy) {
	this->image = image;
	this->mass = mass;
	this->positionx = positionx;
	this->positiony = positiony;
	this->rotationx = rotationx;
	this->rotationy = rotationy;
}

gImageGameObject::~gImageGameObject() {
	// TODO Auto-generated destructor stub
}

void gImageGameObject::draw() {

}
