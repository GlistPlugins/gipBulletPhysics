/*
 * gImageGameObject.h
 *
 *  Created on: 24 Aug 2022
 *      Author: Faruk Aygun
 */

#ifndef SRC_GIMAGEGAMEOBJECT_H_
#define SRC_GIMAGEGAMEOBJECT_H_

#include "gImage.h"

#include "glm/glm.hpp"

class gImageGameObject {
public:
	gImageGameObject(gImage image, float mass, float positionx, float positiony, float rotationx, float rotationy);
	virtual ~gImageGameObject();

	void draw();
	// TODO: create method body.
	void loadImage(std::string imageName);

	// TODO: set access modifier private and create get-set methods
	// TODO: create (float)objMass variable
	gImage image;

	int id = -1;
	float mass;
	float positionx;
	float positiony;
	float rotationx;
	float rotationy;
};

#endif /* SRC_GIMAGEGAMEOBJECT_H_ */
