/*
 * gImageGameObject.h
 *
 *  Created on: 24 Aðu 2022
 *      Author: Faruk
 */

#ifndef SRC_GIMAGEGAMEOBJECT_H_
#define SRC_GIMAGEGAMEOBJECT_H_

#include "gImage.h"

#include "glm/glm.hpp"

class gImageGameObject {
public:
	gImageGameObject(gImage image, float positionx, float positiony, float rotationx, float rotationy);
	virtual ~gImageGameObject();

	void draw();

	gImage image;

	float positionx;
	float positiony;
	float rotationx;
	float rotationy;
};

#endif /* SRC_GIMAGEGAMEOBJECT_H_ */
