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
	gImageGameObject(gImage image, float mass, glm::vec2 position, float rotationAngle);
	virtual ~gImageGameObject();

	void draw();
	void loadImage(std::string imagePath);
	void setImage(gImage image);
	void setId(int id);
	void setMass(float mass);
	void setPosition(glm::vec2 position);
	void setRotationAngle(float angle);

	gImage* getImage();

	int getId();

	float getMass();
	float getWidth();
	float getHeight();
	float getRotationAngle();

	glm::vec2 getPosition();

private:
	gImage image;

	float mass;
	float rotationAngle;

	glm::vec2 position;

	int id = -1;
	int width;
	int height;
};

#endif /* SRC_GIMAGEGAMEOBJECT_H_ */
