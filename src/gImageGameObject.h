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
	gImageGameObject(gImage image, float mass, glm::vec2 position, glm::vec2 rotation);
	virtual ~gImageGameObject();

	void draw();
	void loadImage(std::string imagePath);
	void setImage(gImage image);
	void setId(int id);
	void setMass(float mass);
	void setPosition(glm::vec2 position);
	void setRotation(glm::vec2 rotation);

	gImage getImage();

	int getId();

	float getMass();
	/*
	 * If you use image.getWidth() or getHeight() instead of these methods,
	 * you will get <terminated> (exit value: -1.073.740.940)
	 * see FIXME on createSoftContactBox2dObject method in gipBulletPhysics.cpp.
	 */
	float getWidth();
	float getHeight();

	glm::vec2 getPosition();
	glm::vec2 getRotation();

private:
	gImage image;

	float mass;

	glm::vec2 position;
	glm::vec2 rotation;

	int id = -1;
};

#endif /* SRC_GIMAGEGAMEOBJECT_H_ */
