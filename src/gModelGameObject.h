/*
 * gModelGameObject.h
 *
 *  Created on: 15 Þub 2023
 *      Author: remzi
 */

#ifndef SRC_GMODELGAMEOBJECT_H_
#define SRC_GMODELGAMEOBJECT_H_



#include "gModel.h"
#include "glm/glm.hpp"

class gModelGameObject {
public:

	gModelGameObject(gModel objectModel, float mass, glm::vec3 position, glm::vec3 rotationAngle, float scale);
	virtual ~gModelGameObject();

	void draw();
	void loadModel(std::string modelPath);
	void setId(int id);
	void setMass(float mass);
	void setPosition(glm::vec3 position);
	void rotateAround(float rad, glm::vec3, glm::vec3 rotationCenter);

	gModel* getModel();

	int getId();

	float getMass();
	float getScale();
	glm::vec3 getPosition();

private:
	gModel model;

	float mass;
	glm::vec3 rotationAngle;
	glm::vec3 position;

	int id = -1;
	float scale;
};


#endif /* SRC_GMODELGAMEOBJECT_H_ */
