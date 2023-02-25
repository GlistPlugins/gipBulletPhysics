/*
 * gPhysic2dBox.h
 *
 *  Created on: 20 Þub 2023
 *      Author: Remzi ÝÞÇÝ
 */

#ifndef SRC_GPHYSIC2DBOX_H_
#define SRC_GPHYSIC2DBOX_H_

#include "bases/gPhysicObject.h"
#include "bases/gPhysic.h"


class gPhysic2dBox:public gPhysicObject {
public:
	//Constructer for 2d image
	gPhysic2dBox(gImage* image, bool isstatic = true, float mass = 0.0f, glm::vec3 size = glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3 rotation = glm::vec3(0.0f, 0.0f,0.0f), glm::vec3 position = glm::vec3(0.0f, 0.0f,0.0f));
	//Constructer for 3d model
	gPhysic2dBox(gModel* model, bool isstatic = true, float mass = 0.0f);
	//Constructer for physic object without any content
	gPhysic2dBox(bool isstatic = true, float mass = 0.0f, int width = 200, int height = 200, int depth = 200, glm::vec3 size = glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3 rotation = glm::vec3(0.0f, 0.0f,0.0f), glm::vec3 position = glm::vec3(0.0f, 0.0f,0.0f));

	virtual ~gPhysic2dBox();
	inline void draw() override;
	void setRendererObjectSize() override;
	void setRendererObjectPosition() override;
	void setRendererObjectRotation() override;

protected:

private:

};


#endif /* SRC_GPHYSIC2DBOX_H_ */
