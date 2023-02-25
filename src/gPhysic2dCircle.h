/*
 * gPhysic2dCircle.h
 *
 *  Created on: 23 Þub 2023
 *      Author: Remzi ÝÞÇÝ
 */

#ifndef SRC_GPHYSIC2DCIRCLE_H_
#define SRC_GPHYSIC2DCIRCLE_H_


#include "bases/gPhysicObject.h"
#include "bases/gPhysic.h"


class gPhysic2dCircle:public gPhysicObject {
public:
	//Constructer for 2d image
	gPhysic2dCircle(gImage* image, bool isstatic = true, float mass = 0.0f, float radius = 200, glm::vec3 rotation = glm::vec3(0.0f, 0.0f,0.0f), glm::vec3 position = glm::vec3(0.0f, 0.0f,0.0f));
	//Constructer for 3d model
	gPhysic2dCircle(gModel* model, bool isstatic = true, float mass = 0.0f);
	//Constructer for physic object without any content
	gPhysic2dCircle(bool isstatic = true, float mass = 0.0f, float radius = 200, glm::vec3 rotation = glm::vec3(0.0f, 0.0f,0.0f), glm::vec3 position = glm::vec3(0.0f, 0.0f,0.0f));


	virtual ~gPhysic2dCircle();
	inline void draw() override;
	void setRendererObjectSize() override;
	void setRendererObjectPosition() override;
	void setRendererObjectRotation() override;

protected:

private:

};



#endif /* SRC_GPHYSIC2DCIRCLE_H_ */
