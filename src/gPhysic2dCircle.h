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
	/*
	 * layers are bitewise
	 */
	//Constructer for 2d image
	gPhysic2dCircle(gImage* image, bool isstatic = true, float mass = 0.0f, float radius = 200, int objectlayers = -1, int masklayers = -1);
	//Constructer for 3d model
	gPhysic2dCircle(gMesh* model, bool isstatic = true, float mass = 0.0f, int objectlayers = -1, int masklayers = -1);
	//Constructer for physic object without any content
	gPhysic2dCircle(bool isstatic = true, float mass = 0.0f, float radius = 200, int objectlayers = -1, int masklayers = -1);


	virtual ~gPhysic2dCircle();
	inline void draw() override;
	void setRendererObjectSize() override;
	void setRendererObjectPosition() override;
	void setRendererObjectRotation() override;

protected:

private:

};



#endif /* SRC_GPHYSIC2DCIRCLE_H_ */
