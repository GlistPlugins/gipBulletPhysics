/*
 * gPhysic2dCircle.h
 *
 *  Created on: 23 02 2023
 *      Author: Remzi ISCI
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
	gPhysic2dCircle(gImage* image, bool isstatic = true, float mass = 0.0f, float radius = 1, int objectlayers = -1, int masklayers = -1);
	//Constructer for physic object without any content
	gPhysic2dCircle(bool isstatic = true, float mass = 0.0f, float radius = 1, int objectlayers = -1, int masklayers = -1);


	virtual ~gPhysic2dCircle();
	inline void draw() override;
	void setRendererObjectSize() override;
	void setRendererObjectPosition() override;
	void setRendererObjectRotation() override;

protected:

private:

};



#endif /* SRC_GPHYSIC2DCIRCLE_H_ */
