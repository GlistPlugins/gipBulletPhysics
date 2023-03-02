/*
 * gPhysic2dCircle.h
 *
 *  Created on: 23 02 2023
 *      Author: Remzi ISCI
 */

#ifndef SRC_GIPPHYSIC2DCIRCLE_H_
#define SRC_GIPPHYSIC2DCIRCLE_H_


#include "bases/gipPhysicObject.h"
#include "bases/gipBulletPhysics.h"


class gipPhysic2dCircle:public gipPhysicObject {
public:
	/*
	 * layers are bitewise
	 */
	//Constructer for 2d image
	gipPhysic2dCircle(gImage* image, bool isstatic = true, float mass = 0.0f, float radius = 1, int objectlayers = -1, int masklayers = -1);
	//Constructer for physic object without any content
	gipPhysic2dCircle(bool isstatic = true, float mass = 0.0f, float radius = 1, int objectlayers = -1, int masklayers = -1);


	virtual ~gipPhysic2dCircle();
	inline void draw() override;
	void setRendererObjectSize() override;
	void setRendererObjectPosition() override;
	void setRendererObjectRotation() override;

protected:

private:

};



#endif /* SRC_GIPPHYSIC2DCIRCLE_H_ */
