/*
 * gPhysic2dBox.h
 *
 *  Created on: 20 �ub 2023
 *      Author: Remzi ����
 */

#ifndef SRC_GIPPHYSIC2DBOX_H_
#define SRC_GIPPHYSIC2DBOX_H_

#include "bases/gipPhysicObject.h"
#include "bases/gipBulletPhysics.h"

class gipPhysic2dBox:public gipPhysicObject {
public:
	/*
	 * layers are bitewise
	 */
	//Constructer for 2d image
	gipPhysic2dBox(gImage* image, bool isstatic = true, float mass = 0.0f, int objectlayers = -1, int masklayers = -1);
	//Constructer for physic object without any content
	gipPhysic2dBox( int width = 200, int height = 200, bool isstatic = true, float mass = 0.0f, int objectlayers = -1, int masklayers = -1);

	virtual ~gipPhysic2dBox();
	inline void draw() override;
	void setRendererObjectSize() override;
	void setRendererObjectPosition() override;
	void setRendererObjectRotation() override;

protected:

private:

};


#endif /* SRC_GIPPHYSIC2DBOX_H_ */
