/*
 * gPhysic2dBox.cpp
 *
 *  Created on: 28 �ub 2023
 *      Author: Remzi ����
 */
#ifndef SRC_GIPPHYSIC3DBOX_H_
#define SRC_GIPPHYSIC3DBOX_H_


#include "bases/gipPhysicObject.h"
#include "bases/gipBulletPhysics.h"

class gipPhysic3dBox:public gipPhysicObject {
public:
	/*
	 * layers are bitewise
	 */
	//Constructer for 3d model
	gipPhysic3dBox(gMesh* model, bool isstatic = true, float mass = 0.0f, int objectlayers = -1, int masklayers = -1);
	//Constructer for physic object without any content
	gipPhysic3dBox( int width = 200, int height = 200, int depth = 200, bool isstatic = true, float mass = 0.0f, int objectlayers = -1, int masklayers = -1);

	virtual ~gipPhysic3dBox();
	inline void draw() override;
	void setRendererObjectSize() override;
	void setRendererObjectPosition() override;
	void setRendererObjectRotation() override;

protected:

private:

};



#endif /* SRC_GIPPHYSIC3DBOX_H_ */
