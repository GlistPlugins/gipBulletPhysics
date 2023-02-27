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
	/*
	 * layers are bitewise
	 */
	//Constructer for 2d image
	gPhysic2dBox(gImage* image, bool isstatic = true, float mass = 0.0f, int objectlayers = -1, int masklayers = -1);
	//Constructer for 3d model
	gPhysic2dBox(gMesh* model, bool isstatic = true, float mass = 0.0f, int objectlayers = -1, int masklayers = -1);
	//Constructer for physic object without any content
	gPhysic2dBox( int width = 200, int height = 200, int depth = 200, bool isstatic = true, float mass = 0.0f, int objectlayers = -1, int masklayers = -1);

	virtual ~gPhysic2dBox();
	inline void draw() override;
	void setRendererObjectSize() override;
	void setRendererObjectPosition() override;
	void setRendererObjectRotation() override;

protected:

private:

};


#endif /* SRC_GPHYSIC2DBOX_H_ */
