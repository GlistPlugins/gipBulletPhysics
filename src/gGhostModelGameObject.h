/*
 * gGhostGameObject3D.h
 *
 *  Created on: 12 Mar 2023
 *      Author: Remzi ISCI
 *
 *  This class inherit from gipPhysicObject
 *  This clas uses 2d coordinate system
 *  Layers are bit wise
 *  Rotaions have been setted according degree format
 *
 *   Ghost object doesnt get any effect by physic world velocity and forces
 */

#ifndef SRC_GGHOSTMODELGAMEOBJECT_H_
#define SRC_GGHOSTMODELGAMEOBJECT_H_


#include "gipBaseGameObject.h"
#include "gipBulletPhysics.h"


class gGhostModelGameObject : public gipBaseGameObject {
public:
	gGhostModelGameObject(gipBulletPhysics* physicworld);
	virtual ~gGhostModelGameObject();

	//Will load model from assest/images
	void loadModel(std::string modelpath);

	//Will load model with full file path
	void load(std::string fullpath);

	//Will get model from external source
	void setModel(gModel* sourcemodel);

	//Will get mesh from external source
	void setMesh(gMesh* sourcemesh);

	//Will remove model source
	void clearModel();

	//Will remove mesh source
	void clearMesh();

	//Call this function to draw image
	void draw();

protected:


private:

};


#endif /* SRC_GGHOSTMODELGAMEOBJECT_H_ */
