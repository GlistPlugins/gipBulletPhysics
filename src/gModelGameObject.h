/*
 * gModelGameObject.h
 *
 *  Edited on : 16.02.2023
 *  	Author: Remzi ISCI
 */

#ifndef SRC_GMODELGAMEOBJECT_H_
#define SRC_GMODELGAMEOBJECT_H_

#include "gipBaseGameObject.h"
#include "gipBulletPhysics.h"

class gModelGameObject : public gipBaseGameObject {
public:
	gModelGameObject(gipBulletPhysics* physicworld);
	virtual ~gModelGameObject();

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

private:

};


#endif /* SRC_GMODELGAMEOBJECT_H_ */
