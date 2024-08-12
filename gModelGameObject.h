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

	// Loads model from assets/images
	void loadModel(std::string modelpath);

	// Loads model from full file path
	void load(std::string fullpath);

	// Loads model from already existing source
	void setModel(gModel* sourcemodel);

	// Loads mesh from already existing source
	void setMesh(gMesh* sourcemesh);

	// Turns own model off
	void clearModel();

	// Turns own mesh off
	void clearMesh();

	// Call this function to draw image
	void draw();

private:

};


#endif /* SRC_GMODELGAMEOBJECT_H_ */
