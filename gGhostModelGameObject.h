/*
 * gGhostGameObject3D.h
 *
 *  Created on: 12 Mar 2023
 *      Author: Remzi ISCI
 */

/*
 *  This class is inherited from gipBaseGameObject
 *  This clas uses 2d coordinate system
 *  Layers are bitwise
 *  Rotations have been set according to degree format
 *
 *  Ghost object doesn't get effected by physics world velocity and forces
 */

#ifndef SRC_GGHOSTMODELGAMEOBJECT_H_
#define SRC_GGHOSTMODELGAMEOBJECT_H_


#include "gipBaseGameObject.h"
#include "gipBulletPhysics.h"


class gGhostModelGameObject : public gipBaseGameObject {
public:
	gGhostModelGameObject(gipBulletPhysics* physicworld);
	virtual ~gGhostModelGameObject();

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

protected:


private:

};


#endif /* SRC_GGHOSTMODELGAMEOBJECT_H_ */
