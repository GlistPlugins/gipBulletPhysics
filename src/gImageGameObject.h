/*
 * gImageGameObject.h
 *
 *  Created on: 4 Mar 2023
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

#ifndef SRC_GIMAGEGAMEOBJECT_H_
#define SRC_GIMAGEGAMEOBJECT_H_

#include "gipBaseGameObject.h"
#include "gipBulletPhysics.h"

class gImageGameObject : public gipBaseGameObject {
public:
	gImageGameObject(gipBulletPhysics* physicworld);
	virtual ~gImageGameObject();

	// Loads image from assets/images
	void loadImage(std::string imagepath);

	// Loads image from full file path
	void load(std::string fullpath);

	// Loads image from already existing source
	void setImage(gImage* sourceimage);

	// Turns own image off
	void clearImage();

	//Call this function to draw image
	void draw();

protected:


private:

};

#endif /* SRC_GIMAGEGAMEOBJECT_H_ */
