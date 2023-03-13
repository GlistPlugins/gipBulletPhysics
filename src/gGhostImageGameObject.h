/*
 * gGhostImageGameObject.h
 *
 *  Created on: 12 Mar 2023
 *      Author: Remzi ISCI
 */
/*
 *  This class inherit from gipPhysicObject
 *  This clas uses 2d coordinate system
 *  Layers are bit wise
 *  Rotaions have been setted according degree format
 *
 *  Ghost object doesnt get any effect by physic world velocity and forces
 */
#ifndef SRC_GGHOSTIMAGEGAMEOBJECT_H_
#define SRC_GGHOSTIMAGEGAMEOBJECT_H_

#include "gipBaseGameObject.h"
#include "gipBulletPhysics.h"


class gGhostImageGameObject : public gipBaseGameObject {
public:
	gGhostImageGameObject(gipBulletPhysics* physicworld);
	virtual ~gGhostImageGameObject();


	//Will load image from assest/images
	void loadImage(std::string imagepath);

	//Will load image with full file path
	void load(std::string fullpath);

	//Will get image from external source
	void setImage(gImage* sourceimage);

	//Will remove image source
	void clearImage();

	//Call this function to draw image
	void draw();
protected:


private:

};


#endif /* SRC_GGHOSTIMAGEGAMEOBJECT_H_ */
