/*
 * gImageGameObject.h
 *
 *  Created on: 4 Mar 2023
 *      Author: Remzi ISCI
 */
/*
 *  This class inherit from gipPhysicObject
 *  This clas uses 2d coordinate system
 *  Layers are bit wise
 *  Rotaions have been setted according degree format
 */

#ifndef SRC_GIMAGEGAMEOBJECT_H_
#define SRC_GIMAGEGAMEOBJECT_H_

#include "gipBaseGameObject.h"
#include "gipBulletPhysics.h"

class gImageGameObject : public gipBaseGameObject {
public:
	gImageGameObject(gipBulletPhysics* physicworld);
	virtual ~gImageGameObject();

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

#endif /* SRC_GIMAGEGAMEOBJECT_H_ */
