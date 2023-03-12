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
 */

#ifndef SRC_GGHOSTGAMEOBJECT3D_H_
#define SRC_GGHOSTGAMEOBJECT3D_H_


#include "gipBaseGameObject.h"
#include "gipBulletPhysics.h"


class gGhostGameObject3D : public gipBaseGameObject {
public:
	gGhostGameObject3D(gipBulletPhysics* physicworld);
	virtual ~gGhostGameObject3D();


protected:


private:

};


#endif /* SRC_GGHOSTGAMEOBJECT3D_H_ */
