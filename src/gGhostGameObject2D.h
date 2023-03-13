/*
 * gGhostGameObject2D.h
 *
 *  Created on: 12 Mar 2023
 *      Author: Remzi ISCI
 */
/*
 *  This class inherit from gipPhysicObject
 *  This clas uses 2d coordinate system
 *  Layers are bit wise
 *  Rotaions have been setted according degree format
 */
#ifndef SRC_GGHOSTGAMEOBJECT2D_H_
#define SRC_GGHOSTGAMEOBJECT2D_H_

#include "gipBaseGameObject.h"
#include "gipBulletPhysics.h"


class gGhostGameObject2D : public gipBaseGameObject {
public:
	gGhostGameObject2D(gipBulletPhysics* physicworld);
	virtual ~gGhostGameObject2D();


protected:


private:

};


#endif /* SRC_GGHOSTGAMEOBJECT2D_H_ */
