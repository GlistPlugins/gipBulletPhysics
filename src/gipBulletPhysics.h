/*
 * gipEmpty.h
 *
 *  Created on: 2 Aug 2021
 *      Author: kayra
 */

#ifndef SRC_GIPBULLETPHYSICS_H_
#define SRC_GIPBULLETPHYSICS_H_

#include "gBasePlugin.h"

#include <stdio.h>

class gipBulletPhysics : public gBasePlugin{
public:
	gipBulletPhysics();
	virtual ~gipBulletPhysics();

	void update();
};

#endif /* SRC_GIPBULLETPHYSICS_H_ */
