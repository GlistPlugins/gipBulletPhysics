/*
 * gDebugDraw.h
 *
 *  Created on: 16 �ub 2023
 *      Author: Remzi ����
 */

#ifndef SRC_GDEBUGDRAW_H_
#define SRC_GDEBUGDRAW_H_

#include "btIDebugDraw.h"
#include "gRenderer.h"
#include "gLine.h"
#include "gCircle.h"
#include "gTriangle.h"
#include "gRect.h"
#include "gSphere.h"
#include "gFont.h"

class gipDebugDraw : public btIDebugDraw
{
int m_debugMode = DBG_DrawWireframe;
bool is2dworld = true;
public:

gipDebugDraw(bool is2d);
virtual ~gipDebugDraw();

virtual void    drawLine(const btVector3& from,const btVector3& to,const btVector3&  fromColor, const btVector3& toColor);

virtual void    drawLine(const btVector3& from,const btVector3& to,const btVector3& color);

virtual void    drawSphere (const btVector3& p, btScalar radius, const btVector3& color);

virtual void 	drawSphere (btScalar radius, const btTransform &transform, const btVector3 &color);

virtual void    drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha);

virtual void    drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);

virtual void    reportErrorWarning(const char* warningString);


virtual void    draw3dText(const btVector3& location,const char* textString);

virtual void    setDebugMode(int debugMode);

virtual int     getDebugMode() const { return m_debugMode;}

void setIs2d(bool is2d);

};


#endif /* SRC_GDEBUGDRAW_H_ */
