/*
 * gDebugDraw.cpp
 *
 *  Created on: 16 �ub 2023
 *      Author: Remzi ����
 */
/*
 * Engine Y axis direciton is opposite to coming data that comes from bullet physic Y axis direction
 *
 *
 *
 */
#include "gDebugDraw.h"
#include "gRenderer.h"

gDebugDraw::gDebugDraw(){

}

gDebugDraw::~gDebugDraw() {
	// TODO Auto-generated destructor stub
}

void gDebugDraw::setDebugMode(int debugMode) {
	this->m_debugMode = debugMode;

}

void gDebugDraw::drawLine(const btVector3& from,const btVector3& to,const btVector3&  fromColor, const btVector3& toColor) {
	gLine* line = new gLine();
	line->draw(from.getX(), -from.getY(),  from.getZ(),to.getX(), -to.getY(), to.getZ());
}

void gDebugDraw::drawLine(const btVector3& from,const btVector3& to,const btVector3&  fromColor) {
	gLine* line = new gLine();
	line->draw(from.getX(), -from.getY(),  from.getZ(),to.getX(), -to.getY(), to.getZ());}

void gDebugDraw::drawSphere (btScalar radius, const btTransform &transform, const btVector3 &color) {
	gCircle* circle = new gCircle();
	circle->draw(transform.getOrigin().x(), -transform.getOrigin().y(), radius, false, 8);
}


void gDebugDraw::drawSphere (const btVector3& p, btScalar radius, const btVector3& color) {
	gCircle* circle = new gCircle();
	circle->draw(p.x(), -p.y(), radius, true, 8);
}

void gDebugDraw::drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha) {
	gTriangle* triangle = new gTriangle();
	triangle->draw(a.x(), -a.y(), b.x(), -b.y(), c.x(), -c.y(), false);
}


void gDebugDraw::reportErrorWarning(const char* warningString) {
	std::cout << "Physic world debug draw warning : " << warningString;
}

void gDebugDraw::drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color) {
	//gCircle* circle = new gCircle();
	//circle->draw(PointOnB.x(), -PointOnB.y(), 20.0f, true, 8);
	//TODO: need to write codes for this
}

void gDebugDraw::draw3dText(const btVector3& location,const char* textString) {

}

