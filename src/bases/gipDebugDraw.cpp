/*
 * gDebugDraw.cpp
 *
 *  Created on: 16 Þub 2023
 *      Author: Remzi ÝÞÇÝ
 */
/*
 * Engine Y axis direciton is opposite to coming data that comes from bullet physic Y axis direction
 *
 *
 *
 */
#include "gipDebugDraw.h"


gipDebugDraw::gipDebugDraw(bool is2d) {
	this->is2dworld = is2d;
}



gipDebugDraw::~gipDebugDraw() {
	// TODO Auto-generated destructor stub
}

void gipDebugDraw::setDebugMode(int debugMode) {
	this->m_debugMode = debugMode;


}

void gipDebugDraw::drawLine(const btVector3& from,const btVector3& to,const btVector3&  fromColor, const btVector3& toColor) {
	gLine* line = new gLine();
	if(is2dworld) line->draw(from.getX(), -from.getY(), to.getX(), -to.getY());
	else line->draw(from.getX(), -from.getY(),  from.getZ(),to.getX(), -to.getY(), to.getZ());
}

void gipDebugDraw::drawLine(const btVector3& from,const btVector3& to,const btVector3&  fromColor) {
	gLine* line = new gLine();
	if(is2dworld) line->draw(from.getX(), -from.getY(), to.getX(), -to.getY());
	else line->draw(from.getX(), -from.getY(),  from.getZ(),to.getX(), -to.getY(), to.getZ());
}

void gipDebugDraw::drawSphere (btScalar radius, const btTransform &transform, const btVector3 &color) {
	if(is2dworld) {
		gCircle* circle = new gCircle();
		circle->draw(transform.getOrigin().x(), -transform.getOrigin().y(), radius, false, 8);
	} else {
		gCircle* circle = new gCircle();
		circle->draw(transform.getOrigin().x(), -transform.getOrigin().y(), radius, false, 8);
	}
}


void gipDebugDraw::drawSphere (const btVector3& p, btScalar radius, const btVector3& color) {
	gCircle* circle = new gCircle();
	circle->draw(p.x(), -p.y(), radius, true, 8);
}

void gipDebugDraw::drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha) {
	gTriangle* triangle = new gTriangle();
	triangle->draw(a.x(), -a.y(), b.x(), -b.y(), c.x(), -c.y(), false);

}


void gipDebugDraw::reportErrorWarning(const char* warningString) {
	std::cout << "Physic world debug draw warning : " << warningString;
}

void gipDebugDraw::drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color) {
	gCircle* circle = new gCircle();
	circle->draw(PointOnB.x(), -PointOnB.y(), 10.0f, true, 8);
	gLine* line = new gLine();
	//TODO: need to write codes for this
}

void gipDebugDraw::draw3dText(const btVector3& location,const char* textString) {
	gFont font = gFont();
	font.loadFont("FreeSans.ttf", 22);
	font.drawText(textString, location.x(), location.y());

}

