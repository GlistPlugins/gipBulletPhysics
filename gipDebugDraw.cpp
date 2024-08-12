/*
 * gDebugDraw.cpp
 *
 *  Created on: 16 02 2023
 *      Author: Remzi ISCI
 */

/*
 * The Glist Engine references the top left corner for object positions;
 * but the bullet3 library references the center of transform.
 * Glist engine Y axis is opposite to bullet physics y axis we need to convert y axis by multiplying by -1
 * so we should convert Glist positions to bullet3 positions with (+img.getHeight()).
 */
#include "gipDebugDraw.h"
#include "gRenderer.h"

gipDebugDraw::gipDebugDraw(int worldcoordinate){
	this->_worldcoordinate = worldcoordinate;
}

gipDebugDraw::~gipDebugDraw() {
	// TODO Auto-generated destructor stub
}

void gipDebugDraw::setDebugMode(int debugMode) {
	this->m_debugMode = debugMode;
}

void gipDebugDraw::drawLine(const btVector3& from,const btVector3& to,const btVector3&  fromColor, const btVector3& toColor) {
	gLine* line = new gLine();

	gColor oldcolor = line->getRenderer()->getColor();
	line->getRenderer()->setColor(fromColor.x(), fromColor.y(), fromColor.z());
	if(_worldcoordinate == 0) line->draw(from.getX(), -from.getY(), to.getX(), -to.getY());
	else line->draw(from.getX(), -from.getY(),  from.getZ(),to.getX(), -to.getY(), to.getZ());
	line->getRenderer()->setColor(oldcolor);
}

void gipDebugDraw::drawLine(const btVector3& from,const btVector3& to,const btVector3&  fromColor) {
	gLine* line = new gLine();

	gColor oldcolor = line->getRenderer()->getColor();
	line->getRenderer()->setColor(fromColor.x(), fromColor.y(), fromColor.z());
	if(_worldcoordinate == 0) line->draw(from.getX(), -from.getY(), to.getX(), -to.getY());
	else line->draw(from.getX(), -from.getY(),  from.getZ(),to.getX(), -to.getY(), to.getZ());
	line->getRenderer()->setColor(oldcolor);

}

void gipDebugDraw::drawSphere (btScalar radius, const btTransform &transform, const btVector3 &color) {
	gCircle* circle = new gCircle();
	gColor oldcolor = circle->getRenderer()->getColor();
	circle->getRenderer()->setColor(color.x(), color.y(), color.z());
	if(_worldcoordinate == 0) {
		circle->draw(transform.getOrigin().x(), -transform.getOrigin().y(), radius, false, 8);
	} else {
		circle->draw(transform.getOrigin().x(), -transform.getOrigin().y(), radius, false, 8);
	}
	circle->getRenderer()->setColor(oldcolor);
}


void gipDebugDraw::drawSphere (const btVector3& p, btScalar radius, const btVector3& color) {
	gCircle* circle = new gCircle();
	gColor oldcolor = circle->getRenderer()->getColor();
	circle->getRenderer()->setColor(color.x(), color.y(), color.z());
	circle->draw(p.x(), -p.y(), radius, true, 8);
	circle->getRenderer()->setColor(oldcolor);
}

void gipDebugDraw::drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha) {
	gTriangle* triangle = new gTriangle();
	gColor oldcolor = triangle->getRenderer()->getColor();
	triangle->getRenderer()->setColor(color.x(), color.y(), color.z());
	triangle->draw(a.x(), -a.y(), b.x(), -b.y(), c.x(), -c.y(), false);
	triangle->getRenderer()->setColor(oldcolor);

}


void gipDebugDraw::reportErrorWarning(const char* warningString) {
	std::cout << "Physic world debug draw warning : " << warningString;
}

void gipDebugDraw::drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color) {
	gCircle* circle = new gCircle();
	gColor oldcolor = circle->getRenderer()->getColor();
	circle->draw(PointOnB.x(), -PointOnB.y(), 10.0f, true, 8);
//	gLine* line = new gLine();
	//TODO: need to write codes for this
	circle->getRenderer()->setColor(oldcolor);
}

void gipDebugDraw::draw3dText(const btVector3& location,const char* textString) {
	gFont font = gFont();
	font.loadFont("FreeSans.ttf", 22);
	font.drawText(textString, location.x(), location.y());

}
