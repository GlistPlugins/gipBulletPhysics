/*
 * gDebugDraw.cpp
 *
 *  Created on: 16 02 2023
 *      Author: Remzi ISCI
 */

#include "gipDebugDraw.h"
#include "gRenderer.h"
#include "gCamera.h"

gipDebugDraw::gipDebugDraw(int worldcoordinate) : worldcoordinate(worldcoordinate) {
	font.loadFont("FreeSans.ttf", 22);
}

gipDebugDraw::~gipDebugDraw() {
}

bool gipDebugDraw::isVisible(const glm::vec3& point, float radius) {
	if (worldcoordinate == 0) {
		return true; // 2D mode, skip culling
	}

	// Distance culling
	glm::vec3 camPos = renderer->getCamera()->getPosition();
	float distSq = glm::dot(point - camPos, point - camPos);
	if (distSq > maxDrawDistanceSq) {
		return false;
	}

	// Frustum culling
	glm::vec3 min = point - glm::vec3(radius);
	glm::vec3 max = point + glm::vec3(radius);
	gBoundingBox bb{min.x, min.y, min.z, max.x, max.y, max.z};
	return renderer->getCamera()->isInFrustum(bb);
}

bool gipDebugDraw::isVisible(const glm::vec3& minPoint, const glm::vec3& maxPoint) {
	if (worldcoordinate == 0) {
		return true;
	}

	// Distance culling using center of bounds
	glm::vec3 center = (minPoint + maxPoint) * 0.5f;
	glm::vec3 camPos = renderer->getCamera()->getPosition();
	float distSq = glm::dot(center - camPos, center - camPos);
	if (distSq > maxDrawDistanceSq) {
		return false;
	}

	gBoundingBox bb{minPoint.x, minPoint.y, minPoint.z, maxPoint.x, maxPoint.y, maxPoint.z};
	return renderer->getCamera()->isInFrustum(bb);
}

void gipDebugDraw::drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor,
                            const btVector3& toColor) {
	glm::vec3 p1(from.getX(), -from.getY(), from.getZ());
	glm::vec3 p2(to.getX(), -to.getY(), to.getZ());

	if (!isVisible(glm::min(p1, p2), glm::max(p1, p2))) {
		return;
	}

	gColor oldcolor = renderer->getColor();
	renderer->setColor(fromColor.x(), fromColor.y(), fromColor.z());
	if (worldcoordinate == 0) {
		renderer->drawLine(from.getX(), -from.getY(), to.getX(), -to.getY());
	} else {
		renderer->drawLine(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
	}
	renderer->setColor(oldcolor);
}

void gipDebugDraw::drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor) {
	glm::vec3 p1(from.getX(), -from.getY(), from.getZ());
	glm::vec3 p2(to.getX(), -to.getY(), to.getZ());

	if (!isVisible(glm::min(p1, p2), glm::max(p1, p2))) {
		return;
	}

	gColor oldcolor = renderer->getColor();
	renderer->setColor(fromColor.x(), fromColor.y(), fromColor.z());
	if (worldcoordinate == 0) {
		renderer->drawLine(from.getX(), -from.getY(), to.getX(), -to.getY());
	} else {
		renderer->drawLine(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
	}
	renderer->setColor(oldcolor);
}

void gipDebugDraw::drawSphere(btScalar radius, const btTransform& transform, const btVector3& color) {
	glm::vec3 center(transform.getOrigin().x(), -transform.getOrigin().y(), -transform.getOrigin().z());

	if (!isVisible(center, radius)) {
		return;
	}

	gColor oldcolor = renderer->getColor();
	renderer->setColor(color.x(), color.y(), color.z());
	if (worldcoordinate == 0) {
		renderer->drawCircle(center.x, -center.y, radius, 8);
	} else {
		renderer->drawSphere(center.x, center.y, center.z, {radius, radius, radius}, 8, 8);
	}
	renderer->setColor(oldcolor);
}

void gipDebugDraw::drawSphere(const btVector3& p, btScalar radius, const btVector3& color) {
	glm::vec3 center(p.x(), -p.y(), p.z());

	if (!isVisible(center, radius)) {
		return;
	}

	gColor oldcolor = renderer->getColor();
	renderer->setColor(color.x(), color.y(), color.z());
	renderer->drawCircle(p.x(), -p.y(), radius, true, 8);
	renderer->setColor(oldcolor);
}

void gipDebugDraw::drawTriangle(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& color,
                                btScalar alpha) {
	glm::vec3 pa(a.x(), -a.y(), a.z());
	glm::vec3 pb(b.x(), -b.y(), b.z());
	glm::vec3 pc(c.x(), -c.y(), c.z());

	glm::vec3 minP = glm::min(glm::min(pa, pb), pc);
	glm::vec3 maxP = glm::max(glm::max(pa, pb), pc);

	if (!isVisible(minP, maxP)) {
		return;
	}

	gColor oldcolor = renderer->getColor();
	renderer->setColor(color.x(), color.y(), color.z());
	renderer->drawTriangle(a.x(), -a.y(), b.x(), -b.y(), c.x(), -c.y(), false);
	renderer->setColor(oldcolor);
}

void gipDebugDraw::reportErrorWarning(const char* warningString) {
	gLogw("gipDebugDraw") << "Physic world debug draw warning : " << warningString;
}

void gipDebugDraw::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance,
                                    int lifeTime, const btVector3& color) {
	glm::vec3 point(PointOnB.x(), -PointOnB.y(), PointOnB.z());

	if (!isVisible(point, 10.0f)) {
		return;
	}

	gColor oldcolor = renderer->getColor();
	renderer->setColor(color.x(), color.y(), color.z());
	renderer->drawCircle(PointOnB.x(), -PointOnB.y(), 10.0f, true, 8);
	renderer->setColor(oldcolor);
}

void gipDebugDraw::draw3dText(const btVector3& location, const char* textString) {
	if (worldcoordinate == 0) {
		font.drawText(textString, location.x(), -location.y());
		return;
	}

	glm::vec3 point(location.x(), -location.y(), location.z());

	if (!isVisible(point, 1.0f)) {
		return;
	}

	// Project 3D world position to 2D screen coordinates
	glm::vec4 worldPos(point, 1.0f);
	glm::mat4 vp = renderer->getProjectionMatrix() * renderer->getViewMatrix();
	glm::vec4 clipPos = vp * worldPos;

	if (clipPos.w <= 0.0f) {
		return;
	}

	glm::vec3 ndc = glm::vec3(clipPos) / clipPos.w;

	float screenX = (ndc.x + 1.0f) * 0.5f * renderer->getWidth();
	float screenY = (1.0f - ndc.y) * 0.5f * renderer->getHeight();

	font.drawText(textString, screenX, screenY);
}

void gipDebugDraw::setMaxDrawDistance(float distance) {
	maxDrawDistanceSq = distance * distance;
}