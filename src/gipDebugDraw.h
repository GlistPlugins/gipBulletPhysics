/*
* gDebugDraw.h
 *
 *  Created on: 16 02 2023
 *      Author: Remzi ISCI
 */

#ifndef SRC_GDEBUGDRAW_H_
#define SRC_GDEBUGDRAW_H_

#include "bullet/btBulletCollisionCommon.h"
#include "bullet/btBulletDynamicsCommon.h"
#include "gLine.h"
#include "gCircle.h"
#include "gTriangle.h"
#include "gRect.h"
#include "gSphere.h"
#include "gFont.h"

class gipDebugDraw : public btIDebugDraw {
public:
	/*
	 * worldcoordinate enum: 0 = 2D , 1 = 3D
	 */
	gipDebugDraw(int worldcoordinate = 0);
	~gipDebugDraw() override;

	void drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor,
	              const btVector3& toColor) override;
	void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override;
	void drawSphere(const btVector3& p, btScalar radius, const btVector3& color) override;
	void drawSphere(btScalar radius, const btTransform& transform, const btVector3& color) override;
	void drawTriangle(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& color,
	                  btScalar alpha) override;
	void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance,
	                      int lifeTime, const btVector3& color) override;
	void reportErrorWarning(const char* warningString) override;
	void draw3dText(const btVector3& location, const char* textString) override;
	void setDebugMode(int debugMode) override { debugmode = debugMode; }
	int getDebugMode() const override { return debugmode; }
	void setMaxDrawDistance(float distance);

private:
	int debugmode;
	float maxDrawDistanceSq = 10000.0f * 10000.0f; // 10k units default
	int worldcoordinate = 0;
	gFont font;

	bool isVisible(const glm::vec3& point, float radius = 0.0f);
	bool isVisible(const glm::vec3& minPoint, const glm::vec3& maxPoint);
};


#endif /* SRC_GDEBUGDRAW_H_ */