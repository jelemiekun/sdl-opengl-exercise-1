#pragma once

#include <bullet/btBulletDynamicsCommon.h>
#include <iostream>

class DebugDrawer : public btIDebugDraw {
public:
    DebugDrawer();
    virtual ~DebugDrawer();

    // btIDebugDraw overrides
    virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override;
    virtual void drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB,
                                  btScalar distance, int lifeTime, const btVector3& color) override;
    virtual void reportErrorWarning(const char* warningString) override;
    virtual void draw3dText(const btVector3& location, const char* textString) override;
    virtual void setDebugMode(int debugMode) override;
    virtual int getDebugMode() const override;

private:
    int m_debugMode;
};
