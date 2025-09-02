#include "DebugDrawer.h"

DebugDrawer::DebugDrawer() : m_debugMode(DBG_DrawWireframe) {}

DebugDrawer::~DebugDrawer() {}

void DebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
    std::cout << "Draw line from (" << from.x() << ", " << from.y() << ", " << from.z()
              << ") to (" << to.x() << ", " << to.y() << ", " << to.z() << ")\n";
}

void DebugDrawer::drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB,
                                   btScalar distance, int lifeTime, const btVector3& color) {
    // Optional: print contact point
    std::cout << "Contact point at (" << pointOnB.x() << ", " << pointOnB.y() << ", " << pointOnB.z()
              << ") with normal (" << normalOnB.x() << ", " << normalOnB.y() << ", " << normalOnB.z() << ")\n";
}

void DebugDrawer::reportErrorWarning(const char* warningString) {
    std::cerr << "Bullet warning: " << warningString << "\n";
}

void DebugDrawer::draw3dText(const btVector3& location, const char* textString) {
    // Optional: print text
    std::cout << "3D Text at (" << location.x() << ", " << location.y() << ", " << location.z()
              << "): " << textString << "\n";
}

void DebugDrawer::setDebugMode(int debugMode) {
    m_debugMode = debugMode;
}

int DebugDrawer::getDebugMode() const {
    return m_debugMode;
}