#include "Trackball.hpp"


using namespace gui;


void Trackball::handleMouseMove(int dx, int dy)
{
    theta -= dx * speed;
    phi -= dy * speed;

    theta = fmodf(theta, 2 * PI);
    phi = fmodf(phi, 2 * PI);
}

void Trackball::handleMouseScroll(int d)
{
    r -= d * 50;
}

void Trackball::doLookAt(Camera& camera)
{
    camera.lookAt(Vec3GLf(sin(phi) * sin(theta),
                          cos(phi),
                          sin(phi) * cos(theta)) * r,
                  Vec3GLf(0, 680.0f + 0, 0),
                  Vec3GLf(0, sin(phi), 0.0));
}
