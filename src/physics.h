#pragma once

struct PhysicsScene;
struct Scene;

PhysicsScene *physicsCreateScene(const Scene *scene);
void physicsUpdate(PhysicsScene *scene, int iterations);
void physicsRender(PhysicsScene *scene, void *renderContext);

