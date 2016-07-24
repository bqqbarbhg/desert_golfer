#include "physics.h"
#include "scene.h"
#include "../external/chipmunk/include/chipmunk/chipmunk.h"
#include "../external/nanovg/nanovg.h"

#define X_STEPS 50
#define Y_STEPS 50
#define MAX_VELOCITY 600.0f
#define GRAVITY 120.0f

#define BALL_DENSITY 1.0f

#define BALL_FRICTION 1.0f
#define BALL_ELASTICITY 0.5f

struct PhysicsPoly
{
	cpBody *body;
	cpShape **segments;
	unsigned numSegments;
};

NVGcolor stateColor[] = {
	nvgRGB(0x00, 0x00, 0x00),
	nvgRGB(0x00, 0x00, 0xFF),
	nvgRGB(0x00, 0xFF, 0x00),
	nvgRGB(0xFF, 0x00, 0x00),
	nvgRGB(0xBB, 0x00, 0x00),
};

enum BallState
{
	BALL_NONE,
	BALL_IN_AIR,
	BALL_IN_GOAL,
	BALL_FLY_OUT,
	BALL_STOPPED,
};

struct PhysicsBall
{
	cpBody *body;
	cpShape *shape;
	BallState state;
};

struct PhysicsScene
{
	const Scene *scene;

	cpSpace *space;

	PhysicsPoly *polys;
	unsigned numPolys;

	PhysicsBall *balls;
	unsigned numBalls;

	bool hasSpawnedBalls;

	cpShapeFilter groundFilter;
	cpShapeFilter ballFilter;
};

PhysicsScene *physicsCreateScene(const Scene *scene)
{
	PhysicsScene *p = (PhysicsScene*)malloc(sizeof(PhysicsScene));
	p->scene = scene;

	p->space = cpSpaceNew();

	p->groundFilter = cpShapeFilterNew(1, 1, 2);
	p->ballFilter = cpShapeFilterNew(2, 2, 1);

	cpSpaceSetGravity(p->space, cpv(0.0f, GRAVITY));
	cpSpaceSetSleepTimeThreshold(p->space, 0.5f);
	cpSpaceSetIdleSpeedThreshold(p->space, 15.0f);
	cpSpaceSetDamping(p->space, 0.8f);

	cpSpaceSetIterations(p->space, 20);

	p->hasSpawnedBalls = false;

	p->numPolys = scene->numPolys;
	p->polys = (PhysicsPoly*)malloc(sizeof(PhysicsPoly) * p->numPolys);
	for (unsigned pi = 0; pi < scene->numPolys; pi++)
	{
		ScenePoly *scenePoly = &scene->polys[pi];
		PhysicsPoly *poly = &p->polys[pi];
		poly->body = cpBodyNewStatic();
		cpSpaceAddBody(p->space, poly->body);

		unsigned numVerts = scenePoly->numVertices;
		poly->numSegments = numVerts;
		poly->segments = (cpShape**)malloc(sizeof(cpShape*) * numVerts);

		cpVect *tempVerts = (cpVect*)malloc(sizeof(cpVect) * numVerts);
		for (unsigned i = 0; i < numVerts; i++)
			tempVerts[i] = cpv(scenePoly->vertices[i].x, scenePoly->vertices[i].y);

		for (unsigned i = 0; i < numVerts; i++)
		{
			cpShape *segment = cpSegmentShapeNew(poly->body, tempVerts[i], tempVerts[(i + 1) % numVerts], 0.0f);
			cpSegmentShapeSetNeighbors(segment, tempVerts[(i + (numVerts - 1)) % numVerts], tempVerts[(i + 2) % numVerts]);
			poly->segments[i] = segment;

			cpShapeSetElasticity(segment, sqrtf(BALL_ELASTICITY));
			cpShapeSetFriction(segment, sqrtf(BALL_FRICTION));

			cpShapeSetFilter(segment, p->groundFilter);

			cpSpaceAddShape(p->space, segment);
		}

		free(tempVerts);
	}

	return p;
}

void physicsUpdate(PhysicsScene *scene, int iterations)
{
	if (!scene->hasSpawnedBalls)
	{
		scene->numBalls = X_STEPS * Y_STEPS;
		scene->balls = (PhysicsBall*)calloc(scene->numBalls, sizeof(PhysicsBall));
		cpVect ballPos = cpv(scene->scene->ballPos.x, scene->scene->ballPos.y);

		for (unsigned y = 0; y < Y_STEPS; y++)
		{
			for (unsigned x = 0; x < X_STEPS; x++)
			{
				float fx = (x / (float)(X_STEPS - 1)) * 2.0f - 1.0f;
				float fy = (y / (float)(Y_STEPS - 1)) * 2.0f - 1.0f;

				float vfx = fx * MAX_VELOCITY;
				float vfy = fy * MAX_VELOCITY;

				PhysicsBall *ball = &scene->balls[x + y * X_STEPS];
				ball->state = BALL_NONE;

				if (vfx * vfx + vfy * vfy > MAX_VELOCITY * MAX_VELOCITY)
					continue;

				ball->state = BALL_IN_AIR;

				ball->body = cpBodyNew(1.0f, 1.0f);
				ball->shape = cpCircleShapeNew(ball->body, scene->scene->ballRadius, cpvzero);

				cpShapeSetFilter(ball->shape, scene->ballFilter);

				cpShapeSetElasticity(ball->shape, sqrt(BALL_ELASTICITY));
				cpShapeSetFriction(ball->shape, sqrtf(BALL_FRICTION));

				cpShapeSetDensity(ball->shape, BALL_DENSITY);
				cpBodySetMass(ball->body, cpShapeGetMass(ball->shape));
				cpBodySetMoment(ball->body, cpShapeGetMoment(ball->shape));

				cpBodySetPosition(ball->body, ballPos);
				cpBodySetVelocity(ball->body, cpv(vfx, vfy));

				cpSpaceAddBody(scene->space, ball->body);
				cpSpaceAddShape(scene->space, ball->shape);
			}
		}

		scene->hasSpawnedBalls = true;
	}

	for (int i = 0; i < iterations; i++)
		cpSpaceStep(scene->space, 0.016f);

	for (unsigned i = 0; i < scene->numBalls; i++)
	{
		PhysicsBall *ball = &scene->balls[i];
		if (!ball->body)
			continue;

		cpVect pos = cpBodyGetPosition(ball->body);

		if (pos.x < 0.0f || pos.y < 0.0f || pos.x > scene->scene->screenWidth || pos.y > scene->scene->screenHeight)
			ball->state = BALL_FLY_OUT;

		if (cpBodyIsSleeping(ball->body))
		{
			if (pos.x >= scene->scene->goalMin.x && pos.x <= scene->scene->goalMax.x && pos.y >= scene->scene->goalMin.y && pos.y <= scene->scene->goalMax.y)
				ball->state = BALL_IN_GOAL;
			else
				ball->state = BALL_STOPPED;
		}

		if (ball->state != BALL_IN_AIR)
		{
			cpSpaceRemoveBody(scene->space, ball->body);
			cpSpaceRemoveShape(scene->space, ball->shape);
			cpBodyDestroy(ball->body);
			cpShapeDestroy(ball->shape);
			ball->body = 0;
			ball->shape = 0;
		}
	}
}

void physicsRender(PhysicsScene *scene, void *renderContext)
{
	NVGcontext *ctx = (NVGcontext*)renderContext;

	nvgBeginPath(ctx);
	nvgRect(ctx, 0.0f, 0.0f, (float)scene->scene->screenWidth, (float)scene->scene->screenHeight);
	nvgFillColor(ctx, nvgRGB(0xd6, 0xad, 0x71));
	nvgFill(ctx);

	for (unsigned pi = 0; pi < scene->scene->numPolys; pi++)
	{
		ScenePoly *poly = &scene->scene->polys[pi];

		SceneVec2 first = poly->vertices[0];

		nvgBeginPath(ctx);
		nvgMoveTo(ctx, first.x, first.y);

		for (unsigned i = 1; i < poly->numVertices; i++)
		{
			SceneVec2 pos = poly->vertices[i];
			nvgLineTo(ctx, pos.x, pos.y);
		}
		
		nvgFillColor(ctx, nvgRGB(0xd6, 0x8a, 0x44));
		nvgFill(ctx);
	}

	for (unsigned bi = 0; bi < scene->numBalls; bi++)
	{
		PhysicsBall *ball = &scene->balls[bi];
		if (!ball->body)
			continue;

		cpVect pos = cpBodyGetPosition(ball->body);

		nvgBeginPath(ctx);
		nvgCircle(ctx, (float)pos.x, (float)pos.y, (float)scene->scene->ballRadius);

		nvgFillColor(ctx, nvgRGB(0xFF, 0xFF, 0xFF));
		nvgFill(ctx);
	}

	nvgBeginPath(ctx);
	nvgRect(ctx,
		scene->scene->goalMin.x,
		scene->scene->goalMin.y,
		scene->scene->goalMax.x - scene->scene->goalMin.x,
		scene->scene->goalMax.y - scene->scene->goalMin.y);

	nvgStrokeWidth(ctx, 1.0f);
	nvgStrokeColor(ctx, nvgRGB(0x00, 0x00, 0x00));
	nvgStroke(ctx);

	const float visBlockSize = 3.0f;
	for (int y = 0; y < Y_STEPS; y++)
	{
		for (int x = 0; x < X_STEPS; x++)
		{
			nvgBeginPath(ctx);
			nvgRect(ctx, x * visBlockSize, y * visBlockSize, visBlockSize, visBlockSize);
			nvgFillColor(ctx, stateColor[scene->balls[x + y * X_STEPS].state]);
			nvgFill(ctx);
		}
	}
}


