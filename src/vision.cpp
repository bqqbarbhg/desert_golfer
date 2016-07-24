#define _CRT_SECURE_NO_WARNINGS

#include "../external/nanovg/stb_image.h"
#include "vision.h"
#include "scene.h"

// #define STB_IMAGE_WRITE_IMPLEMENTATION
// #include "stb_image_write.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define IPHONE 1

inline int minint(int a, int b)
{
	return a < b ? a : b;
}

inline int maxint(int a, int b)
{
	return a > b ? a : b;
}

#define FILTER_SIZE 5
#define THRESHOLD 10
#define BLOCK_SIZE 7
#define BLOCK_INNER_THRESHOLD 30
#define BLOCK_QUALITY_THRESHOLD ((BLOCK_SIZE) * (BLOCK_SIZE))
#define FLOOD_FILL_THRESHOLD 40
#define NEIGHBOR_THRESHOLD 80
#define NEIGHBOR_RADIUS 3
#define MIN_COMPONENT_BLOCKS ((30*30) / (BLOCK_SIZE))
#define MIN_OBJECT_PIXELS 5
#define OBJECT_THRESHOLD 20
#define OBJECT_NEIGHBOR_THRESHOLD 40

#define Count(arr) (sizeof(arr) / sizeof(*(arr)))

stbi_uc g_debugColors[][3] =
{
	{ 0xFF, 0x00, 0x00 },
	{ 0x00, 0xFF, 0x00 },
	{ 0x00, 0x00, 0xFF },
	{ 0xFF, 0xFF, 0x00 },
	{ 0xFF, 0x00, 0xFF },
	{ 0x00, 0xFF, 0xFF },
};

void selectiveBoxBlur(stbi_uc *out, const stbi_uc *in, int width, int height)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int startX = maxint(x - FILTER_SIZE, 0);
			int startY = maxint(y - FILTER_SIZE, 0);
			int endX = minint(x + FILTER_SIZE, width);
			int endY = minint(y + FILTER_SIZE, height);

			stbi_uc *rpx = out + (x + y * width) * 3;
			const stbi_uc *px = in + (x + y * width) * 3;
			stbi_uc r = px[0], g = px[1], b = px[2];

			unsigned rr = 0;
			unsigned rg = 0;
			unsigned rb = 0;
			unsigned rn = 0;

			for (int sx = startX; sx < endX; sx++)
			{
				for (int sy = startY; sy < endY; sy++)
				{
					const stbi_uc *spx = in + (sx + sy * width) * 3;
					stbi_uc sr = spx[0], sg = spx[1], sb = spx[2];

					int delta = abs((int)sr - (int)r) + abs((int)sg - (int)g) + abs((int)sb - (int)b);

					if (delta <= THRESHOLD)
					{
						rr += sr;
						rg += sg;
						rb += sb;
						rn++;
					}
				}
			}

			rpx[0] = rr / rn;
			rpx[1] = rg / rn;
			rpx[2] = rb / rn;
		}
	}
}

struct FloodFillPos
{
	uint16_t x, y;
};

inline FloodFillPos makeFloodFillPos(uint16_t x, uint16_t y)
{
	FloodFillPos pos;
	pos.x = x;
	pos.y = y;
	return pos;
}

void connectComponents(int *components, const stbi_uc *data, int width, int height, stbi_uc *debug)
{
	int blockWidth = (width + BLOCK_SIZE - 1) / BLOCK_SIZE;
	int blockHeight = (height + BLOCK_SIZE - 1) / BLOCK_SIZE;
	int *blocks = (int*)malloc(blockWidth * blockHeight * sizeof(int));
	stbi_uc *blockAvg = (stbi_uc*)malloc(blockWidth * blockHeight * 3);
	int *comps = (int*)calloc(blockWidth * blockHeight, sizeof(int));
	FloodFillPos *stack = (FloodFillPos*)malloc(width * height * sizeof(FloodFillPos));
	int *freeComponents = (int*)malloc(width * height * sizeof(int));

	int blockCounter = 0;
	int componentCounter = 0;

	// -- Collect pixels into NxN blocks and calculate the average, separate the blocks
	// into good and bad depending on if all the pixels are close to the average
	for (int y = 0; y < blockHeight; y++)
	{
		for (int x = 0; x < blockWidth; x++)
		{
			int startX = x * BLOCK_SIZE;
			int startY = y * BLOCK_SIZE;
			int endX = minint(startX + BLOCK_SIZE, width);
			int endY = minint(startY + BLOCK_SIZE, height);

			unsigned rr = 0;
			unsigned rg = 0;
			unsigned rb = 0;
			unsigned rn = 0;

			for (int sx = startX; sx < endX; sx++)
			{
				for (int sy = startY; sy < endY; sy++)
				{
					const stbi_uc *spx = data + (sx + sy * width) * 3;
					stbi_uc sr = spx[0], sg = spx[1], sb = spx[2];

					rr += sr;
					rg += sg;
					rb += sb;
					rn++;
				}
			}
		
			rr /= rn;
			rg /= rn;
			rb /= rn;

			int blockQuality = 0;
			for (int sx = startX; sx < endX; sx++)
			{
				for (int sy = startY; sy < endY; sy++)
				{
					const stbi_uc *spx = data + (sx + sy * width) * 3;
					stbi_uc sr = spx[0], sg = spx[1], sb = spx[2];

					unsigned delta = abs((int)sr - (int)rr) + abs((int)sg - (int)rg) + abs((int)sb - (int)rb);

					if (delta <= BLOCK_INNER_THRESHOLD)
						blockQuality++;
				}
			}

			int blockVal = 0;
			if (blockQuality >= BLOCK_QUALITY_THRESHOLD)
				blockVal = ++blockCounter;
			blocks[x + y * blockWidth] = blockVal;

			stbi_uc *rpx = blockAvg + (x + y * blockWidth) * 3;
			rpx[0] = rr;
			rpx[1] = rg;
			rpx[2] = rb;
		}
	}

	// -- Flood fill components through the good blocks
	for (int y = 0; y < blockHeight; y++)
	{
		for (int x = 0; x < blockWidth; x++)
		{
			int comp = comps[x + y * blockWidth];
			if (comp != 0)
				continue;

			int compIx = ++componentCounter;

			int stackPos = 0;
			stack[stackPos++] = makeFloodFillPos(x, y);

			while (stackPos > 0)
			{
				FloodFillPos pos = stack[--stackPos];

				if (blocks[pos.x + pos.y * blockWidth] == 0)
					continue;

				comps[pos.x + pos.y * blockWidth] = compIx;

				FloodFillPos neighbor[4];
				int neighborCount = 0;

				if (pos.x > 0 && comps[(pos.x - 1) + pos.y * blockWidth] == 0)
					neighbor[neighborCount++] = makeFloodFillPos(pos.x - 1, pos.y);
				if (pos.x + 1 < blockWidth && comps[(pos.x + 1) + pos.y * blockWidth] == 0)
					neighbor[neighborCount++] = makeFloodFillPos(pos.x + 1, pos.y);
				if (pos.y > 0 && comps[pos.x + (pos.y - 1) * blockWidth] == 0)
					neighbor[neighborCount++] = makeFloodFillPos(pos.x, pos.y - 1);
				if (pos.y + 1 < blockHeight && comps[pos.x + (pos.y + 1) * blockWidth] == 0)
					neighbor[neighborCount++] = makeFloodFillPos(pos.x, pos.y + 1);

				if (neighborCount == 0)
					continue;

				stbi_uc *avg = blockAvg + (pos.x + pos.y * blockWidth) * 3;
				stbi_uc ar = avg[0], ag = avg[1], ab = avg[2];

				for (int i = 0; i < neighborCount; i++)
				{
					FloodFillPos pos = neighbor[i];
					stbi_uc *savg = blockAvg + (pos.x + pos.y * blockWidth) * 3;
					stbi_uc sr = savg[0], sg = savg[1], sb = savg[2];

					unsigned delta = abs((int)sr - (int)ar) + abs((int)sg - (int)ag) + abs((int)sb - (int)ab);

					if (delta <= FLOOD_FILL_THRESHOLD)
					{
						stack[stackPos++] = pos;
					}
				}
			}
		}
	}

	// -- Eliminate components which don't have enough blocks

	int *componentCount = (int*)calloc(componentCounter + 1, sizeof(int));
	int *componentMapping = (int*)calloc(componentCounter + 1, sizeof(int));

	int blockCount = blockWidth * blockHeight;
	for (int i = 0; i < blockCount; i++)
		componentCount[comps[i]]++;

	int finalComponentCounter = 0;
	for (int i = 1; i < componentCounter; i++)
	{
		if (componentCount[i] >= MIN_COMPONENT_BLOCKS)
		{
			componentMapping[i] = ++finalComponentCounter;
		}
	}

	for (int i = 0; i < blockCount; i++)
	{
		comps[i] = componentMapping[comps[i]];
	}

	// -- Write the output
	// Good blocks (that survived the elimination) are written as-is with the block value
	// Bad blocks take the value from surrounding good blocks if possible

	for (int y = 0; y < blockHeight; y++)
	{
		for (int x = 0; x < blockWidth; x++)
		{
			int comp = comps[x + y * blockWidth];

			int startX = x * BLOCK_SIZE;
			int startY = y * BLOCK_SIZE;
			int endX = minint(startX + BLOCK_SIZE, width);
			int endY = minint(startY + BLOCK_SIZE, height);

			if (comp != 0)
			{
				for (int sx = startX; sx < endX; sx++)
				{
					for (int sy = startY; sy < endY; sy++)
					{
						freeComponents[sx + sy * width] = comp;
					}
				}
			}
			else
			{
				int nStartX = maxint(x - NEIGHBOR_RADIUS, 0);
				int nStartY = maxint(y - NEIGHBOR_RADIUS, 0);
				int nEndX = minint(x + NEIGHBOR_RADIUS, blockWidth);
				int nEndY = minint(y + NEIGHBOR_RADIUS, blockHeight);

				for (int sy = startY; sy < endY; sy++)
				{
					for (int sx = startX; sx < endX; sx++)
					{
						int closestComp = 0;
						unsigned closestDelta = NEIGHBOR_THRESHOLD + 1;

						const stbi_uc *spx = data + (sx + sy * width) * 3;
						stbi_uc sr = spx[0], sg = spx[1], sb = spx[2];

						for (int ny = nStartY; ny < nEndY; ny++)
						{
							for (int nx = nStartX; nx < nEndX; nx++)
							{
								int comp = comps[nx + ny * blockWidth];
								if (comp == 0)
									continue;

								stbi_uc *avg = blockAvg + (nx + ny * blockWidth) * 3;
								unsigned delta = abs((int)sr - (int)avg[0]) + abs((int)sg - (int)avg[1]) + abs((int)sb - (int)avg[2]);

								if (delta < closestDelta)
								{
									closestDelta = delta;
									closestComp = comp;
								}
							}
						}

						freeComponents[sx + sy * width] = closestComp;
					}
				}
			}
		}
	}

	// -- Do pixel level flood fill to force connected areas (slow!)

	unsigned char *componentState = (unsigned char*)calloc(finalComponentCounter + 1, 1);

	// Don't process the zero group
	componentState[0] = 1;

	memset(components, 0, sizeof(int) * width * height);

	for (int y = 0; y < blockHeight; y++)
	{
		for (int x = 0; x < blockWidth; x++)
		{
			int comp = comps[x + y * blockWidth];
			if (componentState[comp])
				continue;
			componentState[comp] = 1;

			int stackPos = 0;
			stack[stackPos++] = makeFloodFillPos(x * BLOCK_SIZE, y * BLOCK_SIZE);
			components[stack[0].x + stack[0].y * width] = comp;

			while (stackPos > 0)
			{
				FloodFillPos pos = stack[--stackPos];

				if (pos.x > 0 && components[(pos.x - 1) + pos.y * width] == 0 && freeComponents[(pos.x - 1) + pos.y * width] == comp)
				{
					components[(pos.x - 1) + pos.y * width] = comp;
					stack[stackPos++] = makeFloodFillPos(pos.x - 1, pos.y);
				}
				if (pos.x + 1 < width && components[(pos.x + 1) + pos.y * width] == 0 && freeComponents[(pos.x + 1) + pos.y * width] == comp)
				{
					components[(pos.x + 1) + pos.y * width] = comp;
					stack[stackPos++] = makeFloodFillPos(pos.x + 1, pos.y);
				}
				if (pos.y > 0 && components[pos.x + (pos.y - 1) * width] == 0 && freeComponents[pos.x + (pos.y - 1) * width] == comp)
				{
					components[pos.x + (pos.y - 1) * width] = comp;
					stack[stackPos++] = makeFloodFillPos(pos.x, pos.y - 1);
				}
				if (pos.y + 1 < height && components[pos.x + (pos.y + 1) * width] == 0 && freeComponents[pos.x + (pos.y + 1) * width] == comp)
				{
					components[pos.x + (pos.y + 1) * width] = comp;
					stack[stackPos++] = makeFloodFillPos(pos.x, pos.y + 1);
				}
			}
		}
	}

	if (debug)
	{
		for (int y = 0; y < blockHeight; y++)
		{
			for (int x = 0; x < blockWidth; x++)
			{
				int startX = x * BLOCK_SIZE;
				int startY = y * BLOCK_SIZE;
				int endX = minint(startX + BLOCK_SIZE, width);
				int endY = minint(startY + BLOCK_SIZE, height);

				int c = comps[x + y * blockWidth];

				stbi_uc *apx = g_debugColors[c % Count(g_debugColors)];
				stbi_uc ar = apx[0], ag = apx[1], ab = apx[2];

				if (c == 0)
				{
					ar = ag = ab = 0;
				}

				int block = blocks[x + y * blockWidth];

				for (int sx = startX; sx < endX; sx++)
				{
					for (int sy = startY; sy < endY; sy++)
					{
						stbi_uc *spx = debug + (sx + sy * width) * 3;

						spx[0] = ar;
						spx[1] = ag;
						spx[2] = ab;
					}
				}
			}
		}
	}

	free(blocks);
	free(blockAvg);
	free(comps);
	free(stack);
	free(freeComponents);
	free(componentCount);
	free(componentMapping);
	free(componentState);
}

struct EdgePoint
{
	uint16_t x, y;
};

EdgePoint makeEdgePoint(uint16_t x, uint16_t y)
{
	EdgePoint p;
	p.x = x;
	p.y = y;
	return p;
}

struct IVec2
{
	int x, y;
};

const IVec2 edgeAdvanceDirection[] = {
	{ 1, 0 },
	{ 0, 1 },
	{ -1, 0 },
	{ 0, -1 },
};

const IVec2 edgeSampleDirection[] = {
	{ 0, 0 },
	{ -1, 0 },
	{ -1, -1 },
	{ 0, -1 },
};

struct EdgeIx
{
	unsigned index;
	unsigned count;
};

struct EdgeList
{
	EdgePoint *points;
	EdgeIx *edges;
	unsigned edgeCount;
};

EdgeList findEdges(int *comps, int width, int height, int component)
{
	unsigned pointCount = 0;
	unsigned pointCapacity = 0;
	EdgePoint *points = NULL;

	unsigned edgeCount = 0;
	unsigned edgeCapacity = 0;
	EdgeIx *edgeIndices = NULL;

	stbi_uc *state = (stbi_uc*)calloc(width * height, 1);

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (comps[x + y * width] != component)
				continue;

			if (state[x + y * width] != 0)
				continue;

			if (y > 0 && comps[x + (y - 1) * width] == component)
				continue;

			if (edgeCount >= edgeCapacity)
			{
				edgeCapacity = maxint(4, edgeCapacity * 2);
				edgeIndices = (EdgeIx*)realloc(edgeIndices, sizeof(EdgeIx) * edgeCapacity);
			}

			EdgeIx *ix = edgeIndices + edgeCount;
			edgeCount++;

			ix->index = pointCount;

			int px = x;
			int py = y;

			unsigned dir = 0;

			do
			{
				IVec2 samplePos = edgeSampleDirection[dir];
				int spx = px + samplePos.x;
				int spy = py + samplePos.y;

				if (pointCount >= pointCapacity)
				{
					pointCapacity = maxint(32, pointCapacity * 2);
					points = (EdgePoint*)realloc(points, sizeof(EdgePoint) * pointCapacity);
				}

				points[pointCount++] = makeEdgePoint(px, py);

				if (spx >= 0 && spy >= 0 && spx < width && spy < height && comps[spx + spy * width] == component)
				{
					state[spx + spy * width] = 1;

					IVec2 sample2Pos = edgeSampleDirection[(dir - 1 + 4) % 4];
					int sp2x = px + sample2Pos.x;
					int sp2y = py + sample2Pos.y;

					if (sp2x >= 0 && sp2y >= 0 && sp2x < width && sp2y < height && comps[sp2x + sp2y * width] == component)
					{
						state[sp2x + sp2y * width] = 1;
						dir = (dir - 1 + 4) % 4;
					}
				}
				else
				{
					dir = (dir + 1) % 4;
				}

				IVec2 advance = edgeAdvanceDirection[dir];
				px += advance.x;
				py += advance.y;
			}
			while (px != x || py != y);

			ix->count = pointCount - ix->index;
		}
	}

	free(state);

	EdgeList list;
	list.points = points;
	list.edges = edgeIndices;
	list.edgeCount = edgeCount;
	return list;
}

struct Vec2
{
	float x, y;
};

Vec2 makeVec2(float x, float y)
{
	Vec2 v;
	v.x = x;
	v.y = y;
	return v;
}

Vec2 edgePointVec2(EdgePoint p)
{
	return makeVec2(p.x, p.y);
}

Vec2 operator+(const Vec2& a, const Vec2& b)
{
	return makeVec2(a.x + b.x, a.y + b.y);
}

Vec2 operator-(const Vec2& a, const Vec2& b)
{
	return makeVec2(a.x - b.x, a.y - b.y);
}

Vec2 operator*(const Vec2& a, float b)
{
	return makeVec2(a.x * b, a.y * b);
}

float dot(const Vec2& a, const Vec2& b)
{
	return a.x*b.x + a.y*b.y;
}

float lengthSq(const Vec2& a)
{
	return a.x*a.x + a.y*a.y;
}

float length(const Vec2& a)
{
	return sqrtf(a.x*a.x + a.y*a.y);
}

Vec2 lerp(const Vec2& a, const Vec2& b, float t)
{
	return a * (1.0f - t) + b * t;
}

float lerp(float a, float b, float t)
{
	return (1.0f - t) * a + t * b;
}

#define EPSILON 0.0001f

float lineDistanceSq(const Vec2& a, const Vec2& b, const Vec2& p)
{
	Vec2 dir = b - a;
	float distSquared = dot(dir, dir);
	if (distSquared >= -EPSILON && distSquared <= EPSILON)
	{
		return lengthSq(a - p);
	}
	else
	{
		float t = dot(dir, p - a) / distSquared;
		if (t < 0.0f) t = 0.0f;
		if (t > 1.0f) t = 1.0f;
		Vec2 q = a + dir * t;
		return lengthSq(p - q);
	}
}

unsigned simplifyEdgeRamerDouglasPeucker(EdgePoint *res, const EdgePoint *points, unsigned count, float epsilon)
{
	float maxDist = 0.0f;
	int maxIndex = 0;

	if (count > 1)
	{
		Vec2 a = edgePointVec2(points[0]);
		Vec2 b = edgePointVec2(points[count - 1]);

		for (unsigned i = 1; i < count - 1; i++)
		{
			float d = lineDistanceSq(a, b, edgePointVec2(points[i]));
			if (d > maxDist)
			{
				maxIndex = i;
				maxDist = d;
			}
		}
	}

	if (maxDist > epsilon)
	{
		EdgePoint *resPtr = res;
		resPtr += simplifyEdgeRamerDouglasPeucker(resPtr, points, maxIndex + 1, epsilon);
		resPtr--;
		resPtr += simplifyEdgeRamerDouglasPeucker(resPtr, points + maxIndex, count - maxIndex, epsilon);
		return resPtr - res;
	}
	else
	{
		res[0] = points[0];
		res[1] = points[count - 1];
		return 2;
	}
}

unsigned simplifyEdge(EdgePoint *res, const EdgePoint *points, unsigned count, float maxDist)
{
	EdgePoint *tempOut = (EdgePoint*)malloc(sizeof(EdgePoint) * (count + 1));
	EdgePoint *temp = (EdgePoint*)malloc(sizeof(EdgePoint) * (count + 1));
	memcpy(temp, points, count * sizeof(EdgePoint));
	temp[count] = points[0];

	unsigned tempCount = simplifyEdgeRamerDouglasPeucker(tempOut, temp, count + 1, maxDist * maxDist);

	// Align the start at one of the simplified points
	EdgePoint newStartPoint = tempOut[1];
	unsigned newStart;
	for (newStart = 0; newStart < count; newStart++)
	{
		if (points[newStart].x == newStartPoint.x && points[newStart].y == newStartPoint.y)
			break;
	}

	memcpy(temp, points + newStart, (count - newStart) * sizeof(EdgePoint));
	memcpy(temp + (count - newStart), points, newStart * sizeof(EdgePoint));
	temp[count] = temp[0];

	tempCount = simplifyEdgeRamerDouglasPeucker(tempOut, temp, count + 1, maxDist * maxDist);
	
	memcpy(res, tempOut, (tempCount - 1) * sizeof(EdgePoint));
	return tempCount - 1;
}

struct EdgeCorners
{
	EdgePoint topLeft;
	EdgePoint topRight;
	EdgePoint bottomLeft;
	EdgePoint bottomRight;
};

struct EdgePointList
{
	EdgePoint *points;
	unsigned count;
};

EdgeCorners findCorners(EdgePointList *edges, unsigned edgeCount)
{
	int minDiag1 = INT_MAX, maxDiag1 = INT_MIN;
	EdgePoint minPoint1, maxPoint1;
	int minDiag2 = INT_MAX, maxDiag2 = INT_MIN;
	EdgePoint minPoint2, maxPoint2;

	for (unsigned ei = 0; ei < edgeCount; ei++)
	{
		EdgePointList edge = edges[ei];
		for (unsigned i = 0; i < edge.count; i++)
		{
			EdgePoint p = edge.points[i];
			int diag1 = (int)p.x + (int)p.y;
			int diag2 = (int)p.x - (int)p.y;

			if (diag1 < minDiag1)
			{
				minDiag1 = diag1;
				minPoint1 = p;
			}
			if (diag1 > maxDiag1)
			{
				maxDiag1 = diag1;
				maxPoint1 = p;
			}
			if (diag2 < minDiag2)
			{
				minDiag2 = diag2;
				minPoint2 = p;
			}
			if (diag2 > maxDiag2)
			{
				maxDiag2 = diag2;
				maxPoint2 = p;
			}
		}
	}

	EdgeCorners corners;
	corners.topLeft = minPoint1;
	corners.topRight = maxPoint2;
	corners.bottomLeft = minPoint2;
	corners.bottomRight = maxPoint1;
	return corners;
}

struct Vec2Corners
{
	Vec2 topLeft;
	Vec2 topRight;
	Vec2 bottomLeft;
	Vec2 bottomRight;
};

void perspectiveTransform(stbi_uc *dst, int dstWidth, int dstHeight, const stbi_uc* src, int srcWidth, int srcHeight, Vec2Corners corners)
{
	for (int y = 0; y < dstHeight; y++)
	{
		float dy = (float)y / (float)dstHeight;
		Vec2 left = lerp(corners.topLeft, corners.bottomLeft, dy);
		Vec2 right = lerp(corners.topRight, corners.bottomRight, dy);

		for (int x = 0; x < dstWidth; x++)
		{
			float dx = (float)x / (float)dstWidth;
			Vec2 pos = lerp(left, right, dx);

			float fsx = floorf(pos.x);
			float fsy = floorf(pos.y);

			float lx = pos.x - fsx;
			float ly = pos.y - fsy;

			int sx = (int)fsx;
			int sy = (int)fsy;

			if (sx >= 0 && sy >= 0 && sx < srcWidth - 1 && sy < srcHeight - 1)
			{
				const stbi_uc *a = src + (sx + sy * srcWidth) * 3;
				const stbi_uc *b = src + ((sx + 1) + sy * srcWidth) * 3;
				const stbi_uc *c = src + (sx + (sy + 1) * srcWidth) * 3;
				const stbi_uc *d = src + ((sx + 1) + (sy + 1) * srcWidth) * 3;
				stbi_uc *out = dst + (x + y * dstWidth) * 3;

				for (int ch = 0; ch < 3; ch++)
				{
					float top = lerp((float)a[ch], (float)b[ch], lx);
					float bottom = lerp((float)c[ch], (float)d[ch], lx);
					float val = lerp(top, bottom, ly);
					out[ch] = (stbi_uc)val;
				}
			}
		}
	}
}

struct ObjectInfo
{
	int minX, minY;
	int maxX, maxY;

	int pixelCount;
	int index;

	stbi_uc color[3];
};

struct ObjectList
{
	ObjectInfo *objects;
	unsigned count;
};

bool objectPixelCompare(int *objectPixels, const int *components, const stbi_uc *data, unsigned index, const stbi_uc *pixel)
{
	if (objectPixels[index] != 0)
		return false;

	if (components[index] != 0)
		return false;

	const stbi_uc *spx = data + index * 3;
	stbi_uc rd = abs(spx[0] - pixel[0]);
	stbi_uc gd = abs(spx[1] - pixel[1]);
	stbi_uc bd = abs(spx[2] - pixel[2]);

	int delta = rd + gd + bd;
	return delta <= OBJECT_THRESHOLD;
}

int compareObjects(const void *av, const void *bv)
{
	int pxa = ((const ObjectInfo*)av)->pixelCount;
	int pxb = ((const ObjectInfo*)bv)->pixelCount;

	if (pxa > pxb)
		return 1;
	else if (pxa < pxb)
		return -1;
	else
		return 0;
}

ObjectList findObjects(int *objectPixels, const int *components, const stbi_uc *data, unsigned width, unsigned height)
{
	memset(objectPixels, 0, width * height * sizeof(int));
	FloodFillPos *stack = (FloodFillPos*)malloc(width * height * sizeof(FloodFillPos));

	int *preObjectPixels = (int*)calloc(width * height, sizeof(int));

	unsigned objectCount = 0;
	unsigned objectCapacity = 0;
	ObjectInfo *objects = NULL;
		
	int objectCounter = 0;

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int index = x + y * width;
			if (components[index] == 0 && preObjectPixels[index] == 0)
			{
				int objI = ++objectCounter;

				if (objectCount >= objectCapacity)
				{
					objectCapacity = maxint(16, objectCapacity * 2);
					objects = (ObjectInfo*)realloc(objects, objectCapacity * sizeof(ObjectInfo));
				}

				ObjectInfo *info = &objects[objectCount++];
				info->minX = INT_MAX;
				info->minY = INT_MAX;
				info->maxX = INT_MIN;
				info->maxY = INT_MIN;
				info->pixelCount = 0;
				info->index = objI;

				unsigned color[3] = { 0 };

				int stackPos = 0;
				stack[stackPos++] = makeFloodFillPos(x, y);
				preObjectPixels[stack[0].x + stack[0].y * width] = objI;

				while (stackPos > 0)
				{
					FloodFillPos pos = stack[--stackPos];

					info->minX = minint(info->minX, pos.x);
					info->minY = minint(info->minY, pos.y);
					info->maxX = maxint(info->maxX, pos.x + 1);
					info->maxY = maxint(info->maxY, pos.y + 1);
					info->pixelCount++;

					const stbi_uc *px = data + (pos.x + pos.y * width) * 3;
					color[0] += px[0];
					color[1] += px[1];
					color[2] += px[2];

					if (pos.x > 0 && objectPixelCompare(preObjectPixels, components, data, (pos.x - 1) + pos.y * width, px))
					{
						preObjectPixels[(pos.x - 1) + pos.y * width] = objI;
						stack[stackPos++] = makeFloodFillPos(pos.x - 1, pos.y);
					}
					if (pos.x + 1 < width && objectPixelCompare(preObjectPixels, components, data, (pos.x + 1) + pos.y * width, px))
					{
						preObjectPixels[(pos.x - 1) + pos.y * width] = objI;
						stack[stackPos++] = makeFloodFillPos(pos.x + 1, pos.y);
					}
					if (pos.y > 0 && objectPixelCompare(preObjectPixels, components, data, pos.x + (pos.y - 1) * width, px))
					{
						preObjectPixels[pos.x + (pos.y - 1) * width] = objI;
						stack[stackPos++] = makeFloodFillPos(pos.x, pos.y - 1);
					}
					if (pos.y + 1 < height && objectPixelCompare(preObjectPixels, components, data, pos.x + (pos.y + 1) * width, px))
					{
						preObjectPixels[pos.x + (pos.y + 1) * width] = objI;
						stack[stackPos++] = makeFloodFillPos(pos.x, pos.y + 1);
					}
				}

				info->color[0] = color[0] / info->pixelCount;
				info->color[1] = color[1] / info->pixelCount;
				info->color[2] = color[2] / info->pixelCount;
			}
		}
	}

	qsort(objects, objectCount, sizeof(ObjectInfo), compareObjects);

	for (unsigned objI = 0; objI < objectCount; objI++)
	{
		ObjectInfo *obj = &objects[objI];

		for (int y = obj->minY; y < obj->maxY; y++)
		{
			for (int x = obj->minX; x < obj->maxX; x++)
			{
				int objI = obj->index;
				if (preObjectPixels[x + y * width] != objI)
					continue;

				int stackPos = 0;
				stack[stackPos++] = makeFloodFillPos(x, y);
				objectPixels[stack[0].x + stack[0].y * width] = objI;

				obj->minX = INT_MAX;
				obj->minY = INT_MAX;
				obj->maxX = INT_MIN;
				obj->maxY = INT_MIN;
				obj->pixelCount = 0;

				while (stackPos > 0)
				{
					FloodFillPos pos = stack[--stackPos];

					obj->minX = minint(obj->minX, pos.x);
					obj->minY = minint(obj->minY, pos.y);
					obj->maxX = maxint(obj->maxX, pos.x + 1);
					obj->maxY = maxint(obj->maxY, pos.y + 1);
					obj->pixelCount++;

					FloodFillPos neighbor[4];
					int neighborCount = 0;

					if (pos.x > 0)
						neighbor[neighborCount++] = makeFloodFillPos(pos.x - 1, pos.y);
					if (pos.x + 1 < width)
						neighbor[neighborCount++] = makeFloodFillPos(pos.x + 1, pos.y);
					if (pos.y > 0)
						neighbor[neighborCount++] = makeFloodFillPos(pos.x, pos.y - 1);
					if (pos.y + 1 < height)
						neighbor[neighborCount++] = makeFloodFillPos(pos.x, pos.y + 1);

					for (int i = 0; i < neighborCount; i++)
					{
						FloodFillPos np = neighbor[i];
						int index = np.x + np.y * width;

						if (objectPixels[index] != 0)
							continue;

						if (preObjectPixels[index] != objI)
						{
							const stbi_uc *apx = obj->color;
							const stbi_uc *spx = data + index * 3;

							int delta = abs(apx[0] - spx[0]) + abs(apx[1] - spx[1]) + abs(apx[2] - spx[2]);
							if (delta > OBJECT_NEIGHBOR_THRESHOLD)
								continue;
						}

						objectPixels[index] = objI;
						stack[stackPos++] = np;
					}
				}

				goto break_pixel_search;
			}
		}
	break_pixel_search: {}
	}

	free(stack);

	ObjectList list;
	list.objects = objects;
	list.count = objectCount;
	return list;
}

void debugSvgPolygon(FILE *svg, EdgePoint *points, unsigned count, const char *style)
{
	if (style == NULL)
		style = "fill: none; stroke: black; stroke-width: 1";

	fprintf(svg, "<polygon style=\"%s\" points=\"", style);
	for (unsigned i = 0; i < count; i++)
	{
		fprintf(svg, "%d,%d", points[i].x, points[i].y);
		if (i + 1 < count)
			fputc(' ', svg);
	}
	fprintf(svg, "\" />\n");
}

struct VisionScene
{
	EdgeCorners screenCorners;
};

VisionScene *visionReadScene(Scene *scene, const ViewInfo *info)
{
	VisionScene *vision = (VisionScene*)malloc(sizeof(VisionScene));

	int *comps = (int*)malloc(info->viewWidth * info->viewHeight * sizeof(int));
	connectComponents(comps, (const stbi_uc*)info->viewData, info->viewWidth, info->viewHeight, 0);

	EdgePointList edges[16];
	unsigned numEdges = 0;

	// TODO
#if IPHONE
	int hackComps[] = { 2, 3 };
#else
	int hackComps[] = { 3, 4 };
#endif
	for (unsigned i = 0; i < Count(hackComps); i++)
	{
		EdgeList el = findEdges(comps, info->viewWidth, info->viewHeight, hackComps[i]);

		EdgeIx ix = el.edges[0];
		EdgePoint *points = el.points + ix.index;
		EdgePoint *simplePoints = (EdgePoint*)malloc(sizeof(EdgePoint) * ix.count);

		unsigned simpleCount = simplifyEdge(simplePoints, points, ix.count, 2.0f);
		free(el.points);
		free(el.edges);

		edges[numEdges].points = simplePoints;
		edges[numEdges].count = simpleCount;
		numEdges++;
	}

	free(comps);
	EdgeCorners corners = findCorners(edges, numEdges);

	for (unsigned i = 0; i < numEdges; i++)
	{
		free(edges[i].points);
	}

	vision->screenCorners = corners;

	Vec2Corners vecCorners;
	vecCorners.topLeft.x = (float)corners.topLeft.x;
	vecCorners.topLeft.y = (float)corners.topLeft.y;
	vecCorners.topRight.x = (float)corners.topRight.x;
	vecCorners.topRight.y = (float)corners.topRight.y;
	vecCorners.bottomLeft.x = (float)corners.bottomLeft.x;
	vecCorners.bottomLeft.y = (float)corners.bottomLeft.y;
	vecCorners.bottomRight.x = (float)corners.bottomRight.x;
	vecCorners.bottomRight.y = (float)corners.bottomRight.y;

	stbi_uc *perspectiveData = (stbi_uc*)calloc(info->screenWidth * info->screenHeight, 3);
	perspectiveTransform(
		perspectiveData, info->screenWidth, info->screenHeight,
		(const stbi_uc*)info->viewData, info->viewWidth, info->viewHeight,
		vecCorners
		);

	int *perspectiveComps = (int*)malloc(info->screenWidth * info->screenHeight * sizeof(int));
	connectComponents(perspectiveComps, perspectiveData, info->screenWidth, info->screenHeight, 0);

	int *objectPixels = (int*)malloc(info->screenWidth * info->screenHeight * sizeof(int));
	ObjectList objectList = findObjects(objectPixels, perspectiveComps, perspectiveData, info->screenWidth, info->screenHeight);

	EdgePointList perspectiveEdges[16];
	unsigned perspectiveNumEdges = 0;

	// TODO
	int perspectiveHackComps[] = { 1, 2 };
	for (unsigned i = 0; i < Count(perspectiveHackComps); i++)
	{
		EdgeList el = findEdges(perspectiveComps, info->screenWidth, info->screenHeight, perspectiveHackComps[i]);

		EdgeIx ix = el.edges[0];
		EdgePoint *points = el.points + ix.index;
		EdgePoint *simplePoints = (EdgePoint*)malloc(sizeof(EdgePoint) * ix.count);

		unsigned simpleCount = simplifyEdge(simplePoints, points, ix.count, 2.0f);
		free(el.points);
		free(el.edges);

		perspectiveEdges[perspectiveNumEdges].points = simplePoints;
		perspectiveEdges[perspectiveNumEdges].count = simpleCount;
		perspectiveNumEdges++;
	}

	free(objectPixels);
	free(perspectiveComps);
	free(perspectiveData);

	if (perspectiveNumEdges > 0)
	{
		scene->polys = (ScenePoly*)malloc(sizeof(ScenePoly) * (perspectiveNumEdges - 1));
		scene->numPolys = perspectiveNumEdges - 1;
		for (unsigned i = 0; i < perspectiveNumEdges - 1; i++)
		{
			EdgePointList *edge = &perspectiveEdges[i + 1];
			scene->polys[i].numVertices = edge->count;
			scene->polys[i].vertices = (SceneVec2*)malloc(sizeof(SceneVec2) * edge->count);
			for (unsigned j = 0; j < edge->count; j++)
			{
				scene->polys[i].vertices[j].x = (float)edge->points[j].x;
				scene->polys[i].vertices[j].y = (float)edge->points[j].y;
			}
		}
	}
	else
	{
		scene->numPolys = 0;
	}

	scene->screenWidth = info->screenWidth;
	scene->screenHeight = info->screenHeight;

	return vision;
}

#if 0
int main(int argc, char **argv)
{
	int width, height, channels;

#if IPHONE
	stbi_uc *data = stbi_load("test5.png", &width, &height, &channels, 3);
#else
	stbi_uc *data = stbi_load("test.png", &width, &height, &channels, 3);
#endif

#if IPHONE
	unsigned perspectiveWidth = 1334;
	unsigned perspectiveHeight = 750;
#else
	unsigned perspectiveWidth = 1920;
	unsigned perspectiveHeight = 1080;

#endif

	stbi_uc *resultData = (stbi_uc*)calloc(width * height, 3);
	stbi_uc *perspectiveResultData = (stbi_uc*)calloc(perspectiveWidth * perspectiveHeight, 3);
	stbi_uc *debugData = (stbi_uc*)calloc(width * height, 3);
	stbi_uc *perspectiveData = (stbi_uc*)calloc(perspectiveWidth * perspectiveHeight, 3);

	int *comps = (int*)malloc(width * height * sizeof(int));

	// selectiveBoxBlur(resultData, data, width, height);
	connectComponents(comps, data, width, height, 0);

	EdgePointList edges[16];
	unsigned numEdges = 0;

	// TODO
#if IPHONE
	int hackComps[] = { 2, 3 };
#else
	int hackComps[] = { 3, 4 };
#endif
	for (unsigned i = 0; i < Count(hackComps); i++)
	{
		EdgeList el = findEdges(comps, width, height, hackComps[i]);

		EdgeIx ix = el.edges[0];
		EdgePoint *points = el.points + ix.index;
		EdgePoint *simplePoints = (EdgePoint*)malloc(sizeof(EdgePoint) * ix.count);

		unsigned simpleCount = simplifyEdge(simplePoints, points, ix.count, 2.0f);

		edges[numEdges].points = simplePoints;
		edges[numEdges].count = simpleCount;
		numEdges++;
	}

	EdgeCorners corners = findCorners(edges, numEdges);

	Vec2Corners vecCorners;
	vecCorners.topLeft.x = (float)corners.topLeft.x;
	vecCorners.topLeft.y = (float)corners.topLeft.y;
	vecCorners.topRight.x = (float)corners.topRight.x;
	vecCorners.topRight.y = (float)corners.topRight.y;
	vecCorners.bottomLeft.x = (float)corners.bottomLeft.x;
	vecCorners.bottomLeft.y = (float)corners.bottomLeft.y;
	vecCorners.bottomRight.x = (float)corners.bottomRight.x;
	vecCorners.bottomRight.y = (float)corners.bottomRight.y;

	perspectiveTransform(
		perspectiveData, perspectiveWidth, perspectiveHeight,
		data, width, height,
		vecCorners
		);

	int *perspectiveComps = (int*)malloc(perspectiveWidth * perspectiveHeight * sizeof(int));
	connectComponents(perspectiveComps, perspectiveData, perspectiveWidth, perspectiveHeight, 0);

	int *objectPixels = (int*)malloc(perspectiveWidth * perspectiveHeight * sizeof(int));
	ObjectList objectList = findObjects(objectPixels, perspectiveComps, perspectiveData, perspectiveWidth, perspectiveHeight);

	EdgePointList perspectiveEdges[16];
	unsigned perspectiveNumEdges = 0;

	// TODO
	int perspectiveHackComps[] = { 1, 2 };
	for (unsigned i = 0; i < Count(perspectiveHackComps); i++)
	{
		EdgeList el = findEdges(perspectiveComps, perspectiveWidth, perspectiveHeight, perspectiveHackComps[i]);

		EdgeIx ix = el.edges[0];
		EdgePoint *points = el.points + ix.index;
		EdgePoint *simplePoints = (EdgePoint*)malloc(sizeof(EdgePoint) * ix.count);

		unsigned simpleCount = simplifyEdge(simplePoints, points, ix.count, 2.0f);

		perspectiveEdges[perspectiveNumEdges].points = simplePoints;
		perspectiveEdges[perspectiveNumEdges].count = simpleCount;
		perspectiveNumEdges++;
	}

	{
		FILE *svg = fopen("debug.svg", "wb");

		fprintf(svg, "<?xml version=\"1.0\"?>\n");
		fprintf(svg, "<svg width=\"%d\" height=\"%d\" viewPort=\"0 0 %d %d\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\">\n", width, height, width, height);

		{
			EdgePoint crn[] = {
				corners.topLeft,
				corners.topRight,
				corners.bottomRight,
				corners.bottomLeft,
			};

			debugSvgPolygon(svg, crn, Count(crn), "fill: #333;");
		}

		for (unsigned ei = 0; ei < numEdges; ei++)
		{
			debugSvgPolygon(svg, edges[ei].points, edges[ei].count, NULL);
		}
		fprintf(svg, "</svg>");
	}

	{
		FILE *svg = fopen("pedebug.svg", "wb");

		fprintf(svg, "<?xml version=\"1.0\"?>\n");
		fprintf(svg, "<svg width=\"%d\" height=\"%d\" viewPort=\"0 0 %d %d\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\">\n", perspectiveWidth, perspectiveHeight, perspectiveWidth, perspectiveHeight);
		fprintf(svg, "<rect width=\"%d\" height=\"%d\" style=\"%s\" />\n", perspectiveWidth, perspectiveHeight, "fill: #d6ad71;");

		for (unsigned ei = 1; ei < perspectiveNumEdges; ei++)
		{
			debugSvgPolygon(svg, perspectiveEdges[ei].points, perspectiveEdges[ei].count, "fill: #d68a44;");
		}

		for (unsigned oi = 0; oi < objectList.count; oi++)
		{
			ObjectInfo *obj = &objectList.objects[oi];

			if (obj->pixelCount < 40 || obj->pixelCount > 1000)
				continue;

			fprintf(svg, "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" style=\"fill: #%02x%02x%02x; stroke-width: 0;\" />\n",
				obj->minX,
				obj->minY,
				obj->maxX - obj->minX,
				obj->maxY - obj->minY,
				obj->color[0],
				obj->color[1],
				obj->color[2]);
		}

		fprintf(svg, "</svg>");

		fclose(svg);
	}

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int c = comps[x + y * width];
			if (c == 0)
				continue;

			stbi_uc *dpx = g_debugColors[c % Count(g_debugColors)];
			stbi_uc *rpx = resultData + (x + y * width) * 3;
			rpx[0] = dpx[0];
			rpx[1] = dpx[1];
			rpx[2] = dpx[2];
		}
	}

	for (int y = 0; y < perspectiveHeight; y++)
	{
		for (int x = 0; x < perspectiveWidth; x++)
		{
			int c = perspectiveComps[x + y * perspectiveWidth];
			if (c == 0)
				continue;

			stbi_uc *dpx = g_debugColors[c % Count(g_debugColors)];
			stbi_uc *rpx = perspectiveResultData + (x + y * perspectiveWidth) * 3;
			rpx[0] = dpx[0];
			rpx[1] = dpx[1];
			rpx[2] = dpx[2];
		}
	}

	stbi_write_png("debug.png", width, height, 3, debugData, 0);
	stbi_write_png("result.png", width, height, 3, resultData, 0);
	stbi_write_png("perspective.png", perspectiveWidth, perspectiveHeight, 3, perspectiveData, 0);
	stbi_write_png("peresult.png", perspectiveWidth, perspectiveHeight, 3, perspectiveResultData, 0);

	return 0;
}
#endif
