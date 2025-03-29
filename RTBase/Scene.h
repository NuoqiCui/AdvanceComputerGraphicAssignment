#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"

class Camera
{
public:
	Matrix inverseProjectionMatrix;
	Matrix camera;
	float width = 0;
	float height = 0;
	Vec3 origin;
	void init(Matrix ProjectionMatrix, int screenwidth, int screenheight)
	{
		inverseProjectionMatrix = ProjectionMatrix.invert();
		width = (float)screenwidth;
		height = (float)screenheight;
	}
	void updateView(Matrix V)
	{
		camera = V;
		origin = camera.mulPoint(Vec3(0, 0, 0));
	}
	// Add code here
	Ray generateRay(float x, float y)
	{
		float xprime = x / width;
		float yprime = 1.0f - (y / height);
		xprime = (xprime * 2.0f) - 1.0f;
		yprime = (yprime * 2.0f) - 1.0f;
		Vec3 dir(xprime, yprime, 1.0f);
		dir = inverseProjectionMatrix.mulPoint(dir);
		dir = camera.mulVec(dir);
		dir = dir.normalize();
		return Ray(origin, dir);
	}
};

struct VPL {
	Vec3 position;
	Vec3 normal;
	Colour emission;
	float pdf;
};

ShadingData createDummyShadingData(const Vec3& pos, const Vec3& normal)
{
	ShadingData sd;
	sd.x = pos;
	sd.sNormal = normal;
	sd.gNormal = normal;
	sd.tu = 0.5f;
	sd.tv = 0.5f;
	sd.wo = -normal;
	sd.frame.fromVector(normal);
	sd.t = 0.0f;
	sd.bsdf = nullptr;
	return sd;
}

class Scene
{
public:
	std::vector<Triangle> triangles;
	std::vector<BSDF*> materials;
	std::vector<Light*> lights;
	Light* background = NULL;
	BVHNode* bvh = NULL;
	Camera camera;
	AABB bounds;
	std::vector<VPL> vplList;
	void build()
	{
		// Add BVH building code here
		std::vector<Triangle> inputTriangles;
		for (int i = 0; i < triangles.size(); i++)
		{
			inputTriangles.push_back(triangles[i]);
		}
		triangles.clear();
		bvh = new BVHNode();
		bvh->build(inputTriangles, triangles);


		// Do not touch the code below this line!
		// Build light list
		for (int i = 0; i < triangles.size(); i++)
		{
			if (materials[triangles[i].materialIndex]->isLight())
			{
				AreaLight* light = new AreaLight();
				light->triangle = &triangles[i];
				light->emission = materials[triangles[i].materialIndex]->emission;
				lights.push_back(light);
			}
		}
	}

	void computeVPLs(int numPhotons, Sampler* sampler)
	{
		vplList.clear();
		int numLights = lights.size();
		if (numLights == 0)
			return;
		int photonsPerLight = numPhotons / numLights;

		Vec3 center = (bounds.min + bounds.max) * 0.5f;
		Vec3 defaultNormal(0.0f, 1.0f, 0.0f);
		ShadingData dummySD = createDummyShadingData(center, defaultNormal);

		for (int i = 0; i < numLights; i++)
		{
			Light* light = lights[i];
			if (!light->isArea())
				continue;

			for (int j = 0; j < photonsPerLight; j++)
			{
				Colour emitted;
				float pdf;
				Vec3 p = light->sample(dummySD, sampler, emitted, pdf);
				if (pdf < 1e-6f)
					continue;
				float r1 = sampler->next();
				float r2 = sampler->next();
				Vec3 localDir = SamplingDistributions::cosineSampleHemisphere(r1, r2);
				Frame frame;
				frame.fromVector(defaultNormal);
				Vec3 photonDir = frame.toWorld(localDir);
				Ray photonRay(p, photonDir);
				IntersectionData its = traverse(photonRay);
				if (its.t < FLT_MAX)
				{
					ShadingData sd = calculateShadingData(its, photonRay);
					if (!sd.bsdf->isPureSpecular())
					{
						VPL vpl;
						vpl.position = photonRay.at(its.t);
						vpl.normal = sd.sNormal;
						float dist2 = its.t * its.t;
						vpl.emission = emitted / (pdf * dist2);
						vpl.pdf = 1.0f / photonsPerLight;
						vplList.push_back(vpl);
					}
				}
			}
		}
	}


	IntersectionData traverse(const Ray& ray)
	{
		return bvh->traverse(ray, triangles);
	}

	Light* sampleLight(Sampler* sampler, float& pmf)
	{
		float r1 = sampler->next();
		pmf = 1.0f / (float)lights.size();
		return lights[std::min((int)(r1 * lights.size()), (int)(lights.size() - 1))];
	}
	// Do not modify any code below this line
	void init(std::vector<Triangle> meshTriangles, std::vector<BSDF*> meshMaterials, Light* _background)
	{
		for (int i = 0; i < meshTriangles.size(); i++)
		{
			triangles.push_back(meshTriangles[i]);
			bounds.extend(meshTriangles[i].vertices[0].p);
			bounds.extend(meshTriangles[i].vertices[1].p);
			bounds.extend(meshTriangles[i].vertices[2].p);
		}
		for (int i = 0; i < meshMaterials.size(); i++)
		{
			materials.push_back(meshMaterials[i]);
		}
		background = _background;
		if (background->totalIntegratedPower() > 0)
		{
			lights.push_back(background);
		}
	}

	bool visible(const Vec3& p1, const Vec3& p2)
	{
		Ray ray;
		Vec3 dir = p2 - p1;
		float maxT = dir.length() - (2.0f * 0.001f);
		dir = dir.normalize();
		ray.init(p1 + (dir * 0.001f), dir);
		return bvh->traverseVisible(ray, triangles, maxT);
	}


	Colour emit(Triangle* light, ShadingData shadingData, Vec3 wi)
	{
		return materials[light->materialIndex]->emit(shadingData, wi);
	}
	ShadingData calculateShadingData(IntersectionData intersection, Ray& ray)
	{
		ShadingData shadingData = {};
		if (intersection.t < FLT_MAX)
		{
			shadingData.x = ray.at(intersection.t);
			shadingData.gNormal = triangles[intersection.ID].gNormal();
			triangles[intersection.ID].interpolateAttributes(intersection.alpha, intersection.beta, intersection.gamma, shadingData.sNormal, shadingData.tu, shadingData.tv);
			shadingData.bsdf = materials[triangles[intersection.ID].materialIndex];
			shadingData.wo = -ray.dir;
			if (shadingData.bsdf->isTwoSided())
			{
				if (Dot(shadingData.wo, shadingData.sNormal) < 0)
				{
					shadingData.sNormal = -shadingData.sNormal;
				}
				if (Dot(shadingData.wo, shadingData.gNormal) < 0)
				{
					shadingData.gNormal = -shadingData.gNormal;
				}
			}
			shadingData.frame.fromVector(shadingData.sNormal);
			shadingData.t = intersection.t;
		}
		else
		{
			shadingData.wo = -ray.dir;
			shadingData.t = intersection.t;
		}
		return shadingData;
	}
};