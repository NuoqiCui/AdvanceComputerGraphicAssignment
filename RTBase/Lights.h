#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"
#define SQ(x) ((x) * (x))

#pragma warning( disable : 4244)

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) override
	{
		if (!triangle) {
			pdf = 0.0f;
			return Vec3(0.0f, 0.0f, 0.0f);
		}
		return triangle->sample(sampler, pdf);
	}

	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) override
	{
		Vec3 n = triangle->gNormal();
		Frame lightFrame;
		lightFrame.fromVector(n);

		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 localDir = SamplingDistributions::cosineSampleHemisphere(r1, r2);
		pdf = localDir.z / M_PI;

		return lightFrame.toWorld(localDir);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) override
	{
		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 dir = SamplingDistributions::uniformSampleSphere(r1, r2);
		pdf = 1.0f / (4.0f * M_PI);

		const float farDistance = 1e5f;
		return dir * farDistance;
	}

	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) override
	{
		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 wi = SamplingDistributions::uniformSampleSphere(r1, r2);
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};


struct SceneBounds {
	float sceneRadius;
	Vec3 sceneCentre;
};

static SceneBounds g_sceneBounds = { 1.0f, Vec3(0.0f, 0.0f, 0.0f) };

class EnvironmentMap : public Light
{
public:
	Texture* env;
	std::vector<float> rowLuminance;
	std::vector<float> rowCDF;
	std::vector<float> colCDF;
	float totalSum;

	EnvironmentMap(Texture* _env)
	{
		env = _env;
		buildEnvCDF();
	}

	void buildEnvCDF()
	{
		int w = env->width;
		int h = env->height;

		rowLuminance.resize(h, 0.0f);
		rowCDF.resize(h, 0.0f);
		colCDF.resize(h * w, 0.0f);

		for (int y = 0; y < h; y++)
		{
			float theta = (float(y) + 0.5f) / float(h) * M_PI;
			float sinTheta = sinf(theta);

			float rowSum = 0.0f;
			for (int x = 0; x < w; x++)
			{
				Colour c = env->texels[y * w + x];
				float lum = c.Lum() * sinTheta;
				rowSum += lum;
				colCDF[y * w + x] = rowSum;
			}
			rowLuminance[y] = rowSum;
		}

		float prefix = 0.0f;
		for (int y = 0; y < h; y++)
		{
			prefix += rowLuminance[y];
			rowCDF[y] = prefix;
		}
		totalSum = prefix;
	}

	int binarySearchRow(float rv) {
		int low = 0, high = (int)rowCDF.size() - 1;
		while (low < high) {
			int mid = (low + high) >> 1;
			if (rowCDF[mid] < rv) low = mid + 1;
			else high = mid;
		}
		return low;
	}

	int binarySearchCol(float val, int left, int right)
	{
		while (left < right) {
			int mid = (left + right) >> 1;
			if (colCDF[mid] < val) left = mid + 1;
			else right = mid;
		}
		return left;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		float rv = sampler->next() * totalSum;
		int row = binarySearchRow(rv);
		float rowCDF0 = (row == 0) ? 0.0f : rowCDF[row - 1];
		float rowSegment = rv - rowCDF0;

		int w = env->width;
		int h = env->height;
		float rowSum = rowLuminance[row];
		int base = row * w;
		int col = binarySearchCol(rowSegment, base, base + w - 1);

		float u = (float(col) + 0.5f) / float(w);
		float v = (float(row) + 0.5f) / float(h);

		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;
		float sinTheta = sinf(theta);

		Vec3 wi(
			sinTheta * cosf(phi),
			cosf(theta),
			sinTheta * sinf(phi)
		);


		Colour tex = env->sample(u, v);
		float lum = tex.Lum() * sinTheta;
		float discretePdf = lum / (totalSum + 1e-6f);

		pdf = discretePdf * (float(w) * float(h));

		reflectedColour = tex;
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		float theta = acosf(std::max(-1.0f, std::min(1.0f, wi.y)));
		float phi = atan2f(wi.z, wi.x);
		if (phi < 0.0f) phi += 2.0f * M_PI;

		float u = phi / (2.0f * M_PI);
		float v = theta / M_PI;
		Colour tex = env->sample(u, v);
		float sinTheta = sinf(theta);
		float lum = tex.Lum() * sinTheta;

		float pdf = (lum / (totalSum + 1e-6f)) * (float)(env->width * env->height);

		return pdf;
	}

	bool isArea()
	{
		return false;
	}

	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}

	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}

	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * g_sceneBounds.sceneRadius;
		p = p + g_sceneBounds.sceneCentre;
		pdf = 1.0f / (4.0f * M_PI * SQ(g_sceneBounds.sceneRadius));
		return p;
	}

	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};
