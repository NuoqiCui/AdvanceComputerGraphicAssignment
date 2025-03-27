#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <OpenImageDenoise/oidn.hpp>

#define MAX_DEPTH 5
#define EPSILON   1e-4f
#define HUGE_DIST 1e8f

inline float powerHeuristic(float pdfA, float pdfB)
{
	return (pdfA) / (pdfA + pdfB);
}


class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	int numProcs;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;

		film = new Film();
		film->init(
			(unsigned int)scene->camera.width,
			(unsigned int)scene->camera.height,
			new BoxFilter()
		);

		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		samplers = new MTRandom[numProcs];

		clear();
	}

	void clear()
	{
		film->clear();
	}

	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}



	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		// Sample a point on the light
		float pdf;
		Colour emitted;
		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);
		if (light->isArea())
		{
			// Calculate GTerm
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / (l);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		else
		{
			// Calculate GTerm
			Vec3 wi = p;
			float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}


	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true)
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			if (depth > MAX_DEPTH)
			{
				return direct;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}
			Colour bsdf;
			float pdf;
			//Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			Colour bsdfVal;
			float pdfVal;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdfVal, pdfVal);
			pdf = pdfVal;
			bsdf = bsdfVal;
			/*pdf = SamplingDistributions::cosineHemispherePDF(wi);
			wi = shadingData.frame.toWorld(wi);
			bsdf = shadingData.bsdf->evaluate(shadingData, wi);*/
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			r.init(shadingData.x + (wi * EPSILON), wi);
			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}
		return scene->background->evaluate(shadingData, r.dir);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}


	//void denoise(float* denoised)
	//{
	//	int width = film->width;
	//	int height = film->height;

	//	std::vector<float> inputBuffer(width * height * 3);
	//	for (int i = 0; i < width * height; i++)
	//	{
	//		inputBuffer[i * 3 + 0] = film->film[i].r;
	//		inputBuffer[i * 3 + 1] = film->film[i].g;
	//		inputBuffer[i * 3 + 2] = film->film[i].b;
	//	}

	//	std::vector<float> outputBuffer(width * height * 3);

	//	oidn::DeviceRef device = oidn::newDevice();
	//	device.commit();

	//	oidn::FilterRef filter = device.newFilter("RT");
	//	filter.set("hdr", true);

	//	filter.setImage("color", inputBuffer.data(), oidn::Format::Float3, width, height);
	//	filter.setImage("output", outputBuffer.data(), oidn::Format::Float3, width, height);

	//	memccpy(denoised, outputBuffer.data(), width * height * 3);
	//	filter.commit();
	//	filter.execute();
	//}

	//void denoise(float* color, float* albedo, float* normal, float* output, int width, int height)
	//{
	//	std::cerr << "albedo pointer = " << static_cast<void*>(albedo) << std::endl;
	//	try {
	//		oidn::DeviceRef device = oidn::newDevice();
	//		device.commit();

	//		const size_t size = width * height * 3 * sizeof(float); // float3 buffer

	//		oidn::BufferRef colorBuf = device.newBuffer(size);
	//		oidn::BufferRef albedoBuf = albedo ? device.newBuffer(size) : nullptr;
	//		if (albedo) {
	//			albedoBuf = device.newBuffer(size);
	//			std::cerr << "albedoBuf.getData() = " << albedoBuf.getData() << std::endl;
	//			if (!albedoBuf.getData()) {
	//				std::cerr << "Failed to create albedo buffer." << std::endl;
	//			}
	//		}
	//		oidn::BufferRef normalBuf = normal ? device.newBuffer(size) : nullptr;
	//		oidn::BufferRef outputBuf = device.newBuffer(size);

	//		std::memcpy(colorBuf.getData(), color, size);
	//		if (albedo && albedoBuf) std::memcpy(albedoBuf.getData(), albedo, size);
	//		if (normal && normalBuf) std::memcpy(normalBuf.getData(), normal, size);

	//		oidn::FilterRef filter = device.newFilter("RT");
	//		filter.setImage("color", colorBuf, oidn::Format::Float3, width, height);
	//		if (albedo && albedoBuf) filter.setImage("albedo", albedoBuf, oidn::Format::Float3, width, height);
	//		if (normal && normalBuf) filter.setImage("normal", normalBuf, oidn::Format::Float3, width, height);
	//		filter.setImage("output", outputBuf, oidn::Format::Float3, width, height);
	//		filter.set("hdr", true);

	//		filter.commit();
	//		filter.execute();

	//		// copy data from outputBuf
	//		std::memcpy(output, outputBuf.getData(), size);

	//		const char* message;
	//		if (device.getError(message) != oidn::Error::None) {
	//			std::cerr << "[OIDN Error] " << message << std::endl;
	//		}
	//	}
	//	catch (const std::exception& e) {
	//		std::cerr << "OIDN Exception: " << e.what() << std::endl;
	//	}
	//}

	void denoise(float* color, float* output, int width, int height)
	{
		try {
			oidn::DeviceRef device = oidn::newDevice();
			device.commit();

			const size_t size = width * height * 3 * sizeof(float); // float3 buffer

			oidn::BufferRef colorBuf = device.newBuffer(size);
			oidn::BufferRef outputBuf = device.newBuffer(size);

			std::memcpy(colorBuf.getData(), color, size);

			oidn::FilterRef filter = device.newFilter("RT");
			filter.setImage("color", colorBuf, oidn::Format::Float3, width, height);
			filter.setImage("output", outputBuf, oidn::Format::Float3, width, height);
			filter.set("hdr", true);

			filter.commit();
			filter.execute();

			// copy data from outputBuf
			std::memcpy(output, outputBuf.getData(), size);

			const char* message;
			if (device.getError(message) != oidn::Error::None) {
				std::cerr << "[OIDN Error] " << message << std::endl;
			}
		}
		catch (const std::exception& e) {
			std::cerr << "OIDN Exception: " << e.what() << std::endl;
		}
	}

	void render()
	{
		static const int TILE_SIZE = 32;
		film->incrementSPP();

		int numThreads = numProcs;
		std::vector<std::thread> workers;
		workers.reserve(numThreads);

		int numTilesX = (film->width + TILE_SIZE - 1) / TILE_SIZE;
		int numTilesY = (film->height + TILE_SIZE - 1) / TILE_SIZE;

		auto renderTile = [&](int tileX, int tileY, int threadId)
			{
				int startX = tileX * TILE_SIZE;
				int startY = tileY * TILE_SIZE;
				int endX = min(startX + TILE_SIZE, (int)film->width);
				int endY = min(startY + TILE_SIZE, (int)film->height);

				Sampler* localSampler = &samplers[threadId];

				for (int y = startY; y < endY; y++)
				{
					for (int x = startX; x < endX; x++)
					{
						float px = x + 0.5f;
						float py = y + 0.5f;

						Ray ray = scene->camera.generateRay(px, py);

						Colour pathThroughput(1.0f, 1.0f, 1.0f);
						Colour col = pathTrace(ray, pathThroughput, 3, localSampler);

						film->splat(px, py, col);

						unsigned char r = (unsigned char)(min(col.r, 1.0f) * 255);
						unsigned char g = (unsigned char)(min(col.g, 1.0f) * 255);
						unsigned char b = (unsigned char)(min(col.b, 1.0f) * 255);

						film->tonemap(x, y, r, g, b);
						canvas->draw(x, y, r, g, b);
					}
				}
			};

		auto workerFunc = [&](int threadId)
			{
				for (int tileY = 0; tileY < numTilesY; tileY++)
				{
					for (int tileX = 0; tileX < numTilesX; tileX++)
					{
						if (((tileY * numTilesX) + tileX) % numThreads == threadId)
						{
							renderTile(tileX, tileY, threadId);
						}
					}
				}
			};

		for (int i = 0; i < numThreads; i++)
		{
			workers.emplace_back(workerFunc, i);
		}
		for (auto& w : workers) {
			w.join();
		}
		savePNG("output.png");
		float* denoised = new float[film->width * film->height * 3];

		for (int i = 0; i < 10; i++) {
			std::cerr << "film->albedo[" << i << "] = "
				<< film->albedo[i].r << ", "
				<< film->albedo[i].g << ", "
				<< film->albedo[i].b << std::endl;
		}


		//denoise(reinterpret_cast<float*>(film->film),

		//	reinterpret_cast<float*>(film->albedo),

		//	reinterpret_cast<float*>(film->normal),

		//	denoised,

		//	film->width, film->height);

		denoise(reinterpret_cast<float*>(film->film),

			denoised,

			film->width, film->height);

		stbi_write_hdr("raw.hdr", film->width, film->height, 3, (float*)film->film);

		stbi_write_hdr("denoised.hdr", film->width, film->height, 3, denoised);
	}


	int getSPP()
	{
		return film->SPP;
	}

	void saveHDR(std::string filename)
	{
		film->save(filename);
	}

	void savePNG(std::string filename)
	{
		stbi_write_png(
			filename.c_str(),
			canvas->getWidth(),
			canvas->getHeight(),
			3,
			canvas->getBackBuffer(),
			canvas->getWidth() * 3
		);
	}

	void saveHDR(std::string filename, Film* film)
	{
		int width = film->width;
		int height = film->height;

	}



};