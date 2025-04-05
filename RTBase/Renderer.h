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

#define MAX_DEPTH 10
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
	std::vector<VPL> vplVector;

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

	Colour getAlbedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			if (ConductorBSDF* conductor = dynamic_cast<ConductorBSDF*>(shadingData.bsdf))
			{
				return conductor->albedo->sample(shadingData.tu, shadingData.tv);
			}
			return Colour(1.0f, 1.0f, 1.0f);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}


	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular())
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Colour L_direct(0.0f, 0.0f, 0.0f);


		{
			float lightPmf;
			Light* light = scene->sampleLight(sampler, lightPmf);

			float lightPdf;
			Colour emitted;
			Vec3 p = light->sample(shadingData, sampler, emitted, lightPdf);
			float pdf_light = lightPmf * lightPdf;

			Vec3 wi;
			float GTerm = 0.0f;
			bool visible = false;
			if (light->isArea())
			{
				wi = p - shadingData.x;
				float dist2 = wi.lengthSq();
				wi = wi.normalize();
				GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) *
					max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / dist2;
				visible = (GTerm > 0.0f && scene->visible(shadingData.x, p));
			}
			else
			{
				wi = p;
				GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
				visible = (GTerm > 0.0f && scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)));
			}

			if (visible && pdf_light > 1e-6f)
			{
				Colour f = shadingData.bsdf->evaluate(shadingData, wi);
				float pdf_bsdf = shadingData.bsdf->PDF(shadingData, wi);
				float weight_light = pdf_light / (pdf_light + pdf_bsdf + 1e-5f);
				L_direct = L_direct + f * emitted * GTerm * weight_light / (pdf_light + 1e-5f);
			}
		}

		{
			Colour bsdfVal;
			float pdf_bsdf_sample;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdfVal, pdf_bsdf_sample);
			if (pdf_bsdf_sample > 1e-6f)
			{
				Ray shadowRay;
				shadowRay.init(shadingData.x + wi * EPSILON, wi);
				IntersectionData lightIntersection = scene->traverse(shadowRay);
				ShadingData lightShadingData = scene->calculateShadingData(lightIntersection, shadowRay);
				if (lightShadingData.t < FLT_MAX && lightShadingData.bsdf && lightShadingData.bsdf->isLight())
				{
					Colour emitted_bsdf = lightShadingData.bsdf->emit(lightShadingData, -wi);
					float GTerm_bsdf = max(Dot(wi, shadingData.sNormal), 0.0f);
					float pdf_light_from_bsdf = 1.0f / (float)scene->lights.size();
					float weight_bsdf = pdf_bsdf_sample / (pdf_bsdf_sample + pdf_light_from_bsdf + 1e-5f);
					L_direct = L_direct + bsdfVal * emitted_bsdf * GTerm_bsdf * weight_bsdf / (pdf_bsdf_sample + 1e-5f);
				}
			}
		}

		return L_direct;
	}




	Colour computeDirectFromVPLs(const ShadingData& shadingData)
	{
		Colour L(0.0f, 0.0f, 0.0f);
		int numVPLs = scene->vplList.size();
		if (numVPLs == 0)
			return L;

		for (const auto& vpl : scene->vplList)
		{
			Vec3 wi = vpl.position - shadingData.x;
			float distSq = max(wi.lengthSq(), 0.01f);
			wi = wi.normalize();

			float G = (max(Dot(wi, shadingData.sNormal), 0.0f) *
				max(Dot(-wi, vpl.normal), 0.0f)) / distSq;
			if (G <= 0.0f || !scene->visible(shadingData.x, vpl.position))
				continue;

			Colour bsdfVal = shadingData.bsdf->evaluate(shadingData, wi);
			L = L + bsdfVal * vpl.radiance * G;
		}
		return L;
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
			Colour bsdfVal;
			float pdfVal;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdfVal, pdfVal);
			pdf = pdfVal;
			bsdf = bsdfVal;
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			r.init(shadingData.x + (wi * EPSILON), wi);
			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}
		return scene->background->evaluate(shadingData, r.dir);
	}



	Colour connectToCamera(const ShadingData& shadingData, Scene* scene)
	{
		Vec3 camPos = scene->camera.origin;
		Vec3 wi = (camPos - shadingData.x).normalize();
		float dist = (camPos - shadingData.x).length();

		if (!scene->visible(shadingData.x, camPos))
			return Colour(0.0f, 0.0f, 0.0f);

		if (shadingData.bsdf && shadingData.bsdf->isPureSpecular())
			return Colour(0.0f, 0.0f, 0.0f);

		float projX, projY;
		if (!scene->camera.projectOntoCamera(shadingData.x, projX, projY))
			return Colour(0.0f, 0.0f, 0.0f);

		float pixelCenterX = std::floor(projX) + 0.5f;
		float pixelCenterY = std::floor(projY) + 0.5f;

		float dx = projX - pixelCenterX;
		float dy = projY - pixelCenterY;
		float filterWeight = film->filter->filter(dx, dy);

		Colour f = shadingData.bsdf ? shadingData.bsdf->evaluate(shadingData, wi) : Colour(1.0f, 1.0f, 1.0f);
		float cosTheta = max(0.0f, shadingData.sNormal.dot(wi));
		float scaleFactor = 100.0f;
		float geom = scaleFactor / (dist * dist);


		return f * cosTheta * geom * filterWeight;
	}



	Colour lightTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);

		if (intersection.t >= FLT_MAX) {
			ShadingData bgData = createDummyShadingData(r.o, Vec3(0, 1, 0));
			return pathThroughput * scene->background->evaluate(bgData, r.dir);
		}

		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		Colour cameraContrib = connectToCamera(shadingData, scene);
		Colour result = pathThroughput * cameraContrib;

		if (depth >= MAX_DEPTH)
			return result;

		float rrProb = min(pathThroughput.Lum(), 0.9f);
		if (sampler->next() >= rrProb)
			return result;
		pathThroughput = pathThroughput / rrProb;

		Colour bsdfVal;
		float pdfVal;
		Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdfVal, pdfVal);
		if (pdfVal < 1e-6f)
			return result;
		float pdf = pdfVal;
		pathThroughput = pathThroughput * bsdfVal * fabsf(Dot(wi, shadingData.sNormal)) / pdf;

		Ray newRay;
		newRay.init(shadingData.x + wi * EPSILON, wi);

		return result + lightTrace(newRay, pathThroughput, depth + 1, sampler);
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


	void denoise(float* color, float* albedo, float* normal, float* output, int width, int height)
	{
		try {
			oidn::DeviceRef device = oidn::newDevice();
			device.commit();

			const size_t size = width * height * 3 * sizeof(float);
			oidn::BufferRef colorBuf = device.newBuffer(size);
			oidn::BufferRef albedoBuf = albedo ? device.newBuffer(size) : oidn::BufferRef();
			oidn::BufferRef normalBuf = normal ? device.newBuffer(size) : oidn::BufferRef();
			oidn::BufferRef outputBuf = device.newBuffer(size);

			std::memcpy(colorBuf.getData(), color, size);
			if (albedo)
				std::memcpy(albedoBuf.getData(), albedo, size);
			if (normal)
				std::memcpy(normalBuf.getData(), normal, size);

			oidn::FilterRef filter = device.newFilter("RT");
			filter.setImage("color", colorBuf, oidn::Format::Float3, width, height);
			if (albedo)
				filter.setImage("albedo", albedoBuf, oidn::Format::Float3, width, height);
			if (normal)
				filter.setImage("normal", normalBuf, oidn::Format::Float3, width, height);
			filter.setImage("output", outputBuf, oidn::Format::Float3, width, height);
			filter.set("hdr", true);

			filter.commit();
			filter.execute();

			std::memcpy(output, outputBuf.getData(), size);

			const char* message;
			if (device.getError(message) != oidn::Error::None)
				std::cerr << "[OIDN Error] " << message << std::endl;
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
		unsigned int pixelCount = film->width * film->height;

		Colour* ambientBuffer = new Colour[pixelCount];
		memset(ambientBuffer, 0, pixelCount * sizeof(Colour));

		auto splatAmbient = [=](Colour* ambientBuffer, float x, float y, const Colour& L) {
			float filterWeights[25];
			unsigned int indices[25];
			unsigned int used = 0;
			float total = 0;
			int filterSize = film->filter->size();
			for (int i = -filterSize; i <= filterSize; i++) {
				for (int j = -filterSize; j <= filterSize; j++) {
					int px = (int)x + j;
					int py = (int)y + i;
					if (px >= 0 && px < film->width && py >= 0 && py < film->height) {
						indices[used] = (py * film->width) + px;
						filterWeights[used] = film->filter->filter(j, i);
						total += filterWeights[used];
						used++;
					}
				}
			}
			for (unsigned int k = 0; k < used; k++) {
				ambientBuffer[indices[k]] = ambientBuffer[indices[k]] + (L * filterWeights[k] / total);
			}
			};

		auto renderTile = [&](int tileX, int tileY, int threadId) {
			int startX = tileX * TILE_SIZE;
			int startY = tileY * TILE_SIZE;
			int endX = min(startX + TILE_SIZE, (int)film->width);
			int endY = min(startY + TILE_SIZE, (int)film->height);
			Sampler* localSampler = &samplers[threadId];

			for (int y = startY; y < endY; y++) {
				for (int x = startX; x < endX; x++) {
					float px = x + 0.5f;
					float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);

					if (film->SPP == 1) {
						int idx = y * film->width + x;
						film->albedo[idx] = getAlbedo(ray);
						film->normal[idx] = viewNormals(ray);
					}

					Colour throughput(1.0f, 1.0f, 1.0f);
					Colour totalRadiance = pathTrace(ray, throughput, 3, localSampler);

					ShadingData sd = {};
					Colour env(0.0f, 0.0f, 0.0f);
					if (scene->background)
						env = scene->background->evaluate(sd, ray.dir);

					Colour direct = totalRadiance - env;
					direct.r = max(direct.r, 0.0f);
					direct.g = max(direct.g, 0.0f);
					direct.b = max(direct.b, 0.0f);

					film->splat(px, py, direct);
					splatAmbient(ambientBuffer, px, py, env);

					Colour display = direct + env;
					unsigned char r = (unsigned char)(min(display.r, 1.0f) * 255);
					unsigned char g = (unsigned char)(min(display.g, 1.0f) * 255);
					unsigned char b = (unsigned char)(min(display.b, 1.0f) * 255);
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
			};

		auto workerFunc = [&](int threadId) {
			for (int tileY = 0; tileY < numTilesY; tileY++) {
				for (int tileX = 0; tileX < numTilesX; tileX++) {
					if (((tileY * numTilesX) + tileX) % numThreads == threadId)
						renderTile(tileX, tileY, threadId);
				}
			}
			};

		for (int i = 0; i < numThreads; i++)
			workers.emplace_back(workerFunc, i);
		for (auto& w : workers)
			w.join();

		savePNG("output.png");


		float* normalizedDirect = new float[pixelCount * 3];
		float* normalizedAmbient = new float[pixelCount * 3];
		for (unsigned int i = 0; i < pixelCount; i++) {
			normalizedDirect[i * 3 + 0] = film->film[i].r / (float)film->SPP;
			normalizedDirect[i * 3 + 1] = film->film[i].g / (float)film->SPP;
			normalizedDirect[i * 3 + 2] = film->film[i].b / (float)film->SPP;

			normalizedAmbient[i * 3 + 0] = ambientBuffer[i].r / (float)film->SPP;
			normalizedAmbient[i * 3 + 1] = ambientBuffer[i].g / (float)film->SPP;
			normalizedAmbient[i * 3 + 2] = ambientBuffer[i].b / (float)film->SPP;
		}


		float* denoisedDirect = new float[pixelCount * 3];
		denoise(normalizedDirect,
			reinterpret_cast<float*>(film->albedo),
			reinterpret_cast<float*>(film->normal),
			denoisedDirect,
			film->width, film->height);

		float* finalImage = new float[pixelCount * 3];
		for (unsigned int i = 0; i < pixelCount; i++) {
			finalImage[i * 3 + 0] = denoisedDirect[i * 3 + 0] + normalizedAmbient[i * 3 + 0];
			finalImage[i * 3 + 1] = denoisedDirect[i * 3 + 1] + normalizedAmbient[i * 3 + 1];
			finalImage[i * 3 + 2] = denoisedDirect[i * 3 + 2] + normalizedAmbient[i * 3 + 2];
		}

		stbi_write_hdr("raw.hdr", film->width, film->height, 3, normalizedDirect);
		stbi_write_hdr("denoised.hdr", film->width, film->height, 3, finalImage);

		delete[] normalizedDirect;
		delete[] normalizedAmbient;
		delete[] denoisedDirect;
		delete[] finalImage;
		delete[] ambientBuffer;
	}



	void renderInstantRadiosity()
	{
		static const int TILE_SIZE = 32;
		film->incrementSPP();

		scene->computeVPLs(50, &samplers[0]);

		Light* envLight = nullptr;
		for (auto light : scene->lights)
		{
			if (!light->isArea())
			{
				envLight = light;
				break;
			}
		}

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

						if (film->SPP == 1)
						{
							int idx = y * film->width + x;
							film->albedo[idx] = getAlbedo(ray);
							film->normal[idx] = viewNormals(ray);
						}

						Colour vplContrib(0.0f, 0.0f, 0.0f);
						Colour envContrib(0.0f, 0.0f, 0.0f);

						IntersectionData intersection = scene->traverse(ray);
						if (intersection.t < FLT_MAX)
						{
							ShadingData shadingData = scene->calculateShadingData(intersection, ray);

							if (shadingData.bsdf->isLight())
							{
								vplContrib = shadingData.bsdf->emit(shadingData, shadingData.wo);
							}
							else
							{
								vplContrib = computeDirectFromVPLs(shadingData);
							}
							if (envLight)
							{
								envContrib = envLight->evaluate(createDummyShadingData(shadingData.x, shadingData.sNormal), ray.dir);
							}
						}
						else
						{
							if (envLight)
							{
								envContrib = envLight->evaluate(createDummyShadingData(ray.o, Vec3(0, 1, 0)), ray.dir);
							}
						}

						Colour finalColor = vplContrib + envContrib;

						film->splat(px, py, finalColor);

						unsigned char r = (unsigned char)(min(finalColor.r, 1.0f) * 255);
						unsigned char g = (unsigned char)(min(finalColor.g, 1.0f) * 255);
						unsigned char b = (unsigned char)(min(finalColor.b, 1.0f) * 255);
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
		for (auto& w : workers)
		{
			w.join();
		}
		savePNG("output.png");

		float* denoised = new float[film->width * film->height * 3];
		denoise(reinterpret_cast<float*>(film->film),
			reinterpret_cast<float*>(film->albedo),
			reinterpret_cast<float*>(film->normal),
			denoised,
			film->width, film->height);
		stbi_write_hdr("raw.hdr", film->width, film->height, 3, reinterpret_cast<float*>(film->film));
		stbi_write_hdr("denoised.hdr", film->width, film->height, 3, denoised);
		delete[] denoised;
	}



	void renderLightTrace()
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

						Ray camRay = scene->camera.generateRay(px, py);

						Colour z(1.0f, 1.0f, 1.0f);
						Colour colLight = lightTrace(camRay, z, 3, localSampler);

						film->splat(px, py, colLight);

						unsigned char r = (unsigned char)(min(colLight.r, 1.0f) * 255);
						unsigned char g = (unsigned char)(min(colLight.g, 1.0f) * 255);
						unsigned char b = (unsigned char)(min(colLight.b, 1.0f) * 255);

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
		denoise(reinterpret_cast<float*>(film->film),
			reinterpret_cast<float*>(film->albedo),
			reinterpret_cast<float*>(film->normal),
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