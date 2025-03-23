#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"


#pragma warning( disable : 4244)

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Add code here
		return 1.0f;
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// Add code here
		return Colour(1.0f, 1.0f, 1.0f);
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		return 1.0f;
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		return 1.0f;
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		return 1.0f;
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}


	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();

		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(r1, r2);

		float cosTheta = wiLocal.y;
		pdf = cosTheta / M_PI;

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;

		Vec3 wi = shadingData.frame.toWorld(wiLocal);

		return wi;
	}


	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		float cosTheta = wiLocal.y;

		if (cosTheta <= 0.0f)
			return 0.0f;

		return cosTheta / M_PI;
	}

	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 n = shadingData.sNormal;
		Vec3 wo = shadingData.wo;

		Vec3 wi = -wo + n * 2.0f  * (wo.dot(n)) ;

		pdf = 1.0f;

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);

		return wi;
	}


	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{

		return Colour(0.0f, 0.0f, 0.0f);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 0.0f;
	}
	

	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}


	Colour FresnelConductor(float cosTheta, const Colour& eta, const Colour& k)
	{

		Colour eta2 = eta * eta;
		Colour k2 = k * k;
		Colour etaK2 = eta2 + k2; 

		float cos2 = cosTheta * cosTheta;
		Colour twoEtaCos = eta * (2.0f * cosTheta);

		Colour t = etaK2 - (twoEtaCos)+cos2;
		Colour rparl2 = (t * t) / (etaK2 + twoEtaCos + cos2).pow(2.0f);

		Colour rperp2_num = (etaK2 * cos2) - (twoEtaCos * cosTheta) + 1.0f;
		Colour rperp2_denom = (etaK2 * cos2) + (twoEtaCos * cosTheta) + 1.0f;
		Colour rperp2 = (rperp2_num * rperp2_num) / (rperp2_denom * rperp2_denom);

		return (rparl2 + rperp2) * 0.5f;
	}

	float D_GGX(const Vec3& wh, float alpha)
	{
		float alpha2 = alpha * alpha;
		float cosThetaH = wh.z;
		float cos2 = cosThetaH * cosThetaH;
		float denom = cos2 * (alpha2 - 1.0f) + 1.0f;
		denom = denom * denom;
		return alpha2 / (M_PI * denom);
	}

	float G1_GGX(const Vec3& v, float alpha)
	{
		float cosThetaV = std::fabs(v.z);
		if (cosThetaV < 1e-4f) return 0.0f;

		float sinThetaV = std::sqrt(std::max(0.0f, 1.0f - cosThetaV * cosThetaV));
		float tanThetaV = (sinThetaV / cosThetaV);
		float alpha2 = alpha * alpha;
		float tmp = alpha2 * tanThetaV * tanThetaV;
		return 2.0f / (1.0f + std::sqrt(1.0f + tmp));
	}

	float G_SmithGGX(const Vec3& wo, const Vec3& wi, float alpha)
	{
		return G1_GGX(wo, alpha) * G1_GGX(wi, alpha);
	}

	Vec3 sampleGGX_H(float r1, float r2, float alpha)
	{
		float alpha2 = alpha * alpha;

		float phi = 2.0f * M_PI * r1;
		float cosTheta = std::sqrt((1.0f - r2) / (1.0f + (alpha2 - 1.0f) * r2));
		float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));

		Vec3 wh;
		wh.x = sinTheta * std::cos(phi);
		wh.y = sinTheta * std::sin(phi);
		wh.z = cosTheta;

		return wh;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 woWorld = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(woWorld);

		if (woLocal.z <= 0.0f) {
			pdf = 0.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 whLocal = sampleGGX_H(r1, r2, alpha);

		Vec3 wiLocal = -woLocal.reflect(whLocal);
		if (wiLocal.z <= 0.0f) {
			pdf = 0.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f,0.0f, 0.0f);
		}

		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);

		float Dval = D_GGX(whLocal, alpha);
		float cosWO_WH = std::fabs(woLocal.dot(whLocal));
		float cosThetaO = woLocal.z;
		float G1val = G1_GGX(woLocal, alpha);
		pdf = (Dval * G1val * cosWO_WH) / (4.0f * cosThetaO);

		float cosThetaI = wiLocal.z;
		float cosThetaH = whLocal.z;
		float cosDH = std::fabs(wiLocal.dot(whLocal));
		Colour F = FresnelConductor(cosDH, eta, k);
		float D = Dval;
		float G = G_SmithGGX(woLocal, wiLocal, alpha);

		Colour Fr = F * baseColor * (D * G / (4.0f * cosThetaO * cosThetaI));

		reflectedColour = Fr / pdf;

		return wiWorld;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wiWorld)
	{
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);

		if (woLocal.z <= 0.0f || wiLocal.z <= 0.0f)
			return Colour(0.0f, 0.0f, 0.0f);

		Vec3 whLocal = (woLocal + wiLocal).normalize();
		if (whLocal.z < 0.0f) {
			whLocal = -whLocal;
		}

		float D = D_GGX(whLocal, alpha);
		float G = G_SmithGGX(woLocal, wiLocal, alpha);
		float cosDH = std::fabs(woLocal.dot(whLocal));
		Colour F = FresnelConductor(cosDH, eta, k);

		float cosThetaO = std::fabs(woLocal.z);
		float cosThetaI = std::fabs(wiLocal.z);

		Colour Fr = F * baseColor * (D * G / (4.0f * cosThetaO * cosThetaI));
		return Fr;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wiWorld)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);

		if (woLocal.z <= 0.0f || wiLocal.z <= 0.0f)
			return 0.0f;

		Vec3 whLocal = (woLocal + wiLocal).normalize();
		if (whLocal.z < 0) whLocal = -whLocal;

		float D = D_GGX(whLocal, alpha);
		float G1 = G1_GGX(woLocal, alpha);
		float cosWO_WH = std::fabs(woLocal.dot(whLocal));
		float cosThetaO = std::fabs(woLocal.z);

		float pdfVal = (D * G1 * cosWO_WH) / (4.0f * cosThetaO);
		return pdfVal;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}

	inline float fresnelDielectric(float cosThetaI, float extIOR, float intIOR)
	{

		cosThetaI = fabsf(cosThetaI);

		float etaI = extIOR;
		float etaT = intIOR;

		float sinThetaI = sqrtf(std::max(0.0f, 1.0f - cosThetaI * cosThetaI));
		float eta = etaI / etaT;
		float sinThetaT = eta * sinThetaI;

		if (sinThetaT >= 1.0f) {
			return 1.0f;
		}

		float cosThetaT = sqrtf(std::max(0.0f, 1.0f - sinThetaT * sinThetaT));

		float Rs = (etaI * cosThetaI - etaT * cosThetaT) / (etaI * cosThetaI + etaT * cosThetaT);
		float Rp = (etaT * cosThetaI - etaI * cosThetaT) / (etaT * cosThetaI + etaI * cosThetaT);
		return 0.5f * (Rs * Rs + Rp * Rp);
	}

	inline Vec3 refract(const Vec3& I, float cosThetaI, float eta)
	{
		float sinThetaI2 = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
		float sinThetaT2 = eta * eta * sinThetaI2;
		float cosThetaT = sqrtf(std::max(0.0f, 1.0f - sinThetaT2));

		Vec3 T;
		T.x = eta * (-I.x);
		T.y = eta * (-I.y);
		if (I.z >= 0.0f)
			T.z = -cosThetaT;
		else
			T.z = cosThetaT;

		return T;
	}


	Vec3 sample(const ShadingData& shadingData, Sampler* sampler,
		Colour& reflectedColour, float& pdf)
	{
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 woWorld = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(woWorld);

		float cosThetaO = woLocal.z;
		if (fabsf(cosThetaO) < 1e-6f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}

		bool entering = (cosThetaO > 0.0f);
		float n1 = entering ? extIOR : intIOR;
		float n2 = entering ? intIOR : extIOR;
		float eta = n1 / n2;

		float F = fresnelDielectric(cosThetaO, n1, n2);

		float randVal = sampler->next();

		Vec3 wiLocal(0.f, 0.f, 0.f);
		if (randVal < F) {
			Vec3 I = -woLocal;
			float dotVal = I.z; 
			wiLocal = Vec3(I.x - 2.0f * dotVal * 0.f,
				I.y - 2.0f * dotVal * 0.f,
				I.z - 2.0f * dotVal * 1.f);
			pdf = F;

			reflectedColour = baseColor;
		}
		else {
			float cosThetaI = woLocal.z;
			Vec3 I = -woLocal; 
			float sinThetaI2 = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
			float eta2 = eta * eta;
			float sinThetaT2 = eta2 * sinThetaI2;
			if (sinThetaT2 >= 1.0f) {
				float dotVal = I.z;
				wiLocal.z = I.z;
				pdf = 1.f; 
				reflectedColour = baseColor;
			}
			else {
				wiLocal = refract(I, cosThetaI, eta);
				pdf = 1.0f - F;
				reflectedColour = baseColor;
			}
		}

		wiLocal.normalize();
		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);

		return wiWorld;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}

	inline float fresnelDielectric(float cosThetaI, float extIOR, float intIOR)
	{
		cosThetaI = fabsf(cosThetaI);

		float etaI = extIOR;
		float etaT = intIOR;

		float sinThetaI = sqrtf(std::max(0.0f, 1.f - cosThetaI * cosThetaI));
		float eta = etaI / etaT;
		float sinThetaT = eta * sinThetaI;

		if (sinThetaT >= 1.f) {
			return 1.f;
		}

		float cosThetaT = sqrtf(std::max(0.f, 1.f - sinThetaT * sinThetaT));

		float Rs = (etaI * cosThetaI - etaT * cosThetaT)
			/ (etaI * cosThetaI + etaT * cosThetaT);
		float Rp = (etaT * cosThetaI - etaI * cosThetaT)
			/ (etaT * cosThetaI + etaI * cosThetaT);
		return 0.5f * (Rs * Rs + Rp * Rp);
	}

	inline float D_GGX(const Vec3& wh, float alpha)
	{
		float alpha2 = alpha * alpha;
		float cosThetaH = wh.z;
		float cos2 = cosThetaH * cosThetaH;
		float denom = cos2 * (alpha2 - 1.0f) + 1.0f;
		denom = denom * denom;
		return alpha2 / (M_PI * denom);
	}

	inline float G1_GGX(const Vec3& v, float alpha)
	{
		float cosThetaV = fabsf(v.z);
		if (cosThetaV < 1e-4f) return 0.f;
		float sinThetaV = sqrtf(std::max(0.f, 1.f - cosThetaV * cosThetaV));
		float tanThetaV = sinThetaV / cosThetaV;
		float a2 = alpha * alpha;
		float tmp = a2 * tanThetaV * tanThetaV;
		return 2.f / (1.f + sqrtf(1.f + tmp));
	}
	inline float G_SmithGGX(const Vec3& wo, const Vec3& wi, float alpha)
	{
		return G1_GGX(wo, alpha) * G1_GGX(wi, alpha);
	}

	inline Vec3 reflect(const Vec3& I, const Vec3& N)
	{
		float dotVal = I.x * N.x + I.y * N.y + I.z * N.z;
		return Vec3(I.x - 2.f * dotVal * N.x,
			I.y - 2.f * dotVal * N.y,
			I.z - 2.f * dotVal * N.z);
	}

	inline Vec3 refract(const Vec3& I, const Vec3& N, float eta, bool& TIR)
	{
		float cosThetaI = I.x * N.x + I.y * N.y + I.z * N.z;
		float sinThetaI2 = std::max(0.f, 1.f - cosThetaI * cosThetaI);
		float sinThetaT2 = eta * eta * sinThetaI2;
		if (sinThetaT2 > 1.f) {
			TIR = true;
			return Vec3(0.f, 0.f, 0.f);
		}
		TIR = false;
		float cosThetaT = sqrtf(1.f - sinThetaT2);

		float sign = (cosThetaI < 0.f) ? 1.f : -1.f;

		Vec3 T;
		float temp = eta * cosThetaI - sign * cosThetaT;
		T.x = eta * (-I.x) + temp * N.x;
		T.y = eta * (-I.y) + temp * N.y;
		T.z = eta * (-I.z) + temp * N.z;
		return T;
	}

	inline Vec3 sampleGGX_H(float r1, float r2, float alpha)
	{
		float alpha2 = alpha * alpha;
		float phi = 2.f * M_PI * r1;
		float cosTheta = sqrtf((1.f - r2) / (1.f + (alpha2 - 1.f) * r2));
		float sinTheta = sqrtf(std::max(0.f, 1.f - cosTheta * cosTheta));

		return Vec3(sinTheta * cosf(phi),
			sinTheta * sinf(phi),
			cosTheta);
	}


	//Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	//{
	//	// Replace this with Dielectric sampling code
	//	Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
	//	pdf = wi.z / M_PI;
	//	reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	//	wi = shadingData.frame.toWorld(wi);
	//	return wi;
	//}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler,
		Colour& reflectedColour, float& pdf)
	{
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 woWorld = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(woWorld);
		if (std::fabs(woLocal.z) < 1e-6f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}

		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 whLocal = sampleGGX_H(r1, r2, alpha);
		whLocal.normalize();

		Vec3 I = -woLocal;
		Vec3 wiReflect = reflect(I, whLocal);

		bool TIR = false;
		float n1 = extIOR, n2 = intIOR;
		if (woLocal.z < 0.f) std::swap(n1, n2);
		float eta = n1 / n2;
		Vec3 wiRefract = refract(I, whLocal, eta, TIR);

		float cosThetaI = I.x * whLocal.x + I.y * whLocal.y + I.z * whLocal.z;
		cosThetaI = std::fabs(cosThetaI);
		float F = fresnelDielectric(cosThetaI, n1, n2);

		float prob = sampler->next();
		Vec3 wiLocal;
		if (TIR) {
			wiLocal = wiReflect;
			F = 1.f;
		}
		else {
			if (prob < F) {
				wiLocal = wiReflect;
			}
			else {
				wiLocal = wiRefract;
			}
		}

		if (wiLocal.lengthSq() < 1e-12f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}
		wiLocal.normalize();

		float cosWH = std::fabs(I.dot(whLocal));
		float Dval = D_GGX(whLocal, alpha);
		float cosThetaO = std::fabs(woLocal.z);
		float pdfRefl = (Dval * cosWH) / (4.f * cosThetaO);
		float pdfTrans = pdfRefl;
		float finalPdf = (prob < F) ? (F * pdfRefl) : ((1.f - F) * pdfTrans);

		if (finalPdf < 1e-12f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return shadingData.frame.toWorld(wiLocal);
		}
		pdf = finalPdf;

		Colour fVal = evaluate(shadingData, shadingData.frame.toWorld(wiLocal));
		reflectedColour = fVal / pdf;

		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);
		return wiWorld;
	}


	//Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	//{
	//	// Replace this with Dielectric evaluation code
	//	return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	//}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wiWorld)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);

		bool reflection = (woLocal.z * wiLocal.z > 0.f);

		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 whLocal;
		if (reflection) {
			whLocal = (woLocal + wiLocal).normalize();
			if (whLocal.z < 0.f) whLocal = -whLocal;
		}
		else {
			float n1 = extIOR, n2 = intIOR;
			if (woLocal.z < 0.f) std::swap(n1, n2);
			float eta = n1 / n2;
			Vec3 I = -woLocal;
			whLocal = (woLocal + wiLocal * eta).normalize();
			if (whLocal.z < 0.f) whLocal = -whLocal;
		}

		float Dval = D_GGX(whLocal, alpha);
		float Gval = G_SmithGGX(woLocal, wiLocal, alpha);

		Vec3 I = -woLocal;
		float cosThetaI = std::fabs(I.x * whLocal.x + I.y * whLocal.y + I.z * whLocal.z);

		float n1 = extIOR, n2 = intIOR;
		if (woLocal.z < 0.f) std::swap(n1, n2);
		float F_ = fresnelDielectric(cosThetaI, n1, n2);

		float cosThetaO = std::fabs(woLocal.z);

		float fr = 0.f;
		float ft = 0.f;
		if (reflection)
		{
			fr = F_ * Dval * Gval / (4.f * cosThetaO * cosThetaI);
		}
		else
		{
			float eta2 = (n1 / n2) * (n1 / n2);
			ft = (1.f - F_) * Dval * Gval * eta2 *
				std::fabs((wiLocal.dot(whLocal) * woLocal.dot(whLocal)) / (cosThetaO * cosThetaI * (whLocal.z * whLocal.z)));
		}

		float fSum = fr + ft;
		return baseColor * fSum;
	}



	//float PDF(const ShadingData& shadingData, const Vec3& wi)
	//{
	//	// Replace this with Dielectric PDF
	//	Vec3 wiLocal = shadingData.frame.toLocal(wi);
	//	return SamplingDistributions::cosineHemispherePDF(wiLocal);
	//}

	float PDF(const ShadingData& shadingData, const Vec3& wiWorld)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);

		float n1 = extIOR, n2 = intIOR;
		if (woLocal.z < 0.f) std::swap(n1, n2);
		Vec3 I = -woLocal;

		bool reflection = (woLocal.z * wiLocal.z > 0.f);
		Vec3 whLocal;
		if (reflection) {
			whLocal = (woLocal + wiLocal).normalize();
			if (whLocal.z < 0.f) whLocal = -whLocal;
		}
		else {
			float eta = n1 / n2;
			whLocal = (woLocal + wiLocal * eta).normalize();
			if (whLocal.z < 0.f) whLocal = -whLocal;
		}

		float Dval = D_GGX(whLocal, alpha);
		float cosThetaI = std::fabs(I.dot(whLocal));
		float F_ = fresnelDielectric(cosThetaI, n1, n2);
		float cosThetaO = std::fabs(woLocal.z);

		float pdfRefl = (Dval * cosThetaI) / (4.f * cosThetaO);
		float pdfTrans = pdfRefl;

		bool sameSide = (woLocal.z * wiLocal.z > 0.f);
		if (sameSide) {
			return F_ * pdfRefl;
		}
		else {
			return (1.f - F_) * pdfTrans;
		}
	}


	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler,
		Colour& reflectedColour, float& pdf)
	{
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(r1, r2);

		float cosTheta = wiLocal.z;
		pdf = cosTheta / M_PI;

		reflectedColour = baseColor / M_PI;

		Vec3 wi = shadingData.frame.toWorld(wiLocal);

		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wiWorld)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);

		if (woLocal.z <= 0.0f || wiLocal.z <= 0.0f)
			return Colour(0.0f, 0.0f, 0.0f);

		float theta_r = std::acos(std::max(-1.f, std::min(1.f, woLocal.z)));
		float theta_i = std::acos(std::max(-1.f, std::min(1.f, wiLocal.z)));

		float phi_r = std::atan2(woLocal.y, woLocal.x);
		float phi_i = std::atan2(wiLocal.y, wiLocal.x);

		float alpha = std::max(theta_i, theta_r);
		float beta = std::min(theta_i, theta_r);

		float sigmaRad = sigma;
		float s2 = sigmaRad * sigmaRad;

		float A = 1.0f - 0.5f * (s2 / (s2 + 0.33f));
		float B = 0.45f * (s2 / (s2 + 0.09f));

		float delta = phi_i - phi_r;
		float cosDP = std::cos(delta);

		float cosDeltaPhi = std::max(0.0f, cosDP);
		float sinAlpha = std::sin(alpha);
		float tanBeta = std::tan(beta);

		float orenTerm = A + B * cosDeltaPhi * sinAlpha * tanBeta;

		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
		Colour f = baseColor * (orenTerm / M_PI);

		return f;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (wiLocal.z <= 0.0f)
			return 0.0f;

		return (wiLocal.z / M_PI);
	}

	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}

	float fresnelSchlick(float cosTheta, float R0)
	{
		cosTheta = std::fabs(cosTheta);
		return R0 + (1.f - R0) * std::pow((1.f - cosTheta), 5.f);
	}

	Vec3 sample(const ShadingData& shadingData,
		Sampler* sampler,
		Colour& reflectedColour,
		float& pdf)
	{
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		float eta = (extIOR / intIOR);
		float r0 = (eta - 1.f) / (eta + 1.f);
		r0 = r0 * r0;

		Vec3 woWorld = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(woWorld);
		if (woLocal.z <= 0.f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}
		float cosThetaO = woLocal.z;
		float fresnel = fresnelSchlick(cosThetaO, r0);

		float randVal = sampler->next();
		bool chooseSpecular = (randVal < fresnel);

		Vec3 wiLocal(0.f, 0.f, 0.f);
		float specPdf = 0.f, diffPdf = 0.f;
		Colour specF, diffF;

		float exponent = alphaToPhongExponent();

		if (chooseSpecular) {
			float r1 = sampler->next();
			float r2 = sampler->next();
			float phi = 2.f * M_PI * r1;
			float cosT = std::pow(r2, 1.f / (exponent + 1.f));
			float sinT = std::sqrt(std::max(0.f, 1.f - cosT * cosT));

			Vec3 whLocal = Vec3(
				sinT * std::cos(phi),
				sinT * std::sin(phi),
				cosT
			);
			whLocal.normalize();

			Vec3 I = -woLocal;
			float dotVal = I.x * whLocal.x + I.y * whLocal.y + I.z * whLocal.z;
			wiLocal = Vec3(
				I.x - 2.f * dotVal * whLocal.x,
				I.y - 2.f * dotVal * whLocal.y,
				I.z - 2.f * dotVal * whLocal.z
			);
			if (wiLocal.z <= 0.f) {
				pdf = 0.f;
				reflectedColour = Colour(0.f, 0.f, 0.f);
				return shadingData.frame.toWorld(wiLocal);
			}
			wiLocal.normalize();

			float cosH = whLocal.z;
			float blinnPdf = ((exponent + 1.f) / (2.f * M_PI)) * (std::pow(cosH, exponent)) / (4.f * std::fabs(woLocal.dot(whLocal)));
			specPdf = blinnPdf;

			float cosThetaH = std::max(0.f, whLocal.z);
			float specVal = (exponent + 2.f) / (2.f * M_PI) * std::pow(cosThetaH, exponent);
			specF = Colour(specVal, specVal, specVal);

			diffF = Colour(0.f, 0.f, 0.f);
			diffPdf = 0.f;
		}
		else {
			float r1 = sampler->next();
			float r2 = sampler->next();
			wiLocal = SamplingDistributions::cosineSampleHemisphere(r1, r2);
			if (wiLocal.z <= 0.f) {
				pdf = 0.f;
				reflectedColour = Colour(0.f, 0.f, 0.f);
				return shadingData.frame.toWorld(wiLocal);
			}
			diffPdf = (wiLocal.z / M_PI);

			diffF = baseColor * (1.f / M_PI);

			specF = Colour(0.f, 0.f, 0.f);
			specPdf = 0.f;
		}

		float finalPdf = chooseSpecular ? (fresnel * specPdf) : ((1.f - fresnel) * diffPdf);
		if (finalPdf < 1e-10f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return shadingData.frame.toWorld(wiLocal);
		}

		pdf = finalPdf;

		float cosThetaI = wiLocal.z;
		Colour brdfVal = diffF + specF;
		reflectedColour = brdfVal * (cosThetaI / finalPdf);

		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);
		return wiWorld;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wiWorld)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);

		if (woLocal.z <= 0.f || wiLocal.z <= 0.f)
			return Colour(0.f, 0.f, 0.f);


		float cosThetaO = woLocal.z;
		float eta = extIOR / intIOR;
		float r0 = (eta - 1.f) / (eta + 1.f);
		r0 = r0 * r0;
		float fresnel = fresnelSchlick(cosThetaO, r0);

		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
		Colour diffPart = baseColor * ((1.f - fresnel) / M_PI);

		float exponent = alphaToPhongExponent();
		Vec3 H = (wiLocal + woLocal).normalize();
		if (H.z < 0.f) H = -H;
		float cosThetaH = std::max(0.f, H.z);
		float specVal = (exponent + 2.f) / (2.f * M_PI) * std::pow(cosThetaH, exponent);
		Colour specPart = Colour(specVal, specVal, specVal) * fresnel;

		Colour f = diffPart + specPart;
		return f;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wiWorld)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);
		if (woLocal.z <= 0.f || wiLocal.z <= 0.f)
			return 0.f;

		float cosThetaO = woLocal.z;
		float eta = extIOR / intIOR;
		float r0 = (eta - 1.f) / (eta + 1.f);
		r0 = r0 * r0;
		float F = fresnelSchlick(cosThetaO, r0);

		float pdfDiff = (wiLocal.z / M_PI);

		float exponent = alphaToPhongExponent();
		Vec3 H = (wiLocal + woLocal).normalize();
		if (H.lengthSq() < 1e-12f) return pdfDiff * (1.f - F);
		float cosWH = std::fabs(woLocal.dot(H));
		float cosH = std::max(0.f, H.z);
		float pdfSpec = ((exponent + 1.f) / (2.f * M_PI)) * std::pow(cosH, exponent) / (4.f * cosWH);

		float finalPdf = F * pdfSpec + (1.f - F) * pdfDiff;
		return finalPdf;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler,
		Colour& reflectedColour, float& pdf)
	{
		Vec3 wiWorld = base->sample(shadingData, sampler, reflectedColour, pdf);

		if (pdf < 1e-10f || reflectedColour.Lum() < 1e-10f)
			return wiWorld;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);

		bool transmitted = (woLocal.z * wiLocal.z < 0.0f);

		if (transmitted)
		{
			Colour attenuation = Colour::exp( -sigmaa * thickness);
			reflectedColour = reflectedColour * attenuation;
		}

		return wiWorld;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wiWorld)
	{
		Colour baseVal = base->evaluate(shadingData, wiWorld);
		if (baseVal.Lum() < 1e-10f)
			return baseVal;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);
		bool transmitted = (woLocal.z * wiLocal.z < 0.0f);

		if (transmitted)
		{
			Colour attenuation = Colour::exp(-sigmaa * thickness);
			baseVal = baseVal * attenuation;
		}

		return baseVal;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};