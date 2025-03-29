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

	virtual Vec3 sample(const ShadingData& shadingData,
		Sampler* sampler,
		Colour& reflectedColour,
		float& pdf) override
	{
		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());

		pdf = wiLocal.z / M_PI;

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;

		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);
		return wiWorld;
	}

	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wiWorld) override
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}

	virtual float PDF(const ShadingData& shadingData, const Vec3& wiWorld) override
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}

	virtual bool isPureSpecular() override
	{
		return false;
	}

	virtual bool isTwoSided() override
	{
		return true;
	}

	virtual float mask(const ShadingData& shadingData) override
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

	virtual Vec3 sample(const ShadingData& shadingData,
		Sampler* /*sampler*/,
		Colour& reflectedColour,
		float& pdf) override
	{
		const Vec3& n = shadingData.sNormal;
		const Vec3& wo = shadingData.wo;

		Vec3 wi = -wo + n * 2.0f * (wo.dot(n));

		pdf = 1.0f;

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);

		return wi;
	}


	virtual Colour evaluate(const ShadingData& /*shadingData*/,
		const Vec3& /*wi*/) override
	{

		return Colour(0.0f, 0.0f, 0.0f);
	}

	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) override
	{
		const Vec3& n = shadingData.sNormal;
		const Vec3& wo = shadingData.wo;
		Vec3 mirrorDir = -wo + n * 2.0f * (wo.dot(n));

		float dotVal = (wi - mirrorDir).lengthSq();
		if (dotVal < 1e-12f)
			return 1.0f;
		else
			return 0.0f;
	}

	virtual bool isPureSpecular() override
	{
		return true;
	}

	virtual bool isTwoSided() override
	{
		return true;
	}

	virtual float mask(const ShadingData& shadingData) override
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF {
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;

	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness) {
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}


	Colour FresnelConductor(float cosTheta, const Colour& eta, const Colour& k) {
		Colour eta2 = eta * eta;
		Colour k2 = k * k;
		Colour etaK2 = eta2 + k2;
		float cos2 = cosTheta * cosTheta;
		Colour r_perp_num = (etaK2 - (eta * cosTheta * 2.0f) + cos2);
		Colour r_perp_denom = (etaK2 + (eta * cosTheta * 2.0f) + cos2);
		Colour r_perp = r_perp_num / r_perp_denom;
		Colour r_parl_num = (etaK2 * cos2) - (eta * cosTheta * 2.0f) + 1.0f;
		Colour r_parl_denom = (etaK2 * cos2) + (eta * cosTheta * 2.0f) + 1.0f;
		Colour r_parl = r_parl_num / r_parl_denom;
		return (r_parl + r_perp) * 0.5f;
	}

	float D_GGX(const Vec3& wh, float alpha) {
		float alpha2 = alpha * alpha;
		float cosThetaH = wh.z;
		float cos2 = cosThetaH * cosThetaH;
		float denom = cos2 * (alpha2 - 1.0f) + 1.0f;
		denom = denom * denom;
		return alpha2 / (M_PI * denom);
	}

	float G1_GGX(const Vec3& v, float alpha) {
		float cosThetaV = fabs(v.z);
		if (cosThetaV < 1e-4f) return 0.0f;
		float sinThetaV = sqrtf(std::max(0.0f, 1.0f - cosThetaV * cosThetaV));
		float tanThetaV = sinThetaV / cosThetaV;
		float alpha2 = alpha * alpha;
		float tmp = alpha2 * tanThetaV * tanThetaV;
		return 2.0f / (1.0f + sqrtf(1.0f + tmp));
	}

	float G_SmithGGX(const Vec3& wo, const Vec3& wi, float alpha) {
		return G1_GGX(wo, alpha) * G1_GGX(wi, alpha);
	}

	Vec3 sampleGGX_H(float r1, float r2, float alpha) {
		float alpha2 = alpha * alpha;
		float phi = 2.0f * M_PI * r1;
		float cosTheta = sqrtf((1.0f - r2) / (1.0f + (alpha2 - 1.0f) * r2));
		float sinTheta = sqrtf(std::max(0.0f, 1.0f - cosTheta * cosTheta));
		Vec3 wh;
		wh.x = sinTheta * cosf(phi);
		wh.y = sinTheta * sinf(phi);
		wh.z = cosTheta;
		return wh;
	}

	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) override {
		Vec3 woWorld = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(woWorld);
		float cosThetaO = fabs(woLocal.z);
		if (cosThetaO < 1e-6f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}

		float methodSelector = sampler->next();
		Vec3 wiLocal;
		float pdf_GGX, pdf_uni;
		Vec3 whLocal;
		if (methodSelector < 0.5f) {
			float r1 = sampler->next();
			float r2 = sampler->next();
			whLocal = sampleGGX_H(r1, r2, alpha);
			if (whLocal.z * woLocal.z < 0.f)
				whLocal = -whLocal;
			wiLocal = -woLocal.reflect(whLocal);
			float cosThetaI = fabs(wiLocal.z);
			if (cosThetaI < 1e-6f) {
				pdf = 0.f;
				reflectedColour = Colour(0.f, 0.f, 0.f);
				return Vec3(0.f, 0.f, 0.f);
			}
			float cosWO_WH = fabs(woLocal.dot(whLocal));
			pdf_GGX = (D_GGX(whLocal, alpha) * fabs(whLocal.z)) / (4.f * cosWO_WH);
			pdf_uni = 1.0f / (2.0f * M_PI);
		}
		else {
			wiLocal = SamplingDistributions::uniformSampleHemisphere(sampler->next(), sampler->next());
			float cosThetaI = fabs(wiLocal.z);
			if (cosThetaI < 1e-6f) {
				pdf = 0.f;
				reflectedColour = Colour(0.f, 0.f, 0.f);
				return Vec3(0.f, 0.f, 0.f);
			}
			whLocal = (woLocal + wiLocal).normalize();
			if (whLocal.z < 0.f)
				whLocal = -whLocal;
			pdf_uni = 1.0f / (2.0f * M_PI);
			float cosWO_WH = fabs(woLocal.dot(whLocal));
			pdf_GGX = (D_GGX(whLocal, alpha) * fabs(whLocal.z)) / (4.f * cosWO_WH);
		}

		float pdf_combined = 0.5f * pdf_GGX + 0.5f * pdf_uni;

		float cosThetaI = fabs(wiLocal.z);
		float cosWH = fabs(wiLocal.dot(whLocal));
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
		Colour F = FresnelConductor(cosWH, eta, k);
		float D = D_GGX(whLocal, alpha);
		float G = G_SmithGGX(woLocal, wiLocal, alpha);
		Colour bsdf = F * baseColor * (D * G / (4.f * cosThetaO * cosThetaI));

		reflectedColour = bsdf * cosThetaI;
		pdf = pdf_combined;

		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);
		return wiWorld;
	}

	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wiWorld) override {
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);

		float cosThetaO = fabs(woLocal.z);
		float cosThetaI = fabs(wiLocal.z);
		if (cosThetaO < 1e-6f || cosThetaI < 1e-6f)
			return Colour(0.f, 0.f, 0.f);

		Vec3 whLocal = (woLocal + wiLocal).normalize();
		if (whLocal.z < 0.f)
			whLocal = -whLocal;

		float D = D_GGX(whLocal, alpha);
		float G = G_SmithGGX(woLocal, wiLocal, alpha);
		float cosWH = fabs(woLocal.dot(whLocal));
		Colour F = FresnelConductor(cosWH, eta, k);
		Colour bsdf = F * baseColor * (D * G / (4.f * cosThetaO * cosThetaI));
		return bsdf;
	}

	virtual float PDF(const ShadingData& shadingData, const Vec3& wiWorld) override {
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);

		float cosThetaO = fabs(woLocal.z);
		float cosThetaI = fabs(wiLocal.z);
		if (cosThetaO < 1e-6f || cosThetaI < 1e-6f)
			return 0.f;

		Vec3 whLocal = (woLocal + wiLocal).normalize();
		if (whLocal.z < 0.f)
			whLocal = -whLocal;
		float pdf_GGX = (D_GGX(whLocal, alpha) * fabs(whLocal.z)) / (4.f * fabs(woLocal.dot(whLocal)));
		float pdf_uni = 1.0f / (2.0f * M_PI);
		return 0.5f * pdf_GGX + 0.5f * pdf_uni;
	}

	virtual bool isPureSpecular() override {
		return false;
	}

	virtual bool isTwoSided() override {
		return true;
	}

	virtual float mask(const ShadingData& shadingData) override {
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};



class GlassBSDF : public BSDF {
public:
	Texture* albedo;
	float intIOR;
	float extIOR;

	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
		: albedo(_albedo), intIOR(_intIOR), extIOR(_extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}

	inline float fresnelDielectric(float cosThetaI, float extIOR, float intIOR) {
		cosThetaI = fabs(cosThetaI);
		float etaI = extIOR;
		float etaT = intIOR;

		float sinThetaI = sqrtf(std::max(0.0f, 1.0f - cosThetaI * cosThetaI));
		float eta = etaI / etaT;
		float sinThetaT = eta * sinThetaI;

		if (sinThetaT >= 1.0f) {
			return 1.0f;
		}

		float cosThetaT = sqrtf(std::max(0.0f, 1.0f - sinThetaT * sinThetaT));
		float Rs = (etaI * cosThetaI - etaT * cosThetaT)
			/ (etaI * cosThetaI + etaT * cosThetaT);
		float Rp = (etaT * cosThetaI - etaI * cosThetaT)
			/ (etaT * cosThetaI + etaI * cosThetaT);
		return 0.5f * (Rs * Rs + Rp * Rp);
	}

	inline Vec3 reflectLocal(const Vec3& I) {
		return Vec3(I.x, I.y, -I.z);
	}

	inline Vec3 refract(const Vec3& I, float cosThetaI, float eta)
	{
		float sinThetaI2 = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
		float sinThetaT2 = eta * eta * sinThetaI2;
		float cosThetaT = sqrtf(std::max(0.0f, 1.0f - sinThetaT2));

		Vec3 T;
		T.x = eta * (-I.x);
		T.y = eta * (-I.y);
		if (I.z < 0.0f)
			T.z = -cosThetaT;
		else
			T.z = cosThetaT;

		return T;
	}

	virtual Vec3 sample(const ShadingData& shadingData,
		Sampler* sampler,
		Colour& reflectedColour,
		float& pdf) override
	{
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 woWorld = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(woWorld);

		float cosThetaO = woLocal.z;
		if (fabs(cosThetaO) < 1e-6f) {
			pdf = 0.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		bool entering = (cosThetaO > 0.0f);
		float n1 = entering ? extIOR : intIOR;
		float n2 = entering ? intIOR : extIOR;
		float eta = n1 / n2;

		float F = fresnelDielectric(cosThetaO, n1, n2);

		float randVal = sampler->next();

		float branchSelector = sampler->next();
		bool sampleDelta = (branchSelector < 0.5f);
		Vec3 wiLocal;
		float pdf_delta = 0.0f;
		float pdf_uni = 1.0f / (2.0f * M_PI);

		if (sampleDelta) {
			float r = sampler->next();
			if (r < F) {
				wiLocal = reflectLocal(-woLocal);
				pdf_delta = F;
			}
			else {
				float sinThetaI2 = std::max(0.0f, 1.0f - cosThetaO * cosThetaO);
				float sinThetaT2 = eta * eta * sinThetaI2;
				if (sinThetaT2 >= 1.0f) {
					wiLocal = reflectLocal(-woLocal);
					pdf_delta = 1.0f;
				}
				else {
					wiLocal = refract(-woLocal, cosThetaO, eta);
					pdf_delta = 1.0f - F;
				}
			}
		}
		else {
			wiLocal = SamplingDistributions::uniformSampleHemisphere(sampler->next(), sampler->next());
			pdf_delta = 0.0f;
		}

		pdf = 0.5f * pdf_delta + 0.5f * pdf_uni;

		float misWeight = 0.0f;
		if (sampleDelta) {
			misWeight = (0.5f * pdf_delta) / pdf;
		}
		else {
			misWeight = (0.5f * pdf_uni) / pdf;
		}


		Colour bsdfVal = sampleDelta ? baseColor : Colour(0.0f, 0.0f, 0.0f);
		reflectedColour = bsdfVal * misWeight;

		wiLocal.normalize();
		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);

		return wiWorld;
	}

	virtual Colour evaluate(const ShadingData& /*shadingData*/,
		const Vec3& /*wi*/) override
	{
		return Colour(0.0f, 0.0f, 0.0f);
	}

	virtual float PDF(const ShadingData& /*shadingData*/,
		const Vec3& /*wi*/) override
	{
		return 0.0f;
	}

	virtual bool isPureSpecular() override {
		return true;
	}
	virtual bool isTwoSided() override {
		return false;
	}
	virtual float mask(const ShadingData& shadingData) override {
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


	Vec3 sample(const ShadingData& shadingData, Sampler* sampler,
		Colour& reflectedColour, float& pdf) override
	{
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		Vec3 woWorld = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(woWorld);
		if (fabs(woLocal.z) < 1e-6f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}

		float branchSelector = sampler->next();
		Vec3 wiLocal;
		float pdf_mf = 0.f;
		float pdf_uni = 1.f / (4.f * M_PI);

		if (branchSelector < 0.5f) {
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

			float cosThetaI = fabs(I.dot(whLocal));
			float F = fresnelDielectric(cosThetaI, n1, n2);

			float prob = sampler->next();
		Vec3 wiLocal;
			if (TIR) {
				wiLocal = wiReflect;
				F = 1.f;
			}
			else {
				if (prob < F)
					wiLocal = wiReflect;
				else
					wiLocal = wiRefract;
			}
			wiLocal.normalize();

			float cosWH = fabs(I.dot(whLocal));
			float Dval = D_GGX(whLocal, alpha);
			float cosThetaO = fabs(woLocal.z);
			float branch_pdf = (Dval * cosWH) / (4.f * cosThetaO);
			branch_pdf = (prob < F) ? F * branch_pdf : (1.f - F) * branch_pdf;
			pdf_mf = branch_pdf;
		}
		else {
			wiLocal = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
			wiLocal.normalize();
			Vec3 wiWorld_candidate = shadingData.frame.toWorld(wiLocal);
			pdf_mf = PDF(shadingData, wiWorld_candidate);
		}

		float pdf_total = 0.5f * pdf_mf + 0.5f * pdf_uni;
		if (pdf_total < 1e-12f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return shadingData.frame.toWorld(wiLocal);
		}
		pdf = pdf_total;

		Colour fVal = evaluate(shadingData, shadingData.frame.toWorld(wiLocal));
		reflectedColour = fVal / pdf_total;

		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);
		return wiWorld;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wiWorld) override
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
		float cosThetaI = fabs(I.dot(whLocal));

		float n1 = extIOR, n2 = intIOR;
		if (woLocal.z < 0.f) std::swap(n1, n2);
		float F_ = fresnelDielectric(cosThetaI, n1, n2);

		float cosThetaO = fabs(woLocal.z);

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
				fabs((wiLocal.dot(whLocal) * woLocal.dot(whLocal)) / (cosThetaO * cosThetaI * (whLocal.z * whLocal.z)));
		}

		float fSum = fr + ft;
		return baseColor * fSum;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wiWorld) override
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
		float cosThetaI = fabs(I.dot(whLocal));
		float F_ = fresnelDielectric(cosThetaI, n1, n2);
		float cosThetaO = fabs(woLocal.z);

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

	bool isPureSpecular() override
	{
		return false;
	}
	bool isTwoSided() override
	{
		return false;
	}
	float mask(const ShadingData& shadingData) override
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
		: albedo(_albedo), sigma(_sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}

	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler,
		Colour& reflectedColour, float& pdf) override
	{
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(r1, r2);

		float cosTheta = wiLocal.z;
		pdf = (cosTheta > 0.0f) ? (cosTheta / M_PI) : 0.0f;

		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
		reflectedColour = baseColor / M_PI;

		Vec3 wi = shadingData.frame.toWorld(wiLocal);

		return wi;
	}

	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wiWorld) override
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

		float s2 = sigma * sigma;

		float A = 1.0f - 0.5f * (s2 / (s2 + 0.33f));
		float B = 0.45f * (s2 / (s2 + 0.09f));

		float delta = phi_i - phi_r;
		float cosDelta = std::cos(delta);
		float cosDeltaPhi = std::max(0.0f, cosDelta);
		float sinAlpha = std::sin(alpha);
		float tanBeta = std::tan(beta);

		float orenTerm = A + B * cosDeltaPhi * sinAlpha * tanBeta;

		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
		Colour f = baseColor * (orenTerm / M_PI);

		return f;
	}

	virtual float PDF(const ShadingData& shadingData, const Vec3& wiWorld) override
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);
		if (wiLocal.z <= 0.0f)
			return 0.0f;
		return wiLocal.z / M_PI;
	}

	virtual bool isPureSpecular() override
	{
		return false;
	}
	virtual bool isTwoSided() override
	{
		return true;
	}
	virtual float mask(const ShadingData& shadingData) override
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF {
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;

	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
		: albedo(_albedo), intIOR(_intIOR), extIOR(_extIOR) {
		alpha = 1.62142f * sqrtf(roughness);
	}

	float alphaToPhongExponent() const {
		float a = std::max(alpha, 0.001f);
		return (2.0f / (a * a)) - 2.0f;
	}

	float fresnelSchlick(float cosTheta, float F0) const {
		cosTheta = std::fabs(cosTheta);
		return F0 + (1.0f - F0) * std::pow(1.0f - cosTheta, 5.0f);
	}

	void forceAboveSurface(Vec3& v) const {
		if (v.z < 0.0f) {
			v = -v;
		}
	}

	virtual Vec3 sample(const ShadingData& shadingData,
		Sampler* sampler,
		Colour& reflectedColour,
		float& pdf) override
	{
		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);
		float eta = extIOR / intIOR;
		float F0 = (eta - 1.0f) / (eta + 1.0f);
		F0 = F0 * F0;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		forceAboveSurface(woLocal);

		Vec3 woWorld = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(woWorld);
		if (woLocal.z <= 0.f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}
		float cosThetaO = woLocal.z;
		float fresnel = fresnelSchlick(cosThetaO, F0);

		float weightSpec = fresnel;
		float weightDiff = 1.0f - fresnel;

		float randVal = sampler->next();
		bool chooseSpecular = (randVal < weightSpec);

		Vec3 wiLocal;
		float exponent = alphaToPhongExponent();

		if (chooseSpecular) {
			float r1 = sampler->next();
			float r2 = sampler->next();
			float phi = 2.0f * M_PI * r1;
			float cosThetaH = std::pow(r2, 1.0f / (exponent + 1.0f));
			float sinThetaH = std::sqrt(std::max(0.0f, 1.0f - cosThetaH * cosThetaH));
			Vec3 whLocal(sinThetaH * std::cos(phi),
				sinThetaH * std::sin(phi),
				cosThetaH);
			forceAboveSurface(whLocal);
			Vec3 incident = -woLocal;
			float dotIWH = incident.dot(whLocal);
			wiLocal = incident - whLocal * 2.0f * dotIWH;
			forceAboveSurface(wiLocal);
		}
		else {
			float r1 = sampler->next();
			float r2 = sampler->next();
			wiLocal = SamplingDistributions::cosineSampleHemisphere(r1, r2);
			forceAboveSurface(wiLocal);
		}
			diffPdf = (wiLocal.z / M_PI);

		float pdfDiff = wiLocal.z / M_PI;

		Vec3 whLocal = (woLocal + wiLocal).normalize();
		if (whLocal.z < 0.0f)
			whLocal = -whLocal;
		float cosH = std::max(0.0f, whLocal.z);
		float denom = 4.0f * std::fabs(woLocal.dot(whLocal));
		float pdfSpec = 0.0f;
		if (denom > 1e-6f) {
			pdfSpec = ((exponent + 1.0f) / (2.0f * M_PI)) * std::pow(cosH, exponent) / denom;
		}

		float finalPdf = weightSpec * pdfSpec + weightDiff * pdfDiff;

		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);
		Colour fVal = evaluate(shadingData, wiWorld);

		if (finalPdf < 1e-10f) {
			pdf = 0.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
		}
		else {
			pdf = finalPdf;
			reflectedColour = fVal / finalPdf;
		}

		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);
		return wiWorld;
	}

	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wiWorld) override {
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		forceAboveSurface(woLocal);
		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);
		forceAboveSurface(wiLocal);

		if (woLocal.z <= 0.0f || wiLocal.z <= 0.0f)
			return Colour(0.0f, 0.0f, 0.0f);

		Vec3 H = (wiLocal + woLocal).normalize();
		forceAboveSurface(H);

		float cosDH = std::max(0.0f, wiLocal.dot(H));

		float cosThetaO = woLocal.z;
		float eta = extIOR / intIOR;
		float F0 = (eta - 1.0f) / (eta + 1.0f);
		F0 = F0 * F0;

		float fresnel = fresnelSchlick(cosDH, F0);

		Colour baseColor = albedo->sample(shadingData.tu, shadingData.tv);

		Colour diffPart = baseColor * ((1.0f - fresnel) / M_PI);

		float exponent = alphaToPhongExponent();
		float specFactor = (exponent + 2.0f) / (2.0f * M_PI)
			* std::pow(std::max(0.0f, H.z), exponent);
		Colour specPart = Colour(specFactor, specFactor, specFactor) * fresnel;

		return diffPart + specPart;
	}


	virtual float PDF(const ShadingData& shadingData, const Vec3& wiWorld) override {
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		forceAboveSurface(woLocal);

		Vec3 wiLocal = shadingData.frame.toLocal(wiWorld);
		forceAboveSurface(wiLocal);

		if (woLocal.z <= 0.0f || wiLocal.z <= 0.0f)
			return 0.0f;

		float cosThetaO = woLocal.z;
		float eta = extIOR / intIOR;
		float F0 = (eta - 1.0f) / (eta + 1.0f);
		F0 = F0 * F0;
		float F = fresnelSchlick(woLocal.z, F0);

		float pdfDiff = wiLocal.z / M_PI;

		float exponent = alphaToPhongExponent();
		Vec3 H = (wiLocal + woLocal).normalize();
		forceAboveSurface(H);
		float cosH = std::max(0.0f, H.z);
		float denom = 4.0f * std::fabs(woLocal.dot(H));
		float pdfSpec = 0.0f;
		if (denom > 1e-6f)
			pdfSpec = ((exponent + 1.0f) / (2.0f * M_PI)) * std::pow(cosH, exponent) / denom;

		return F * pdfSpec + (1.0f - F) * pdfDiff;
	}

	virtual bool isPureSpecular() override {
		return false;
	}

	virtual bool isTwoSided() override {
		return true;
	}

	virtual float mask(const ShadingData& shadingData) override {
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
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler,
		Colour& reflectedColour, float& pdf) override
	{
		Vec3 wiWorld_refl;
		Colour reflColor_refl;
		float pdf_refl = 0.0f;
		{
			Vec3 candidate = base->sample(shadingData, sampler, reflColor_refl, pdf_refl);
			Vec3 candidateLocal = shadingData.frame.toLocal(candidate);
			if (candidateLocal.z < 0.0f) {
				candidateLocal.z = -candidateLocal.z;
				candidate = shadingData.frame.toWorld(candidateLocal);
			}
			wiWorld_refl = candidate;
		}

		Vec3 wiWorld_trans;
		Colour reflColor_trans;
		float pdf_trans = 0.0f;
		{
			Vec3 candidate = base->sample(shadingData, sampler, reflColor_trans, pdf_trans);
			Vec3 candidateLocal = shadingData.frame.toLocal(candidate);
			if (candidateLocal.z > 0.0f) {
				candidateLocal.z = -candidateLocal.z;
				candidate = shadingData.frame.toWorld(candidateLocal);
			}
			Colour attenuation = Colour::exp(-sigmaa * thickness);
			reflColor_trans = reflColor_trans * attenuation;
			wiWorld_trans = candidate;
		}

		float pdf_refl_eval = base->PDF(shadingData, wiWorld_refl);
		float pdf_trans_eval = base->PDF(shadingData, wiWorld_trans);
		float sumPDF = pdf_refl_eval + pdf_trans_eval + 1e-6f;

		float weight_refl = pdf_refl_eval / sumPDF;
		float weight_trans = pdf_trans_eval / sumPDF;

		float r = sampler->next();
		Vec3 wiWorld_final;
		Colour finalColor;
		float pdf_final;
		if (r < 0.5f) {
			wiWorld_final = wiWorld_refl;
			finalColor = reflColor_refl * weight_refl;
			pdf_final = pdf_refl_eval;
		}
		else {
			wiWorld_final = wiWorld_trans;
			finalColor = reflColor_trans * weight_trans;
			pdf_final = pdf_trans_eval;
		}

		reflectedColour = finalColor;
		pdf = pdf_final;
		return wiWorld_final;
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