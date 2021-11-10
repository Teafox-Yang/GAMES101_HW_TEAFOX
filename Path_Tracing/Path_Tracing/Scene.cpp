//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
	printf(" - Generating BVH...\n\n");
	this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
	return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
	float emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	float p = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
			if (p <= emit_area_sum) {
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(
	const Ray &ray,
	const std::vector<Object*> &objects,
	float &tNear, uint32_t &index, Object **hitObject)
{
	*hitObject = nullptr;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		float tNearK = kInfinity;
		uint32_t indexK;
		Vector2f uvK;
		if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
			*hitObject = objects[k];
			tNear = tNearK;
			index = indexK;
		}
	}


	return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
	// TO DO Implement Path Tracing Algorithm here
	Vector3f L_dir(0, 0, 0), L_indir(0, 0, 0);
	float pdf_light;
	Intersection inter, inter_light;
	inter = intersect(ray);
	if (inter.happened)
	{
		if (inter.m->hasEmission())
		{
			if (depth == 0)
			{
				return inter.m->getEmission();//第一次直接打到光源
			}
			else return Vector3f(0, 0, 0);
		}
		sampleLight(inter_light, pdf_light);
		Vector3f emit = inter.emit;
		Vector3f p = inter_light.coords;//光源采样点
		Vector3f x = inter.coords;//着色点
		Vector3f N = inter.normal;//着色点法线
		Vector3f NN = inter_light.normal;//光源法线
		Material *m = inter.m;//材质
		Vector3f wo = ray.direction;
		Vector3f dis = p - x;
		float distance2 = dis.x * dis.x + dis.y * dis.y + dis.z * dis.z;
		Vector3f ws = dis.normalized();
		Ray light(x, ws);
		Intersection toLight = intersect(light);
		//反射击中光源，只计算直接光照
		if (toLight.happened)
		{
			Vector3f f_r = m->eval(wo, ws, N);
			L_dir = inter_light.emit * f_r * dotProduct(ws, N) * dotProduct(-ws, NN) / distance2 / pdf_light;
		}
		if (get_random_float() < RussianRoulette)
		{
			Vector3f nextSample = inter.m->sample(ray.direction, N).normalized();
			Ray nextRay(x, nextSample);
			Intersection nextInter = intersect(nextRay);
			if (nextInter.happened && !nextInter.m->hasEmission())
			{
				float pdf = inter.m->pdf(ray.direction, nextSample, N);
				Vector3f f_r = inter.m->eval(ray.direction, nextSample, N);
				L_indir = castRay(nextRay, depth + 1) * f_r * dotProduct(nextSample, N) / pdf / RussianRoulette;
			}
		}
	}
	return L_dir + L_indir;
}