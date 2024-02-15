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
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
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
    Vector3f L_dir = Vector3f(0.0, 0.0, 0.0);
    Vector3f L_indir = Vector3f(0.0, 0.0, 0.0);
    Intersection point = Scene::intersect(ray);
    if (!point.happened)
        return L_dir;
    Vector3f p = point.coords;
    Vector3f N = point.normal.normalized();
    Vector3f wo = ray.direction;
    Intersection light_pos;
    float pdf = 0.0f;
    sampleLight(light_pos, pdf);
    Vector3f x = light_pos.coords;
    Vector3f NN = light_pos.normal.normalized();
    Vector3f emit = light_pos.emit;
    Vector3f ws = (x - p).normalized();
    Ray r = Ray(p, ws);
    Intersection light_to_obj = Scene::intersect(r);
	float dis = (p - x).norm();
	float dis2 = light_to_obj.distance;
    if (light_to_obj.happened && (dis2 - dis) > -EPSILON)
    {
        L_dir = emit * point.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN)/ pdf / dotProduct(x - p, x - p);
    }

    float P_RR = get_random_float();
    if (P_RR < RussianRoulette)
    {
        Vector3f wi = point.m->sample(wo, N).normalized();
        Ray r1 = Ray(p, wi);
        Intersection obj_to_obj = Scene::intersect(r1);
        if (obj_to_obj.happened && !obj_to_obj.m->hasEmission())
            L_indir = castRay(r1, depth + 1) * point.m->eval(wo, wi, N) * dotProduct(wi, N) / point.m->pdf(wo, wi, N) / RussianRoulette;
    }
    return L_dir + L_indir + point.m->getEmission();
}
