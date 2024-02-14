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
    Vector3f L_dir = {0.0, 0.0, 0.0};
    Vector3f L_indir = {0.0, 0.0, 0.0};
    Intersection inter = Scene::intersect(ray);
    if (!inter.happened)
        return L_dir;
    if (inter.m->hasEmission())
    {
        if (depth == 0) {
            return inter.m->getEmission();
        }
        else
            return L_dir;
    }
    Intersection light_pos;
    float pdf_light = 0.0;
    sampleLight(light_pos, pdf_light);

    Vector3f p = inter.coords;
    Vector3f N = normalize(inter.normal);
    Vector3f wo = ray.direction;

    Vector3f xx = light_pos.coords;
    Vector3f NN = normalize(light_pos.normal);
    Vector3f ws = normalize(p - xx);
    float dis = dotProduct(p - xx, p - xx);

    Ray light_to_obj(xx, ws);
    Intersection light_to_scene = Scene::intersect(light_to_obj);
    float dis2 = dotProduct(light_to_scene.coords - xx, light_to_scene.coords - xx);
    if (light_to_scene.happened && dis <= dis2)
    {
        Vector3f L_i = light_pos.emit;
        Vector3f f_r = inter.m->eval(wo, -ws, N);
        float cos_theta = dotProduct(-ws, N);
        float cos_theta_l = dotProduct(ws, NN);
        L_dir = L_i * f_r * cos_theta * cos_theta_l / dis / pdf_light;
    }

    float ksi = get_random_float();//随机取[0,1]
    if (ksi < RussianRoulette) {
        
        Vector3f wi = inter.m->sample(wo, N).normalized();
        Ray r(p, wi);
        Intersection obj_to_scene = Scene::intersect(r);
        //击中了物体&&物体不是光源
        if (obj_to_scene.happened && !obj_to_scene.m->hasEmission()) {
            Vector3f f_r = inter.m->eval(wo, wi, N);//wo不参与计算
            float cos_theta = dotProduct(wi, N);
            float pdf_hemi = inter.m->pdf(wo, wi, N);
            L_indir = castRay(r, depth + 1) * f_r * cos_theta / pdf_hemi / RussianRoulette;
        }
    }
    return L_dir + L_indir;

}
