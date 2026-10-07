#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "vector/vector.h"

#define TRUE 1
#define FALSE 0

#define SPHERE 0
#define PLANE 1

#define SAMPLES 64
#define MAX_DEPTH 8

// Window dimensions
const int WIDTH = 1280;
const int HEIGHT = 720;
const float aspect = (float)WIDTH / (float)HEIGHT;

struct Object{
    float color[3];
    float emission[3];
    float albedo;
    int obj_type;
    int emissive;
};

struct Plane{
    struct Object object;
    float normal[3];
    float point[3];
};

// Sphere model data struct
struct Sphere{
    struct Object object;
    float radius;
    float position[3];
};

// Scene objects struct
struct Objects{
    struct Sphere* sphere;
    struct Plane* plane;
    int sphere_count;
    int plane_count;
};

// Vector data struct
struct Vector{
    float position[3];
    float direction[3];
};

float* lambertian(const float* light_dir, const float* normal, const float* sphere_color){
    /* Calculate the Lambertian reflectance for a given light direction and surface normal */

    float dot_product = vector_dot(light_dir, normal, 3);
    float intensity = fmaxf(dot_product, 0.0f);

    return vector_scale(sphere_color, intensity, 3);
}

void intersect_plane(const struct Plane* plane, const struct Vector* ray, float* start_point, float* intersection_point){
    /* Calculate the intersection point between the ray and the plane if there is one */

    float* pl = vector_subtract(plane->point, start_point, 3);
    float first_term = vector_dot(pl, plane->normal, 3);
    float second_term = vector_dot(ray->direction, plane->normal, 3);
    free(pl);

    // If the second term is zero, there is no intersection
    if(fabs(second_term) < 1e-6){
        intersection_point[0] = NAN;
        intersection_point[1] = NAN;
        intersection_point[2] = NAN;
        return;
    }

    float d = first_term / second_term;

    if(d < 0){
        intersection_point[0] = NAN;
        intersection_point[1] = NAN;
        intersection_point[2] = NAN;
        return;
    }

    float* scaled_dir = vector_scale(ray->direction, d, 3);
    float* temp = vector_add(start_point, scaled_dir, 3);
    memcpy(intersection_point, temp, 3 * sizeof(float));

    free(temp);
    free(scaled_dir);
}

void intersect_sphere(const struct Sphere* sphere, const struct Vector* ray, float* start_point, float* intersection_point){
    /* Calculate the intersection point between the camera ray and the sphere if there is one */

    float* oc = vector_subtract(start_point, sphere->position, 3);
    float first_term = vector_dot(ray->direction, oc, 3);
    float second_term = vector_length(oc, 3);
    free(oc);

    float discriminant = first_term * first_term - second_term * second_term + sphere->radius * sphere->radius;

    // From the discriminant we know if there is an intersection or not
    if(discriminant < 0){
        intersection_point[0] = NAN;
        intersection_point[1] = NAN;
        intersection_point[2] = NAN;
        return;
    }

    float d_1 = -first_term + sqrt(discriminant);
    float d_2 = -first_term - sqrt(discriminant);

    float d = (d_1 < d_2) ? d_1 : d_2; // Choose the closest intersection point

    if(d < 0){
        intersection_point[0] = NAN;
        intersection_point[1] = NAN;
        intersection_point[2] = NAN;
        return;
    }

    float* scaled_dir = vector_scale(ray->direction, d, 3);
    float* temp = vector_add(start_point, scaled_dir, 3);
    memcpy(intersection_point, temp, 3 * sizeof(float));

    free(temp);
    free(scaled_dir);
}

int trace_ray(const struct Objects* objects, int object_type, const struct Vector* ray, float* start_point, float* intersection_point, float* intersection_length, int* object_index){
    /* Trace the ray and output the first/shortest hit */

    int object_count = (object_type == SPHERE) ? objects->sphere_count : objects->plane_count;
    int hit_object = FALSE;

    for(int i = 0; i < object_count; ++i){
        float new_intersection_point[3];
        if (object_type == SPHERE){
            intersect_sphere(&objects->sphere[i], ray, start_point, new_intersection_point);
        }
        else{
            intersect_plane(&objects->plane[i], ray, start_point, new_intersection_point);
        }

        if(!isnan(new_intersection_point[0])){ // Got the object
            float* to_hit = vector_subtract(new_intersection_point, start_point, 3);
            float new_intersection_length = vector_length(to_hit, 3);
            free(to_hit);
            if(*intersection_length > new_intersection_length){
                // Update the intersection point and length if this sphere is closer than the previous one
                memcpy(intersection_point, new_intersection_point, 3 * sizeof(float));
                *intersection_length = new_intersection_length;
                *object_index = i;
                hit_object = TRUE;
            }
        }
    }

    if(hit_object == TRUE){
        return 1;
    }

    return 0;
}

struct Object* render_ray(struct Vector* camera, const struct Objects* objects, float* light_dir, float* start_point, float* ray_color, float* intersection_point){
    int object_index = -1;
    struct Object* hit_obj;

    float intersection_length = INFINITY;
    trace_ray(objects, SPHERE, camera, start_point, intersection_point, &intersection_length, &object_index);
    int hit_plane = trace_ray(objects, PLANE, camera, start_point, intersection_point, &intersection_length, &object_index);
    
    if(!isnan(intersection_point[0])){
        // Calculate the normal at the intersection point
        float* normal;
        const float* color;

        // Get object normal vector and color
        if(hit_plane == FALSE){
            float* to_surface = vector_subtract(intersection_point, objects->sphere[object_index].position, 3);
            normal = vector_normalize(to_surface, 3);
            free(to_surface);
            
            hit_obj = &objects->sphere[object_index].object;
            color = hit_obj->color;
        }
        else{
            normal = vector_normalize(objects->plane[object_index].normal, 3);

            hit_obj = &objects->plane[object_index].object;
            color = hit_obj->color;
        }

        if(hit_obj->emissive == TRUE){
            memcpy(ray_color, hit_obj->emission, 3 * sizeof(float));
            free(normal);
            return hit_obj;
        }

        // shadow ray start slightly off the surface and look for an object between the point and the light
        struct Vector shadow_ray;
        memcpy(shadow_ray.direction, light_dir, 3 * sizeof(float));

        float shadow_start[3];
        for(int k = 0; k < 3; ++k){
            shadow_start[k] = intersection_point[k] + normal[k] * 1e-3f;
        }

        float blocker[3];
        float intersection_length = INFINITY;
        int object_index = -1;

        int in_shadow = trace_ray(objects, PLANE, &shadow_ray, shadow_start, blocker, &intersection_length, &object_index);
        // Take spheres into account the same way as planes
        in_shadow |= trace_ray(objects, SPHERE, &shadow_ray, shadow_start, blocker, &intersection_length, &object_index);

        // Ambient makes the shadow not be only black
        const float ambient = 0.15f;
        for(int k = 0; k < 3; ++k){
            ray_color[k] = color[k] * ambient;
        }

        if(in_shadow == FALSE){
            float* lambert_color = lambertian(light_dir, normal, color);
            for(int k = 0; k < 3; ++k){
                ray_color[k] += lambert_color[k] * (1.0f - ambient);
            }
            free(lambert_color);
        }

        free(normal);

        return hit_obj;
    } 
    else {
        // if hits nothing, gets a sky gradient
        float t = fmaxf(camera->direction[1], 0.0f);
        float horizon[3] = {0.90f, 0.93f, 0.97f};
        float zenith[3] = {0.40f, 0.60f, 0.90f};
        for(int k = 0; k < 3; ++k){
            ray_color[k] = horizon[k] * (1.0f - t) + zenith[k] * t;
        }

        return NULL;
    }

}

float* localToGlobal(float* localV, float* normal) {
    /* Transforms the local vector to match a normal vector in global space */
    float tangent[3], bitangent[3];

    if (fabsf(normal[2]) > 0.999f) {
        // if normal points to close to -z
        tangent[0] = 1.0f; 
        tangent[1] = 0.0f; 
        tangent[2] = 0.0f;
    } else {
        // perpendicular vector to normal (rotates 90º)
        float invLen = 1.0f / sqrtf(normal[0] * normal[0] + normal[1] * normal[1]);
        tangent[0] = -normal[1] * invLen;
        tangent[1] = normal[0] * invLen;
        tangent[2] = 0.0f;
    }

    vector_cross(normal, tangent, bitangent);

    // global = x * tangent + y * bitangent + z * normal
    float* globalV = malloc(sizeof(float) * 3);
    for (int k = 0; k < 3; ++k)
        globalV[k] = localV[0] * tangent[k] + localV[1] * bitangent[k] + localV[2] * normal[k];
    return globalV;
}

float* cosine_sample_hemisphere(float a, float b){
    /* Calculate the cosine sample hemisphere direction */

    float theta = acos(sqrt(a));
    float phi = 2.0 * M_PI * b;

    float* vector = malloc(3 * sizeof(float));
    vector[0] = sinf(theta) * cosf(phi);
    vector[1] = sinf(theta) * sinf(phi);
    vector[2] = cosf(theta);

    return vector;
}

void render_pixel(struct Vector* camera, const struct Objects* objects, float* light_dir, float* start_point, float* pixel_color){
    /* Render a single pixel by tracing the ray from the camera through the pixel */

    // Ray loop
    float throughput[3] = { 1.0f, 1.0f, 1.0f };
    float final_color[3] = {0.0f, 0.0f, 0.0f };
    struct Vector ray = *camera;
    float* start_ray = start_point;

    int depth = 0;
    while(depth < MAX_DEPTH){
        // ---------- Get the color for the hit ---------- 
        float intersection_point[3] = {NAN, NAN, NAN};
        float ray_color[3] = {0.0f, 0.0f, 0.0f};
        struct Object* hit_obj = render_ray(&ray, objects, light_dir, start_ray, ray_color, intersection_point);
        
        for (int i = 0; i < 3; ++i){
            final_color[i] += throughput[i] * ray_color[i];
        }

        if(!hit_obj || hit_obj->emissive == TRUE){ 
            break;
        }

        // ---------- Get the next ray direction ---------- 
        // Get the normal based on the type of object
        float normal[3];
        if(hit_obj->obj_type == PLANE){
            struct Plane obj = *(struct Plane*) hit_obj; // go to the object address (same of plane address)
            memcpy(normal, obj.normal, sizeof(normal));
        }
        else {
            struct Sphere obj = *(struct Sphere*) hit_obj;
            float* to_surface = vector_subtract(intersection_point, obj.position, 3);
            float* n = vector_normalize(to_surface, 3);
            memcpy(normal, n, sizeof(normal));
            free(to_surface);
            free(n);
        }

        // Sample from cosine weighted
        float r1 = (float)rand() / RAND_MAX;
        float r2 = (float)rand() / RAND_MAX;
        float* local_dir = cosine_sample_hemisphere(r1, r2);
        float* global_dir = localToGlobal(local_dir, normal);

        for (int i = 0; i < 3; ++i)
            throughput[i] *= hit_obj->albedo * hit_obj->color[i];

        for(int i = 0; i < 3; ++i){
            ray.direction[i] = global_dir[i];
            ray.position[i] = intersection_point[i] + normal[i] * 1e-3;
        }

        start_ray = ray.position;

        free(local_dir);
        free(global_dir);

        // Russian roulette after 2 bounce
        if (depth >= 2) {
            float prob = fmaxf(throughput[0], fmaxf(throughput[1], throughput[2]));
            if (prob > 0.95f)
                prob = 0.95f;

            if ((float)rand() / RAND_MAX > prob) 
                break;

            // Rays that survived become weaker
            for (int i = 0; i < 3; ++i) 
                throughput[i] /= prob;
        }

        depth++;
    }

    for(int k = 0; k < 3; ++k){
        pixel_color[k] = final_color[k];
        if(pixel_color[k] > 1.0f){ pixel_color[k] = 1.0f; }
    }
}

int main(void){
    // Initialize camera at a height of 1.5 above the floor, looking straight ahead so the horizon splits the image
    struct Vector camera = {
        .position = {0.0f, 0.3f, 0.0f},
        .direction = {0.0f, 0.0f, 1.0f}
    };

    // Camera vectors orientation
    float world_up[3] = {0.0f, 1.0f, 0.0f};
    float* right = malloc(sizeof(float) * 3);
    vector_cross(world_up, camera.direction, right);

    float* right_one = vector_normalize(right, 3);
    float* up = malloc(sizeof(float) * 3);
    vector_cross(camera.direction, right_one, up);

    free(right);

    // camera FOV
    const float viewport_half = 0.5f;

    // Two spheres resting on the floor
    struct Sphere spheres[] = {
        { .radius = 0.5f,  .position = {-0.82f, -0.5f,  3.3f}, .object = {.color = {0.3f, 0.4f, 0.80f}, .albedo = 0.7f, .obj_type = SPHERE, .emissive = FALSE} }, // left - blue
        { .radius = 0.53f, .position = { 0.82f, -0.47f, 3.1f}, .object = {.color = {0.85f, 0.25f, 0.30f}, .albedo = 0.7f, .obj_type = SPHERE, .emissive = FALSE} }, // right - red
        { .radius = 0.53f, .position = { 0.0f, 1.2f, 3.0f}, .object = {.color = {1.0f, 1.0f, 1.0f}, .albedo = 0.7f, .obj_type = SPHERE, .emission = {8.0f, 8.0f, 8.0f}, .emissive = TRUE} }, // lamp on top
    };

    // closed box
    struct Plane planes[] = {
        { .normal = { 0.0f, 1.0f,  0.0f}, .point = { 0.0f, -1.0f,  0.0f}, .object = {.color = {0.80f, 0.70f, 0.55f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE} }, // floor
        { .normal = { 0.0f, 0.0f, -1.0f}, .point = { 0.0f,  0.0f,  4.0f}, .object = {.color = {0.80f, 0.70f, 0.55f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE} }, // back wall
        { .normal = { 1.0f, 0.0f,  0.0f}, .point = {-2.0f,  0.0f,  0.0f}, .object = {.color = {0.65f, 0.12f, 0.10f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE} }, // left wall
        { .normal = {-1.0f, 0.0f,  0.0f}, .point = { 2.0f,  0.0f,  0.0f}, .object = {.color = {0.15f, 0.15f, 0.60f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE} }, // right wall
        { .normal = { 0.0f, 0.0f,  1.0f}, .point = { 0.0f,  0.0f, -4.0f}, .object = {.color = {0.80f, 0.70f, 0.55f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE} }, // front wall
        { .normal = { 0.0f, -1.0f, 0.0f}, .point = { 0.0f,  1.0f,  0.0f}, .object = {.color = {0.80f, 0.70f, 0.55f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE} }, // roof
    };

    struct Objects objects;
    objects.sphere = spheres;
    objects.plane = planes;
    objects.sphere_count = sizeof(spheres) / sizeof(spheres[0]);
    objects.plane_count = sizeof(planes) / sizeof(planes[0]);

    FILE *fp = fopen("../analysis/output.csv", "w");

    float light[3] = {0.0f, 1.0f, -0.5f};
    float* light_dir = vector_normalize(light, 3);

    // Render the scene
    for(int j = 0; j < HEIGHT; ++j){
        for(int i = 0; i < WIDTH; ++i){
            // the ray starts at the camera and points through its pixel on the viewport - perspective projection
            float u = ((i + (float)rand()/RAND_MAX) / WIDTH - 0.5f) * 2.0f * viewport_half * aspect;
            float v = ((j + (float)rand()/RAND_MAX) / HEIGHT - 0.5f) * 2.0f * viewport_half;

            float pixel_dir[3];
            float* u_right = vector_scale(right_one, u, 3);
            float* v_up = vector_scale(up, v, 3);
            float* temp = vector_add(camera.direction, u_right, 3);
            float* pixel_dir_temp = vector_add(temp, v_up, 3);
            memcpy(pixel_dir, pixel_dir_temp, 3 * sizeof(float));
            
            free(u_right);
            free(v_up);
            free(temp);
            free(pixel_dir_temp);

            struct Vector ray = camera;
            float* ray_dir = vector_normalize(pixel_dir, 3);
            memcpy(ray.direction, ray_dir, 3 * sizeof(float));
            free(ray_dir);

            float start_point[3];
            memcpy(start_point, camera.position, 3 * sizeof(float));

            float accum[3] = {0.0f, 0.0f, 0.0f};
            for (int s = 0; s < SAMPLES; ++s) {
                float sample[3];
                render_pixel(&ray, &objects, light_dir, start_point, sample);
                for (int k = 0; k < 3; ++k) accum[k] += sample[k];
            }
            float pixel_color[3];
            for (int k = 0; k < 3; ++k) pixel_color[k] = accum[k] / SAMPLES;

            // RGB printing of the viewport for visualization
            fprintf(fp, "(%f,%f,%f)\t", pixel_color[0], pixel_color[1], pixel_color[2]);
        }
        fprintf(fp, "\n");
    }

    fclose(fp);
    free(light_dir);
    free(right_one);
    free(up);
    return 0;
}