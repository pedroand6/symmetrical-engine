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

// Window dimensions
const int WIDTH = 1280;
const int HEIGHT = 720;
const float aspect = (float)WIDTH / (float)HEIGHT;

struct Plane{
    float normal[3];
    float point[3];
    float color[3];
};

// Sphere model data struct
struct Sphere{
    float radius;
    float position[3];
    float color[3];
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

void render_pixel(const struct Vector* camera, const struct Objects* objects, float* light_dir, float* start_point, float* pixel_color){
    int object_index = -1;

    float intersection_point[3] = {NAN, NAN, NAN};
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
            color = objects->sphere[object_index].color;
        }
        else{
            normal = vector_normalize(objects->plane[object_index].normal, 3);
            color = objects->plane[object_index].color;
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
        in_shadow |= trace_ray(objects, SPHERE, &shadow_ray, shadow_start, blocker, &intersection_length, &object_index);

        // Ambient term so shadowed areas are not pure black
        const float ambient = 0.15f;
        for(int k = 0; k < 3; ++k){
            pixel_color[k] = color[k] * ambient;
        }

        if(in_shadow == FALSE){
            float* lambert_color = lambertian(light_dir, normal, color);
            for(int k = 0; k < 3; ++k){
                pixel_color[k] += lambert_color[k] * (1.0f - ambient);
            }
            free(lambert_color);
        }

        free(normal);
    } else {
        // if hits nothing, gets a sky gradient
        float t = fmaxf(camera->direction[1], 0.0f);
        float horizon[3] = {0.90f, 0.93f, 0.97f};
        float zenith[3] = {0.40f, 0.60f, 0.90f};
        for(int k = 0; k < 3; ++k){
            pixel_color[k] = horizon[k] * (1.0f - t) + zenith[k] * t;
        }
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
    float* right = vector_cross(world_up, camera.direction);
    float* right_one = vector_normalize(right, 3);
    float* up = vector_cross(camera.direction, right_one);
    free(right);

    // camera FOV
    const float viewport_half = 0.5f;

    // Two spheres resting on the floor
    struct Sphere spheres[] = {
        { .radius = 0.5f,  .position = {-0.82f, -0.5f,  3.3f}, .color = {0.85f, 0.25f, 0.30f} }, // left
        { .radius = 0.53f, .position = { 0.82f, -0.47f, 3.1f}, .color = {0.3f, 0.4f, 0.80f} }, // right
        { .radius = 0.53f, .position = { 0.0f, 1.2f, 3.0f}, .color = {1.0f, 1.0f, 1.0f} }, // lamp on top
    };

    // closed box
    struct Plane planes[] = {
        { .normal = { 0.0f, 1.0f,  0.0f}, .point = { 0.0f, -1.0f, 0.0f}, .color = {0.80f, 0.70f, 0.55f} }, // floor
        { .normal = { 0.0f, 0.0f, -1.0f}, .point = { 0.0f,  0.0f, 4.0f}, .color = {0.80f, 0.70f, 0.55f} }, // back wall
        { .normal = { 1.0f, 0.0f,  0.0f}, .point = {-2.0f,  0.0f, 0.0f}, .color = {0.65f, 0.12f, 0.10f} }, // left wall
        { .normal = {-1.0f, 0.0f,  0.0f}, .point = { 2.0f,  0.0f, 0.0f}, .color = {0.15f, 0.15f, 0.60f} }, // right wall
        { .normal = { 0.0f, -1.0f,  0.0f}, .point = { 0.0f, 1.0f, 0.0f}, .color = {0.80f, 0.70f, 0.55f} }, // roof
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
            float u = ((float)i / WIDTH  - 0.5f) * 2.0f * viewport_half * aspect;
            float v = ((float)j / HEIGHT - 0.5f) * 2.0f * viewport_half;

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

            float pixel_color[3];
            render_pixel(&ray, &objects, light_dir, start_point, pixel_color); //render objects

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