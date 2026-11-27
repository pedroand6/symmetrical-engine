// OpenMP header
#include <omp.h>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include "vector/vector.h"

#define TRUE 1
#define FALSE 0

#define SPHERE 0
#define PLANE 1

#define OPAQUE 0
#define MIRROR -1

#define MAX_DEPTH 24
#define MAX_RAYS 100

#define TILE 8

// Window dimensions
int SAMPLES = 1;
const int WIDTH = 1280;
const int HEIGHT = 720;
const float aspect = (float)WIDTH / (float)HEIGHT;

struct Object{
    float color[3];
    float emission[3];
    float albedo;
    float refrac;
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

struct Ray{
    struct Vector vector;
    float throughput[3];
    int depth;
};

struct RayStack{
    struct Ray rays[MAX_RAYS];
    int count;
};

void lambertian(const float* light_dir, const float* normal, const float* sphere_color, float* result){
    /* Calculate the Lambertian reflectance for a given light direction and surface normal */

    float dot_product = vector_dot(light_dir, normal, 3);
    float intensity = fmaxf(dot_product, 0.0f);

    vector_scale(sphere_color, intensity, 3, result);
}

void intersect_plane(const struct Plane* plane, const struct Vector* ray, float* start_point, float* intersection_point){
    /* Calculate the intersection point between the ray and the plane if there is one */

    float pl[3];
    vector_subtract(plane->point, start_point, 3, pl);
    
    float first_term = vector_dot(pl, plane->normal, 3);
    float second_term = vector_dot(ray->direction, plane->normal, 3);

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

    float scaled_dir[3];
    vector_scale(ray->direction, d, 3, scaled_dir);
    vector_add(start_point, scaled_dir, 3, intersection_point);
}

void intersect_sphere(const struct Sphere* sphere, const struct Vector* ray, float* start_point, float* intersection_point){
    /* Calculate the intersection point between the camera ray and the sphere if there is one */

    float oc[3];
    vector_subtract(start_point, sphere->position, 3, oc);
    float first_term = vector_dot(ray->direction, oc, 3);
    float second_term = vector_length(oc, 3);

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

    float d = d_2 < 1e-4f ? d_1 : d_2; // Hit point behind the starting point - ray inside sphere / outside

    if(d < 1e-4f){
        intersection_point[0] = NAN;
        intersection_point[1] = NAN;
        intersection_point[2] = NAN;
        return;
    }

    float scaled_dir[3];
    vector_scale(ray->direction, d, 3, scaled_dir);
    vector_add(start_point, scaled_dir, 3, intersection_point);
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
            float to_hit[3];
            vector_subtract(new_intersection_point, start_point, 3, to_hit);
            float new_intersection_length = vector_length(to_hit, 3);
            
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

struct Object* render_ray(struct Vector* ray_vec, const struct Objects* objects, float* light_dir, float* start_point, float* ray_color, float* intersection_point){
    /* This trace a ray and calculate its intersections, return the object from the closest hit. */

    int object_index = -1;
    struct Object* hit_obj;

    float intersection_length = INFINITY;
    trace_ray(objects, SPHERE, ray_vec, start_point, intersection_point, &intersection_length, &object_index);
    int hit_plane = trace_ray(objects, PLANE, ray_vec, start_point, intersection_point, &intersection_length, &object_index);
    
    if(!isnan(intersection_point[0])){
        // Calculate the normal at the intersection point
        float normal[3];
        const float* color;

        // Get object normal vector and color
        if(hit_plane == FALSE){
            float to_surface[3];
            vector_subtract(intersection_point, objects->sphere[object_index].position, 3, to_surface);
            vector_normalize(to_surface, 3, normal);
            
            hit_obj = &objects->sphere[object_index].object;
            color = hit_obj->color;
        }
        else{
            vector_normalize(objects->plane[object_index].normal, 3, normal);

            hit_obj = &objects->plane[object_index].object;
            color = hit_obj->color;
        }

        if(hit_obj->emissive == TRUE){
            memcpy(ray_color, hit_obj->emission, 3 * sizeof(float));
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
            float lambert_color[3];
            lambertian(light_dir, normal, color, lambert_color);
            for(int k = 0; k < 3; ++k){
                ray_color[k] += lambert_color[k] * (1.0f - ambient);
            }
        }

        return hit_obj;
    } 
    else {
        // if hits nothing, gets a sky gradient
        float t = fmaxf(ray_vec->direction[1], 0.0f);
        float horizon[3] = {0.90f, 0.93f, 0.97f};
        float zenith[3] = {0.40f, 0.60f, 0.90f};
        for(int k = 0; k < 3; ++k){
            ray_color[k] = horizon[k] * (1.0f - t) + zenith[k] * t;
        }

        return NULL;
    }

}

void localToGlobal(float* localV, float* normal, float* result){
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
    for (int k = 0; k < 3; ++k)
        result[k] = localV[0] * tangent[k] + localV[1] * bitangent[k] + localV[2] * normal[k];
}

void cosine_sample_hemisphere(float a, float b, float* result){
    /* Calculate the cosine sample hemisphere direction */

    float theta = acos(sqrt(a));
    float phi = 2.0 * M_PI * b;

    result[0] = sinf(theta) * cosf(phi);
    result[1] = sinf(theta) * sinf(phi);
    result[2] = cosf(theta);
}

void get_ray(struct RayStack* ray_stack, struct Ray* ray, unsigned int* seed){
    if(ray->depth >= MAX_DEPTH || ray_stack->count >= MAX_RAYS) return;

    // Russian roulette after 2 bounce
    if (ray->depth >= 2) {
        float prob = fmaxf(ray->throughput[0], fmaxf(ray->throughput[1], ray->throughput[2]));
        if (prob > 0.95f)
            prob = 0.95f;

        if ((float)rand_r(seed) / RAND_MAX > prob)
            return;

        for (int i = 0; i < 3; ++i) 
            ray->throughput[i] /= prob; // multiplies the surviving rays so we dont lose energy
    }

    ray_stack->rays[ray_stack->count++] = *ray;
}

void render_pixel(struct Vector* camera, const struct Objects* objects, float* light_dir, float* pixel_color, unsigned int* seed){
    /* Render a single pixel by tracing the ray from the camera through the pixel */

    struct RayStack ray_stack; // Use a ray stack so we can divide the rays, etc

    // Ray loop
    struct Ray first_ray = {
        .throughput = { 1.0f, 1.0f, 1.0f },
        .vector = *camera,
        .depth = 0
    };

    float final_color[3] = { 0.0f, 0.0f, 0.0f };
    
    ray_stack.rays[0] = first_ray;
    ray_stack.count = 1;

    while( ray_stack.count > 0 ){
        // ---------- Get the color for the hit ---------- 
        float intersection_point[3] = {NAN, NAN, NAN};
        float ray_color[3] = {0.0f, 0.0f, 0.0f};

        struct Ray ray = ray_stack.rays[--ray_stack.count];

        struct Object* hit_obj = render_ray(&ray.vector, objects, light_dir, ray.vector.position, ray_color, intersection_point);

        // Hit nothing or emissive object
        if(!hit_obj || hit_obj->emissive == TRUE){
            for(int i = 0; i < 3; ++i)
                final_color[i] += ray.throughput[i] * ray_color[i];

            continue;
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
            float to_surface[3];
            vector_subtract(intersection_point, obj.position, 3, to_surface);
            vector_normalize(to_surface, 3, normal);
        }

        struct Ray next_ray = ray;
        next_ray.depth = ray.depth + 1;

        if(hit_obj->refrac == MIRROR){
            // if it is mirror then we apply the full reflection
            for(int i = 0; i < 3; ++i){
                next_ray.vector.direction[i] = ray.vector.direction[i] - 2*(vector_dot(ray.vector.direction, normal, 3)) * normal[i];
                next_ray.vector.position[i] = intersection_point[i] + normal[i] * 1e-3;
            }

            get_ray(&ray_stack, &next_ray, seed);
        }
        else if(hit_obj->refrac == OPAQUE){
            // if it is opaque then we apply the Lambertian
            // Sample from cosine weighted
            float r1 = (float)rand_r(seed) / RAND_MAX;
            float r2 = (float)rand_r(seed) / RAND_MAX;

            float local_dir[3];
            cosine_sample_hemisphere(r1, r2, local_dir);
            float global_dir[3];
            localToGlobal(local_dir, normal, global_dir);

            for (int i = 0; i < 3; ++i){
                next_ray.throughput[i] *= hit_obj->albedo * hit_obj->color[i];
                final_color[i] += ray.throughput[i] * ray_color[i]; // Lambertian

                next_ray.vector.direction[i] = global_dir[i];
                next_ray.vector.position[i] = intersection_point[i] + normal[i] * 1e-3;
            }

            get_ray(&ray_stack, &next_ray, seed);
        }
        else{
            // refraction
            float n1 = 1.0f;
            float n2 = hit_obj->refrac;
            float cos_i = -vector_dot(ray.vector.direction, normal, 3);

            if(cos_i < 0.0f){ // Leaving object
                n2 = 1.0f;
                n1 = hit_obj->refrac;
                cos_i = -cos_i;
                
                for(int i = 0; i < 3; ++i)
                    normal[i] = -normal[i];
            }

            float eta = n1 / n2;

            float r_0 = pow((n1 - n2) / (n1 + n2), 2);
            float r = r_0 + (1 - r_0) * pow((1 - cos_i), 5); // schlick approximation for reflection contribution

            float k = 1.0f - eta * eta * (1.0f - cos_i * cos_i);

            if(k < 0.0f)
                r = 1.0f; // total internal reflection

            // ---- create reflected ray ----
            float mirror_dir[3];
            for(int i = 0; i < 3; ++i){
                mirror_dir[i] = ray.vector.direction[i] + 2.0f*cos_i * normal[i];
            }

            float new_throughput[3];
            for (int i = 0; i < 3; ++i)
                new_throughput[i] = ray.throughput[i] * r;

            struct Ray reflected_ray = {
                .throughput = {
                    new_throughput[0],
                    new_throughput[1],
                    new_throughput[2]
                },
                .depth = next_ray.depth,
            };

            for(int i = 0; i < 3; ++i){
                reflected_ray.vector.direction[i] = mirror_dir[i];
                reflected_ray.vector.position[i] = intersection_point[i] + normal[i] * 1e-3;
            }

            get_ray(&ray_stack, &reflected_ray, seed);

            // ---- redirect this as refracted ray ----
            // rotate the vector
            if(k >= 0){ // entering object
                float cos_t = sqrtf(k);
                for (int i = 0; i < 3; ++i){
                    next_ray.vector.direction[i] = eta * ray.vector.direction[i] + (eta * cos_i - cos_t) * normal[i];
                    next_ray.vector.position[i] = intersection_point[i] - normal[i] * 1e-3;
                }

                for (int i = 0; i < 3; ++i)
                    next_ray.throughput[i] *= (1 - r) * hit_obj->color[i];

                get_ray(&ray_stack, &next_ray, seed);
            }
        }
    }

    for(int k = 0; k < 3; ++k){
        pixel_color[k] = final_color[k];
    }
}

static int compare_floats(const void* a, const void* b){
    /* Compare the value of two floats and returns
        1 if a is bigger than b or 0 if the opposite */

    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

void write_image(const char* path, float* image){
    /* Writes the image from the rgb array to a .csv file */

    FILE *fp = fopen(path, "w");

    for(int j = 0; j < HEIGHT; ++j){
        for(int i = 0; i < WIDTH; ++i){
            // RGB printing of the viewport for visualization
            int n = (j * WIDTH + i) * 3;
            fprintf(fp, "(%f,%f,%f)\t", image[n], image[n+1], image[n+2]);
        }
        fprintf(fp, "\n");
    }

    fclose(fp);
}

int main(void){
    // Read user inputs
    printf("Insira a quantidade de Samples por pixel: ");
    fflush(stdout);
    scanf(" %d", &SAMPLES);

    printf("Insira o caminho de saída da imagem: ");
    fflush(stdout);
    char output_path[256];
    scanf(" %255s", output_path);

    printf("Insira o número de threads: ");
    fflush(stdout);
    int nthreads;
    scanf(" %d", &nthreads);

    // Initialize camera at a height of 1.5 above the floor, looking straight ahead so the horizon splits the image
    struct Vector camera = {
        .position = {0.0f, 0.3f, 0.0f},
        .direction = {0.0f, 0.0f, 1.0f}
    };

    // Camera vectors orientation
    float world_up[3] = {0.0f, 1.0f, 0.0f};

    float right[3];
    vector_cross(world_up, camera.direction, right);

    float right_one[3];
    vector_normalize(right, 3, right_one);

    float up[3];
    vector_cross(camera.direction, right_one, up);

    // camera FOV
    const float viewport_half = 0.5f;

    // Two spheres resting on the floor
    struct Sphere spheres[] = {
        { .radius = 0.5f,  .position = {-0.82f, -0.5f,  3.3f}, .object = {.color = {0.3f, 0.4f, 0.80f}, .albedo = 0.7f, .obj_type = SPHERE, .emissive = FALSE, .refrac = OPAQUE} }, // left - blue
        { .radius = 0.53f, .position = { 0.82f, -0.47f, 3.1f}, .object = {.color = {0.85f, 0.25f, 0.30f}, .albedo = 0.7f, .obj_type = SPHERE, .emissive = FALSE, .refrac = OPAQUE} }, // right - red
        { .radius = 0.5f,  .position = {-1.0f, -0.5f,  2.5f}, .object = {.color = {1.0f, 1.0f, 1.0f}, .albedo = 0.7f, .obj_type = SPHERE, .emissive = FALSE, .refrac = 1.5} }, // left - blue
        { .radius = 0.53f, .position = { 1.0f, -0.47f, 2.5f}, .object = {.color = {0.0f, 0.0f, 0.0f}, .albedo = 0.7f, .obj_type = SPHERE, .emissive = FALSE, .refrac = MIRROR} }, // right - red
        { .radius = 0.53f, .position = { 0.0f, 1.2f, 3.0f}, .object = {.color = {1.0f, 1.0f, 1.0f}, .albedo = 0.7f, .obj_type = SPHERE, .emission = {12.0f, 12.0f, 12.0f}, .emissive = TRUE, .refrac = OPAQUE} }, // lamp on top
    };

    // closed box
    struct Plane planes[] = {
        { .normal = { 0.0f, 1.0f,  0.0f}, .point = { 0.0f, -1.0f,  0.0f}, .object = {.color = {0.80f, 0.70f, 0.55f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE, .refrac = OPAQUE} }, // floor
        { .normal = { 0.0f, 0.0f, -1.0f}, .point = { 0.0f,  0.0f,  4.0f}, .object = {.color = {0.80f, 0.70f, 0.55f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE, .refrac = OPAQUE} }, // back wall
        { .normal = { 1.0f, 0.0f,  0.0f}, .point = {-2.0f,  0.0f,  0.0f}, .object = {.color = {0.4f, 0.0f, 0.0f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE, .refrac = OPAQUE} }, // left wall
        { .normal = {-1.0f, 0.0f,  0.0f}, .point = { 2.0f,  0.0f,  0.0f}, .object = {.color = {0.15f, 0.15f, 0.60f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE, .refrac = OPAQUE} }, // right wall
        { .normal = { 0.0f, 0.0f,  1.0f}, .point = { 0.0f,  0.0f, -4.0f}, .object = {.color = {0.80f, 0.70f, 0.55f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE, .refrac = OPAQUE} }, // front wall
        { .normal = { 0.0f, -1.0f, 0.0f}, .point = { 0.0f,  1.0f,  0.0f}, .object = {.color = {0.80f, 0.70f, 0.55f}, .albedo = 0.7f, .obj_type = PLANE, .emissive = FALSE, .refrac = OPAQUE} }, // roof
    };

    struct Objects objects;
    objects.sphere = spheres;
    objects.plane = planes;
    objects.sphere_count = sizeof(spheres) / sizeof(spheres[0]);
    objects.plane_count = sizeof(planes) / sizeof(planes[0]);

    size_t N = HEIGHT * WIDTH;
    float* image = malloc((size_t)N * 3 * sizeof(float));

    float light[3] = {0.0f, 1.0f, -0.5f};
    float light_dir[3];
    vector_normalize(light, 3, light_dir);

    uint base_seed = (unsigned int)time(NULL) ^ 19937;

    const int tiles_x = (WIDTH  + TILE - 1) / TILE;
    const int tiles_y = (HEIGHT + TILE - 1) / TILE;
    const int total_tiles = tiles_x * tiles_y;

    // Render the scene in parallel using OpenMP
    #pragma omp parallel num_threads(nthreads)
    {
        // variables for each thread
        float (*samples)[3] = malloc((size_t)SAMPLES * sizeof(*samples)); // accumulate and get the mean of the rays for each pixel
        float *lum = malloc((size_t)SAMPLES * sizeof(float)); // brightness of each sample
        float *sorted = malloc((size_t)SAMPLES * sizeof(float)); // median brightness of this pixels samples

        // Paralellize in "tiles" of 8x8 pixels
        #pragma omp for schedule(dynamic, 1)
        for (int tile = 0; tile < total_tiles; ++tile) {
            int ty = (tile / tiles_x) * TILE;
            int tx = (tile % tiles_x) * TILE;
            int j_end = (ty + TILE < HEIGHT) ? ty + TILE : HEIGHT;
            int i_end = (tx + TILE < WIDTH) ? tx + TILE : WIDTH;

            for (int j = ty; j < j_end; ++j) {
                for (int i = tx; i < i_end; ++i) {
                    uint seed = (j * WIDTH + i) ^ base_seed;
                    
                    for (int s = 0; s < SAMPLES; ++s) {
                        // the ray starts at the camera and points through its pixel on the viewport - perspective projection
                        // jitter a little the direction for each sample so we can get a anti-aliasing effect
                        float u = ((i + (float)rand_r(&seed)/RAND_MAX) / WIDTH - 0.5f) * 2.0f * viewport_half * aspect;
                        float v = ((j + (float)rand_r(&seed)/RAND_MAX) / HEIGHT - 0.5f) * 2.0f * viewport_half;

                        float pixel_dir[3];
                        float u_right[3];
                        float v_up[3];
                        float temp[3];

                        vector_scale(right_one, u, 3, u_right);
                        vector_scale(up, v, 3, v_up);
                        vector_add(camera.direction, u_right, 3, temp);
                        vector_add(temp, v_up, 3, pixel_dir);

                        struct Vector ray = camera;

                        vector_normalize(pixel_dir, 3, ray.direction);

                        float start_point[3];
                        memcpy(start_point, camera.position, 3 * sizeof(float));

                        // Render the sample
                        render_pixel(&ray, &objects, light_dir, samples[s], &seed);
                        lum[s] = 0.2126f * samples[s][0] + 0.7152f * samples[s][1] + 0.0722f * samples[s][2];
                    }

                    memcpy(sorted, lum, (size_t)SAMPLES * sizeof(float));
                    qsort(sorted, SAMPLES, sizeof(float), compare_floats);
                    float median = sorted[SAMPLES / 2];

                    // a sample is an outlier if its much brighter than the median
                    const float OUTLIER_FACTOR = 4.0f;
                    const float MIN_THRESHOLD = 1.0f;
                    float threshold = fmaxf(OUTLIER_FACTOR * median, MIN_THRESHOLD);

                    float accum[3] = {0.0f, 0.0f, 0.0f};
                    for (int s = 0; s < SAMPLES; ++s) {
                        // scale outliers down to the threshold, keeping their color
                        float scale = (lum[s] > threshold) ? threshold / lum[s] : 1.0f;
                        for (int k = 0; k < 3; ++k)
                            accum[k] += samples[s][k] * scale;
                    }
                    
                    int n = (j * WIDTH + i) * 3;
                    for (int k = 0; k < 3; ++k) {
                        image[n + k] = fminf(accum[k] / SAMPLES, 1.0f);
                    }
                }
            }
        }

        free(sorted);
        free(lum);
        free(samples);
    }

    write_image(output_path, image);

    free(image);
    return 0;
}