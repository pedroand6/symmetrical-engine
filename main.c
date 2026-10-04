#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "vector/vector.h"

// Window dimensions
const int WIDTH = 80;
const int HEIGHT = 60;

// Sphere model data struct
struct Sphere{
    float radius;
    float position[3];
    float color[3];
};

// Camera data struct
struct Camera{
    float position[3];
    float direction[3];
};

void intersect_sphere(const struct Sphere* sphere, const struct Camera* camera, float* start_point, float* intersection_point){
    /* Calculate the intersection point between the camera ray and the sphere if there is one */

    float* oc = vector_subtract(start_point, sphere->position, 3);
    float first_term = vector_dot(camera->direction, oc, 3);
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

    float* scaled_dir = vector_scale(camera->direction, d, 3);
    float* temp = vector_add(start_point, scaled_dir, 3);
    memcpy(intersection_point, temp, 3 * sizeof(float));

    free(temp);
    free(scaled_dir);
}

int main(void){
    // Initialize camera and sphere
    struct Camera camera = {
        .position = {0.0f, 0.0f, 0.0f},
        .direction = {0.0f, 0.0f, 1.0f}
    };

    struct Sphere sphere = {
        .radius = 1.0f,
        .position = {0.0f, 0.0f, 5.0f},
        .color = {1.0f, 0.0f, 0.0f}
    };

    // Render the scene
    for(int j = 0; j < HEIGHT; ++j){
        for(int i = 0; i < WIDTH; ++i){
            float intersection_point[3];
            // Calculate the starting point of the ray for this pixel
            float coordinate[3] = {
                ((float)i / WIDTH  - 0.5f) * 4.0f,
                (0.5f - (float)j / HEIGHT) * 4.0f,
                0.0f
            };

            float* start_point = vector_add(camera.position, coordinate, 3);

            intersect_sphere(&sphere, &camera, start_point, intersection_point);

            // ASCII printing of the viewport for visualization
            if(!isnan(intersection_point[0])){
                printf("*");
            } else {
                printf(" ");
            }
        }
        printf("\n");
    }

    return 0;
}