#include "scene.h"
#include <iostream>

// intersection record
struct intersection3f {
    bool        hit;        // whether it hits something
    float       ray_t;      // ray parameter for the hit
    vec3f       pos;        // hit position
    vec3f       norm;       // hit normal
    Material*   mat;        // hit material

    // constructor (defaults to no intersection)
    intersection3f() : hit(false) { }
};

#define ray3f_epsilon 0.0005f
#define ray3f_rayinf 1000000.0f

// 3D Ray
struct ray3f {
    vec3f e;        // origin
    vec3f d;        // direction
    float tmin;     // min t value
    float tmax;     // max t value

    // Default constructor
    ray3f() : e(zero3f), d(z3f), tmin(ray3f_epsilon), tmax(ray3f_rayinf) { }

    // Element-wise constructor
    ray3f(const vec3f& e, const vec3f& d) :
    e(e), d(d), tmin(ray3f_epsilon), tmax(ray3f_rayinf) { }

    // Element-wise constructor
    ray3f(const vec3f& e, const vec3f& d, float tmin, float tmax) :
    e(e), d(d), tmin(tmin), tmax(tmax) { }

    // Eval ray at a specific t
    vec3f eval(float t) const { return e + d * t; }

    // Create a ray from a segment
    static ray3f make_segment(const vec3f& a, const vec3f& b) { return ray3f(a,normalize(b-a),ray3f_epsilon,dist(a,b)-2*ray3f_epsilon); }
};

// transform a ray by a frame
inline ray3f transform_ray(const frame3f& f, const ray3f& v) {
    return ray3f(transform_point(f,v.e), transform_vector(f,v.d), v.tmin, v.tmax);
}
// transform a ray by a frame inverse
inline ray3f transform_ray_inverse(const frame3f& f, const ray3f& v) {
    return ray3f(transform_point_inverse(f,v.e),transform_vector_inverse(f,v.d),v.tmin,v.tmax);
}


// intersects the scene and return the first intrerseciton
intersection3f intersect(Scene* scene, ray3f ray) {


    // create a default intersection record to be returned
    auto intersection = intersection3f();
    intersection.ray_t = ray3f_rayinf;


    for(Surface *object : scene->surfaces){

        // un transform ray into the object's frame
        ray3f nRay = transform_ray_inverse(object->frame, ray);


        if(object->isquad){

            // check to see if on surface
            // make sure the projection of the normal to the direction is not 0
            if( dot(z3f, nRay.d) != 0){

                // find t
                // equation given: ((C-p) dot n) / d dot n
                // C = zero because center is at the origin
                // normal is the z vector of the frame
                float t = (dot(-nRay.e, z3f)/dot(z3f, nRay.d));

                // find point where t intersects the plane
                vec3f xRay = nRay.eval(t);

                // check to see if point is in the radius (bound the box)
                if ( abs(xRay.x) < object->radius && abs(xRay.y) < object->radius){

                    // check to see if t is in the range
                    if ( t > ray.tmin && t < ray.tmax){

                        // check to see if it is the closest thing
                        if ( t < intersection.ray_t || !intersection.hit){

                            // update intersection
                            intersection.pos = ray.eval(t);
                            intersection.hit = true;
                            intersection.ray_t = t;
                            intersection.mat = object->mat;
                            intersection.norm = object->frame.z;
                        }

                    }
                }
            }
        }

        // else if it is a cylinder

        else if (object->iscyl){
            // find intersection

            float a = nRay.d.x * nRay.d.x + (nRay.d.z * nRay.d.z);
            float b = 2 * (nRay.d.x * nRay.e.x) + 2 * (nRay.d.z * nRay.e.z);
            float c = ((nRay.e.x * nRay.e.x) - (object->radius * object->radius)) + ((nRay.e.z * nRay.e.z) - (object->radius * object->radius));



            // calc determinant
            float det = (b * b) - (4 * a * c);

            // intersection only if the det is non negative ( det = 0 is a tangent )
            if ( det >= 0 ){

                // find t if there is an intersection
                float t = ((-1 * b) - sqrt(det))/(2*a);
                float t1 = ((-1 *b) + sqrt(det)/(2*a));


                // find the y values for the intersections
                float y = nRay.e.y + t * nRay.d.y;
                float y1 = nRay.e.y + t1 * nRay.d.y;

                // check to see if t is in the range
                if ( t > nRay.tmin && t < nRay.tmax){

                    // check to see if t is within bound of sphere
                    // find point where t intersects the plane
                    vec3f xRay = nRay.eval(t);

                    // check to see if point is in the radius (bound the box)
                    if ( abs(xRay.x) < object->radius && abs(xRay.y) < object->radius){

                    // check to see if it is the closest thing
                    if ( t < intersection.ray_t || !intersection.hit){

                        // update intersection
                        intersection.pos = ray.eval(t);
                        intersection.hit = true;
                        intersection.ray_t = t;
                        intersection.mat = object->mat;
                        intersection.norm = (ray.eval(t) - object->frame.o)/object->radius;
                    }
                   }
                }
            }
        }


        // else  if it is a sphere
        else{

           // make circle variables
           // use the det function given in lecture slides 4
           float a = dot(nRay.d, nRay.d);
           float b = 2 * dot(nRay.d, nRay.e);
           float c = dot(nRay.e, nRay.e) - (object->radius * object->radius);

           // calc determinant
           float det = (b * b) - (4 * a * c);

           // intersection only if the det is non negative ( det = 0 is a tangent )
           if ( det >= 0 ){

               // find t if there is an intersection
               float t = ((-1 * b) - sqrt(det))/(2*a);

               // check to see if t is in the range
               if ( t > nRay.tmin && t < nRay.tmax){

                   // check to see if it is the closest thing
                   if ( t < intersection.ray_t || !intersection.hit){

                       // update intersection
                       intersection.pos = ray.eval(t);
                       intersection.hit = true;
                       intersection.ray_t = t;
                       intersection.mat = object->mat;
                       intersection.norm = (ray.eval(t) - object->frame.o)/object->radius;
                   }

               }
           }

        }

    // record closest intersection
    }

    return intersection;
}



// compute the color corresponding to a ray by raytracing
vec3f raytrace_ray(Scene* scene, ray3f ray) {

    // create a vector to hold the color
    vec3f color = zero3f;
    // get the closes shape to this point
    intersection3f shape = intersect(scene, ray);

    // if we didn't intersect anything
    if (!shape.hit){
        // return background
        return scene->background;
    }


    else{
    // accumulate color starting with ambient
        // ambient color = ka * Ia
        color = shape.mat->kd * scene->ambient;
        for( Light *light : scene->lights){
            // get the riemann sum of the lights

            // compute light response
            vec3f I = light->intensity/(lengthSqr(light->frame.o - shape.pos));

            // compute light direction
            vec3f ld = normalize(light->frame.o - shape.pos);

            // get h
            // h = norm of light direction and direction from ray
            vec3f vd = normalize(ray.e - shape.pos);
            vec3f h = normalize(ld + vd);


            // compute the material response (brdf*cos)
            // Sum of Ld + Ls
            vec3f brdf = shape.mat->kd + shape.mat->ks* pow(max(0.0, dot(shape.norm, h)), shape.mat->n);
            vec3f mat_res = I * brdf * max(0.0, dot(shape.norm, ld));

            // check for shadows and accumulate if needed
            // create a shadow ray using position of the intersection point and the lighting direction
            // ray must be bounded = account for epsilon value, and teh max value located light.
            ray3f shadowRay = ray3f(shape.pos, ld, ray3f_epsilon, ray3f_rayinf);
            intersection3f shadowShape = intersect(scene, shadowRay);

            // accumulate color only when there isn't shadow (aka leave shadows black)
            if (!shadowShape.hit){
                // add material response
                color += mat_res;
            }

          }

        // calc lm
        if( shape.mat->kr != zero3f ){
            // create the reflection ray
            // r = 2(n dot vd)*n-vd
            // n = shape.norm
            vec3f vd = normalize(ray.e - shape.pos);
            vec3f refl = (2 * dot(shape.norm, vd)) * shape.norm - vd;
            ray3f reflRay = ray3f(shape.pos, refl, ray3f_epsilon, ray3f_rayinf);

            // accumulate the reflected light (recursive call) scaled by the material reflection
            color += shape.mat->kr * raytrace_ray(scene, reflRay);
        }

    }
    return color;
}



// raytrace an image
image3f raytrace(Scene* scene) {


    // allocate an image of the proper size
    auto image = image3f(scene->image_width, scene->image_height);


    // if no anti-aliasing
    // condition !(image_samples > 1)
    if(!(scene->image_samples > 1)){


        // for every pixel
        for( int pRow = 0; pRow < scene->image_height; pRow++){
            for( int pCol = 0; pCol < scene->image_width; pCol++){


                // compute ray-camera parameters (u,v) for the pixel

                float u = (pRow + .5)/scene->image_width;
                float v = (pCol + .5)/scene->image_height;

                // compute camera ray
                vec3f x = ((u - .5) * scene->camera->width)*x3f;
                vec3f y = ((v-.5)*scene->camera->height)*y3f;
                vec3f z = scene->camera->dist * z3f;

                auto camRay = (x + y - z);
                ray3f normRay = ray3f(zero3f, normalize(camRay));

                // transform ray into the coordinates of the camera frame
                ray3f newRay = transform_ray(scene->camera->frame, normRay);

                // get a color for the pixel
                auto color = raytrace_ray(scene, newRay);
                image.at(pRow, pCol) = color;

            }
        }
    }
    else{
//        // for each pixel
        for( int pRow = 0; pRow < scene->image_height; pRow++){
            for( int pCol = 0; pCol < scene->image_width; pCol++){

//               // init accumulated color
                  vec3f color = zero3f;
                  for( float ii = 0; ii < scene->image_samples; ii++ ){
                      for(float jj = 0; jj < scene->image_samples; jj++){
                          // compute ray-camera parameters (u,v) for the pixel and the sample
                          float u = (pRow + (ii + 0.5)/scene->image_samples)/scene->image_width;
                          float v = (pCol + (jj + 0.5)/scene->image_samples)/scene->image_height;

                          // compute camera ray

                          vec3f x = ((u - .5) * scene->camera->width)*x3f;
                          vec3f y = ((v-.5)*scene->camera->height)*y3f;
                          vec3f z = scene->camera->dist * z3f;

                          auto camRay = (x + y - z);
                          ray3f normRay = ray3f(zero3f, normalize(camRay));

                         // transform ray into the coordinates of the camera frame
                         ray3f newRay = transform_ray(scene->camera->frame, normRay);

                         // get a color for the pixel
                         color += raytrace_ray(scene, newRay);

                      }
                  }

//                // scale by the number of samples
                  image.at(pRow, pCol) = color/(scene->image_samples*scene->image_samples);
            }
          }
       }
    return image;
}








// runs the raytrace over all tests and saves the corresponding images
int main(int argc, char** argv) {
    auto args = parse_cmdline(argc, argv,
        { "01_raytrace", "raytrace a scene",
            {  {"resolution",     "r", "image resolution", typeid(int),    true,  jsonvalue()}  },
            {  {"scene_filename", "",  "scene filename",   typeid(string), false, jsonvalue("scene.json")},
               {"image_filename", "",  "image filename",   typeid(string), true,  jsonvalue("")}  }
        });

    // generate/load scene either by creating a test scene or loading from json file
    string scene_filename = args.object_element("scene_filename").as_string();
    Scene *scene = nullptr;
    if(scene_filename.length() > 9 and scene_filename.substr(0,9) == "testscene") {
        int scene_type = atoi(scene_filename.substr(9).c_str());
        scene = create_test_scene(scene_type);
        scene_filename = scene_filename + ".json";
    } else {
        scene = load_json_scene(scene_filename);
    }
    error_if_not(scene, "scene is nullptr");

    auto image_filename = (args.object_element("image_filename").as_string() != "") ?
        args.object_element("image_filename").as_string() :
        scene_filename.substr(0,scene_filename.size()-5)+".png";

    if(not args.object_element("resolution").is_null()) {
        scene->image_height = args.object_element("resolution").as_int();
        scene->image_width = scene->camera->width * scene->image_height / scene->camera->height;
    }

    message("rendering %s...\n", scene_filename.c_str());
    auto image = raytrace(scene);

    message("writing to png...\n");
    write_png(image_filename, image, true);

    delete scene;
    message("done\n");
}
