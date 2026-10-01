#include "rtweekend.h"
#include "bvh.h"
#include "camera.h"
#include "color.h"
#include "ray.h"
#include "vec3.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "sphere.h"
#include "texture.h"
#include "quad.h"
#include "disk.h"
#include "tri.h"
#include "constant_medium.h"
#include "blackHole.h"

#include <exception>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#include <windows.h>
static LONG WINAPI seh_filter(EXCEPTION_POINTERS* ep) {
    std::fprintf(stderr, "SEH 0x%08lX at %p\n",
                 ep->ExceptionRecord->ExceptionCode, ep->ExceptionRecord->ExceptionAddress);
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif

#include <conio.h> // for getch()

namespace {
    void print_usage(const char* program_name) {
        std::cout << "Usage: " << program_name << " [options]\n"
                  << "  --aspect-ratio VALUE\n"
                  << "  --image-width VALUE\n"
                  << "  --samples-per-pixel VALUE\n"
                  << "  --max-depth VALUE\n"
                  << "  --background R G B\n"
                  << "  --vfov VALUE\n"
                  << "  --lookfrom X Y Z\n"
                  << "  --lookat X Y Z\n"
                  << "  --vup X Y Z\n"
                  << "  --defocus-angle VALUE\n"
                  << "  --gr-mode true|false\n"
                  << "  --num-threads VALUE\n"
                  << "  --help\n";
    }

    double next_double(int& index, int argc, char** argv, const char* option) {
        if (index + 1 >= argc) {
            throw std::invalid_argument(std::string(option) + " requires a value");
        }
        return std::stod(argv[++index]);
    }

    int next_int(int& index, int argc, char** argv, const char* option) {
        if (index + 1 >= argc) {
            throw std::invalid_argument(std::string(option) + " requires a value");
        }
        return std::stoi(argv[++index]);
    }

    vec3 next_vec3(int& index, int argc, char** argv, const char* option) {
        if (index + 3 >= argc) {
            throw std::invalid_argument(std::string(option) + " requires three values");
        }
        return vec3(std::stod(argv[++index]), std::stod(argv[++index]), std::stod(argv[++index]));
    }

    bool next_bool(int& index, int argc, char** argv, const char* option) {
        if (index + 1 >= argc) {
            throw std::invalid_argument(std::string(option) + " requires true or false");
        }
        std::string value = argv[++index];
        if (value == "true" || value == "1") return true;
        if (value == "false" || value == "0") return false;
        throw std::invalid_argument(std::string(option) + " requires true or false");
    }

    void parse_camera_options(int argc, char** argv, camera& cam) {
        for (int index = 1; index < argc; ++index) {
            std::string option = argv[index];
            if (option == "--help") {
                print_usage(argv[0]);
                std::exit(EXIT_SUCCESS);
            } else if (option == "--aspect-ratio") {
                cam.aspect_ratio = next_double(index, argc, argv, "--aspect-ratio");
            } else if (option == "--image-width") {
                cam.image_width = next_int(index, argc, argv, "--image-width");
            } else if (option == "--samples-per-pixel") {
                cam.samples_per_pixel = next_int(index, argc, argv, "--samples-per-pixel");
            } else if (option == "--max-depth") {
                cam.max_depth = next_int(index, argc, argv, "--max-depth");
            } else if (option == "--background") {
                cam.background = next_vec3(index, argc, argv, "--background");
            } else if (option == "--vfov") {
                cam.vfov = next_double(index, argc, argv, "--vfov");
            } else if (option == "--lookfrom") {
                cam.lookfrom = next_vec3(index, argc, argv, "--lookfrom");
            } else if (option == "--lookat") {
                cam.lookat = next_vec3(index, argc, argv, "--lookat");
            } else if (option == "--vup") {
                cam.vup = next_vec3(index, argc, argv, "--vup");
            } else if (option == "--defocus-angle") {
                cam.defocus_angle = next_double(index, argc, argv, "--defocus-angle");
            } else if (option == "--gr-mode") {
                cam.gr_mode = next_bool(index, argc, argv, "--gr-mode");
            } else if (option == "--num-threads") {
                cam.numThreads = next_int(index, argc, argv, "--num-threads");
            } else {
                throw std::invalid_argument("unknown option: " + option);
            }
        }
    }
}

int main(int argc, char** argv) {
    hittable_list world;
    camera cam;

    cam.aspect_ratio      = 1.0;
    cam.image_width       = 4000;
    cam.samples_per_pixel = 50;
    cam.max_depth         = 50;
    cam.background        = color(1,1,1);

    cam.vfov     = 40;
    cam.lookfrom = point3(5 , 15 , -60);
    cam.lookat   = point3(0 , 5 , 0);
    cam.vup      = vec3(0, 1, 0);

    
    cam.defocus_angle = 0;

    cam.gr_mode = true;

    cam.numThreads = 16;

    try {
        parse_camera_options(argc, argv, cam);
    } catch (const std::exception& error) {
        std::cerr << "Argument error: " << error.what() << "\n";
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    
    //materials and textures
    auto red   = make_shared<lambertian>(color(.65, .05, .05));
    auto check = make_shared<checker_texture>(100, color(1,0,0), color(0,0,1));
    auto light = make_shared<diffuse_light>(color(15, 15, 15));
    auto empty_material = shared_ptr<material>();

    //world ground
    world.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(check)));

    // black hole
    auto holePTR = make_shared<black_hole>(point3(0,5,0), 0.5 , 0.5);
    if (!cam.gr_mode) {
        world.add(holePTR); // only include BH as a hittable if not bending
    }

    //light sources
    world.add(make_shared<quad>(point3(-5 , 20 , -5) , vec3(10 , 0 , 0) , vec3(0 , 0 , 10) , light));

    hittable_list lights;
    lights.add(make_shared<quad>(point3(5 , 20 , 5), vec3(-10 , 0 , 0), vec3( 0 , 0 , -10), empty_material));

    //file import
    //world.addFromFile("models/chest.obj" , 0.1 , {0 , 0 , 0} , {0 , -45 , 0});
    //world.addFromFile("models/bow.fbx" , 0.1 , {9 , 0 , 0} , {0 , 0 , 0});

    cam.bh = holePTR.get();

    std::cout << "press any key to continue to rendering\n";
    getch();

    try {
        cam.render(world, lights, "black_hole_testing");
    } catch (const std::exception& error) {
        std::cerr << "Rendering terminated: " << error.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/*/

    //CODE FOR CORNELL BOX SCENE

    hittable_list world;

    auto red   = make_shared<lambertian>(color(.65, .05, .05));
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto green = make_shared<lambertian>(color(.12, .45, .15));
    auto light = make_shared<diffuse_light>(color(15, 15, 15));

    // Cornell box sides
    world.add(make_shared<quad>(point3(555,0,0), vec3(0,0,555), vec3(0,555,0), green));
    world.add(make_shared<quad>(point3(0,0,555), vec3(0,0,-555), vec3(0,555,0), red));
    world.add(make_shared<quad>(point3(0,555,0), vec3(555,0,0), vec3(0,0,555), white));
    world.add(make_shared<quad>(point3(0,0,555), vec3(555,0,0), vec3(0,0,-555), white));
    world.add(make_shared<quad>(point3(555,0,555), vec3(-555,0,0), vec3(0,555,0), white));

    // Light
    world.add(make_shared<quad>(point3(213,554,227), vec3(130,0,0), vec3(0,0,105), light));

    // Box 1
    shared_ptr<material> aluminum = make_shared<metal>(color(0.8, 0.85, 0.88), 0.0);
    shared_ptr<hittable> box1 = box(point3(0,0,0), point3(165,330,165), aluminum);
    box1 = make_shared<rotate_y>(box1, 15);
    box1 = make_shared<translate>(box1, vec3(265,0,295));
    world.add(box1);

    // Glass Sphere
    auto glass = make_shared<dielectric>(1.5);
    world.add(make_shared<sphere>(point3(190,90,190), 90, glass));

    // Light Sources
    auto empty_material = shared_ptr<material>();
    hittable_list lights;
    lights.add(
        make_shared<quad>(point3(343,554,332), vec3(-130,0,0), vec3(0,0,-105), empty_material));
    lights.add(make_shared<sphere>(point3(190, 90, 190), 90, empty_material));

    camera cam;

    cam.aspect_ratio      = 1.0;
    cam.image_width       = 600;
    cam.samples_per_pixel = 1000;
    cam.max_depth         = 50;
    cam.background        = color(0,0,0);

    cam.vfov     = 40;
    cam.lookfrom = point3(278, 278, -800);
    cam.lookat   = point3(278, 278, 0);
    cam.vup      = vec3(0, 1, 0);

    cam.defocus_angle = 0;



    // CODE FOR GEODESIC SCENE
    hittable_list world;
    camera cam;

    cam.aspect_ratio      = 1.0;
    cam.image_width       = 4000;
    cam.samples_per_pixel = 50;
    cam.max_depth         = 50;
    cam.background        = color(1,1,1);

    cam.vfov     = 40;
    cam.lookfrom = point3(5 , 15 , -60);
    cam.lookat   = point3(0 , 5 , 0);
    cam.vup      = vec3(0, 1, 0);

    
    cam.defocus_angle = 0;

    cam.gr_mode = true;

    cam.numThreads = 16;

    try {
        parse_camera_options(argc, argv, cam);
    } catch (const std::exception& error) {
        std::cerr << "Argument error: " << error.what() << "\n";
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    
    //materials and textures
    auto red   = make_shared<lambertian>(color(.65, .05, .05));
    auto check = make_shared<checker_texture>(100, color(1,0,0), color(0,0,1));
    auto light = make_shared<diffuse_light>(color(15, 15, 15));
    auto empty_material = shared_ptr<material>();

    //world ground
    world.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(check)));

    // black hole
    auto holePTR = make_shared<black_hole>(point3(0,5,0), 0.5 , 0.5);
    if (!cam.gr_mode) {
        world.add(holePTR); // only include BH as a hittable if not bending
    }

    //light sources
    world.add(make_shared<quad>(point3(-5 , 20 , -5) , vec3(10 , 0 , 0) , vec3(0 , 0 , 10) , light));

    hittable_list lights;
    lights.add(make_shared<quad>(point3(5 , 20 , 5), vec3(-10 , 0 , 0), vec3( 0 , 0 , -10), empty_material));

    //file import
    //world.addFromFile("models/chest.obj" , 0.1 , {0 , 0 , 0} , {0 , -45 , 0});
    //world.addFromFile("models/bow.fbx" , 0.1 , {9 , 0 , 0} , {0 , 0 , 0});

    cam.bh = holePTR.get();
*/