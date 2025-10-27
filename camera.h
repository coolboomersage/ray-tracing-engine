#ifndef CAMERA_H
#define CAMERA_H

#include "hittable.h"
#include "pdf.h"
#include "timeleft.h"
#include "material.h"
#include "ppmtopng.h"
#include "blackHole.h"
#include "geodesic.h"

#include <iostream>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <atomic>
#include <cmath>
#include <pthread.h>
#include <cstdlib> // Required for system()
#include <thread> // used for std::this_thread::sleep_for()

static std::atomic<int> debug_call_id { 0 };

class camera {
  public:
    double aspect_ratio = 1.0;  // Ratio of image width over height
    int    image_width  = 100;  // Rendered image width in pixel count
    int    samples_per_pixel = 10;   // Count of random samples for each pixel
    int    max_depth         = 10;   // Maximum number of ray bounces into scene
    double    reflectance       = 0.1;  // How reflective the scene is overall
    color  background;               // Scene background color

    const black_hole* bh = nullptr;    // set from scene if present
    bool gr_mode = false;              // on/off switch

    double vfov       = 90;              // Vertical view angle (field of view)
    point3 lookfrom   = point3(0,0,0);   // Point camera is looking from
    point3 lookat     = point3(0,0,-1);  // Point camera is looking at
    vec3   vup        = vec3(0,1,0);     // Camera-relative "up" direction
    int    numThreads = 8;              // Number of threads to use while rendering

    double defocus_angle = 0;  // Variation angle of rays through each pixel
    double focus_dist = 10;    // Distance from camera lookfrom point to plane of perfect focus

    void render(const hittable& world, const hittable& lights , std::string WorldName) {
        initialize();
        auto startTime = std::chrono::steady_clock::now();
        auto currentTime = std::chrono::steady_clock::now();

        std::ofstream ppmFile(WorldName + ".ppm");
        ppmFile << "P3\n" << image_width << ' ' << image_height << "\n255\n";
        std::vector<color> pixelbuffer(image_width * image_height);

        if (numThreads == 1) {
            // --- Single-threaded bypass (no pthreads) ---
            ThreadData data(0, image_height, world, pixelbuffer, this, pixel_samples_scale, lights);

            std::cout << "Running in single-threaded mode...\n";
            renderThread(&data);

            // Write pixels directly
            for (int j = 0; j < image_height; ++j) {
                for (int i = 0; i < image_width; ++i) {
                    write_color(ppmFile, pixelbuffer[j * image_width + i]);
                }
            }
            std::cout << "wrote color to file" << std::endl;

        } else {
            // --- Multithreaded mode (pthreads) ---
            int startLine = 0;
            int endLine = 0;
            int LinesPerThread = floor(image_height / numThreads);
            std::vector<ThreadData*> holder;
            pthread_t threadArray[numThreads];

            // Launch threads
            for(int i = 0; i < numThreads; i ++) {
                if (i == numThreads - 1) {
                    endLine = image_height;
                } else {
                    endLine = startLine + LinesPerThread;
                }
                ThreadData* Data = new ThreadData(startLine , endLine , world , pixelbuffer , this , pixel_samples_scale , lights);
                holder.push_back(Data);

                pthread_create(&threadArray[i] , NULL , renderThread , Data);
                startLine += LinesPerThread;
            }

            // Poll thread progress
            bool allDone = false;
            while(!allDone) {
                std::ostringstream message;
                message.str("");
                message.clear();
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // poll each 0.5s

                int totalCompleted = 0;
                for (int i = 0; i < numThreads; ++i) {
                    int total = holder[i]->stop - holder[i]->start;
                    totalCompleted += holder[i]->lines_completed;

                    if(holder[i]->lines_completed != total){
                        message << "Thread " << i << " has completed "
                                << holder[i]->lines_completed << " out of " << total << " lines\n";
                    }
                }
                clearScreen();
                message << totalCompleted << " out of " << image_height << " lines have been completed overall\n";
                message << "EST time remaining for " << WorldName << ": ";

                // Estimate time
                double seconds = time_left(startTime, currentTime, image_height, totalCompleted).count();
                int hrs = static_cast<int>(seconds) / 3600;
                int mins = (static_cast<int>(seconds) % 3600) / 60;
                int secs = static_cast<int>(seconds) % 60;
                if (hrs > 0) message << hrs << "h ";
                if (mins > 0 || hrs > 0) message << mins << "m ";
                double fractional = seconds - static_cast<int>(seconds);
                message << std::fixed << std::setprecision(2) << secs + fractional << "s\n";

                std::cout << message.str();
                currentTime = std::chrono::steady_clock::now();

                if (totalCompleted == image_height){
                    allDone = true;
                }
            }

            // Join threads
            for(int i = 0; i< numThreads; i++){
                pthread_join(threadArray[i], NULL);
            }
            for (auto x: holder){
                delete(x);
            }

            // Write pixels after threads complete
            for (int j = 0; j < image_height; ++j) {
                for (int i = 0; i < image_width; ++i) {
                    write_color(ppmFile, pixelbuffer[j * image_width + i]);
                }
            }
        }

        ppmFile.close();  
        convertppm(WorldName);
        std::cout << "\r" << WorldName << " is done.                                            \n";
    }

    static void* renderThread(void* arg){
        ThreadData* info = static_cast<ThreadData*>(arg);

        for(int j = info->start; j < info->stop; j ++){
            for(int i = 0; i < info->instance->image_width; i++){
                color pixel_color(0, 0, 0);
                for (int s_j = 0; s_j < info->instance->sqrt_spp; s_j++) {
                    for (int s_i = 0; s_i < info->instance->sqrt_spp; s_i++) {
                        ray r = info->instance->get_ray(i, j, s_i, s_j);
                        pixel_color += info->instance->ray_color(r, info->instance->max_depth, info->world, info->lights);
                    }
                }
                info->pixelbuffer[j * info->instance->image_width + i] = pixel_color * info->pixelSampleScale;
                //std::cout << "added to buffer\n";
            }
            info->lines_completed++;
        }

        return(NULL);
    }

  private:
    int    image_height;   // Rendered image height
    double pixel_samples_scale;  // Color scale factor for a sum of pixel samples
    int    sqrt_spp;             // Square root of number of samples per pixel
    double recip_sqrt_spp;       // 1 / sqrt_spp
    point3 center;         // Camera center
    point3 pixel00_loc;    // Location of pixel 0, 0
    vec3   pixel_delta_u;  // Offset to pixel to the right
    vec3   pixel_delta_v;  // Offset to pixel below
    vec3   u, v, w;              // Camera frame basis vectors
    vec3   defocus_disk_u;       // Defocus disk horizontal radius
    vec3   defocus_disk_v;       // Defocus disk vertical radius

    class ThreadData{
        public:
            int start;
            int stop;
            const hittable& world;
            const hittable& lights;
            double pixelSampleScale;
            std::vector<color>& pixelbuffer;
            camera* instance;
            std::atomic<int> lines_completed;

            ThreadData(int startLine , int endLine , const hittable& worldInput , std::vector<color>& inputBuffer , camera* instanceInput , double sampleInput , const hittable& lightInput):
                start(startLine) , stop(endLine) , world(worldInput) , pixelbuffer(inputBuffer) , instance(instanceInput) , pixelSampleScale(sampleInput) , lights(lightInput){
                    lines_completed = 0;
                }
        
    };

    void initialize() {
        image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        sqrt_spp = int(std::sqrt(samples_per_pixel));
        pixel_samples_scale = 1.0 / (sqrt_spp * sqrt_spp);
        recip_sqrt_spp = 1.0 / sqrt_spp;

        center = lookfrom;

        // Determine viewport dimensions.
        auto theta = degrees_to_radians(vfov);
        auto h = std::tan(theta/2);
        auto viewport_height = 2 * h * focus_dist;
        auto viewport_width = viewport_height * (double(image_width)/image_height);

        // Calculate the u,v,w unit basis vectors for the camera coordinate frame.
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        vec3 viewport_u = viewport_width * u;    // Vector across viewport horizontal edge
        vec3 viewport_v = viewport_height * -v;  // Vector down viewport vertical edge

        // Calculate the horizontal and vertical delta vectors from pixel to pixel.
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // Calculate the location of the upper left pixel.
        auto viewport_upper_left = center - (focus_dist * w) - viewport_u/2 - viewport_v/2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        // Calculate the camera defocus disk basis vectors.
        auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j, int s_i, int s_j) const {
        // Construct a camera ray originating from the defocus disk and directed at a randomly
        // sampled point around the pixel location i, j for stratified sample square s_i, s_j.

        auto offset = sample_square_stratified(s_i, s_j);
        auto pixel_sample = pixel00_loc
                          + ((i + offset.x()) * pixel_delta_u)
                          + ((j + offset.y()) * pixel_delta_v);

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;
        auto ray_time = random_double();

        return ray(ray_origin, ray_direction, ray_time);
    }

    vec3 sample_square_stratified(int s_i, int s_j) const {
        // Returns the vector to a random point in the square sub-pixel specified by grid
        // indices s_i and s_j, for an idealized unit square pixel [-.5,-.5] to [+.5,+.5].

        auto px = ((s_i + random_double()) * recip_sqrt_spp) - 0.5;
        auto py = ((s_j + random_double()) * recip_sqrt_spp) - 0.5;

        return vec3(px, py, 0);
    }

    vec3 sample_square() const {
        // Returns the vector to a random point in the [-.5,-.5]-[+.5,+.5] unit square.
        return vec3(random_double() - 0.5, random_double() - 0.5, 0);
    }

    color sample_background_dir(const vec3& d) const {
        vec3 unit_direction = unit_vector(d);
        auto a = 0.5*(unit_direction.y() + 1.0);
        return (1.0-a)*color(1.0, 1.0, 1.0) + a*background;
    }

    point3 defocus_disk_sample() const {
        // Returns a random point in the camera defocus disk.
        auto p = random_in_unit_disk();
        return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    color ray_color(const ray& r, int depth, const hittable& world, const hittable& lights)
    const
{
    // If we've exceeded the ray bounce limit, no more light is gathered.
    if (depth <= 0) return color(0,0,0);

    // ---------- GR branch ----------
    if (gr_mode && bh && depth == max_depth) {
        auto res = trace_null_geodesic_schwarzschild(r.origin(), r.direction(), *bh);

        // Black hole captured or perfect loop → return black
        if (res.black == true) {
            return color(0,0,0);
        } else if (res.out_ray.has_value()) {
            const ray escaped_ray = *res.out_ray;
            hit_record rec;

            // If the ray hits nothing, return the background color.
            if (!world.hit(escaped_ray, interval(0.001, infinity), rec)){ return background; }

            //if (world.hit(escaped_ray, interval(0.001, infinity), rec)){ return color(1,0,1); }

            scatter_record srec;
            color color_from_emission = rec.mat->emitted(escaped_ray, rec, rec.u, rec.v, rec.p);

            if (!rec.mat->scatter(escaped_ray, rec, srec))
                return color_from_emission;

            if (srec.skip_pdf) {
                return srec.attenuation * ray_color(srec.skip_pdf_ray, depth-1, world, lights);
            }

            auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
            mixture_pdf p(light_ptr, srec.pdf_ptr);

            ray scattered = ray(rec.p, p.generate(), escaped_ray.time());
            auto pdf_value = p.value(scattered.direction());

            double scattering_pdf = rec.mat->scattering_pdf(ray(rec.p, -unit_vector(escaped_ray.direction()), escaped_ray.time()), rec, scattered);

            color sample_color = ray_color(scattered, depth-1, world, lights);
            color color_from_scatter =
                (srec.attenuation * scattering_pdf * sample_color) / pdf_value;

            return color_from_emission + color_from_scatter;
            
        } else {
            // bug catcher
            return color(1,1,1);
        }
        // ---------- END GR branch ----------
    }
    

    if(gr_mode == false) {
        hit_record rec;

        // If the ray hits nothing, return the background color.
        if (!world.hit(r, interval(0.001, infinity), rec)){ return background; }

        scatter_record srec;
        color color_from_emission = rec.mat->emitted(r, rec, rec.u, rec.v, rec.p);

        if (!rec.mat->scatter(r, rec, srec))
            return color_from_emission;

        if (srec.skip_pdf) {
            return srec.attenuation * ray_color(srec.skip_pdf_ray, depth-1, world, lights);
        }

        auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
        mixture_pdf p(light_ptr, srec.pdf_ptr);

        ray scattered = ray(rec.p, p.generate(), r.time());
        auto pdf_value = p.value(scattered.direction());

        double scattering_pdf = rec.mat->scattering_pdf(r, rec, scattered);

        color sample_color = ray_color(scattered, depth-1, world, lights);
        color color_from_scatter =
            (srec.attenuation * scattering_pdf * sample_color) / pdf_value;

        return color_from_emission + color_from_scatter;
    }
    return(color{1,1,1});
}

    void clearScreen() {
        #ifdef _WIN32 // For Windows
            system("cls");
        #else // For Linux/macOS
            system("clear");
        #endif
        }
};


#endif