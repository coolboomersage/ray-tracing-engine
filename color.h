#ifndef COLOR_H
#define COLOR_H

#include "vec3.h"
#include "interval.h"

#include <stdexcept>

using color = vec3;

inline double linear_to_gamma(double linear_component)
{
    if (linear_component > 0)
        return std::sqrt(linear_component);

    return 0;
}

void write_color(std::ostream& out, const color& pixel_color) {

    auto r = pixel_color.x();
    auto g = pixel_color.y();
    auto b = pixel_color.z();

    // Replace NaN components with zero.
    if (r != r) r = 0.0;
    if (g != g) g = 0.0;
    if (b != b) b = 0.0;

    // Apply a linear to gamma transform for gamma 2
    r = linear_to_gamma(r);
    g = linear_to_gamma(g);
    b = linear_to_gamma(b);

    // Translate the [0,1] component values to the byte range [0,255].
    static const interval intensity(0.000, 0.999);
    int rbyte = int(256 * intensity.clamp(r));
    int gbyte = int(256 * intensity.clamp(g));
    int bbyte = int(256 * intensity.clamp(b));

    // Write out the pixel color components.
    out << rbyte << ' ' << gbyte << ' ' << bbyte << '\n';
}

inline void write_color_checked(std::ostream& out, const color& pixel_color, int x, int y, const char* output_name) {
    try {
        write_color(out, pixel_color);
    } catch (const std::exception& error) {
        std::cerr << "Failed writing pixel (" << x << ", " << y << ") to "
                  << output_name << ": " << error.what() << std::endl;
        throw;
    }

    if (!out) {
        const std::string message = "Output stream failed after pixel (" +
            std::to_string(x) + ", " + std::to_string(y) + ")";
        std::cerr << "Failed writing pixel (" << x << ", " << y << ") to "
                  << output_name << ": " << message << std::endl;
        throw std::ios_base::failure(message);
    }
}

#endif