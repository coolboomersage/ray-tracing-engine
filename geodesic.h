#ifndef GEODESIC_H
#define GEODESIC_H

#include "vec3.h"
#include "ray.h"
#include "blackHole.h"
#include "rtweekend.h"
#include <cmath>
#include <optional>

struct plane_basis {
    vec3 e_r0;
    vec3 e_phi0;
};

// Safe normalize with fallback
inline vec3 safe_unit(const vec3& v) {
    auto n = v.length();
    return (n > 0) ? (v / n) : vec3(1,0,0);
}

inline plane_basis make_plane_basis(const point3& O, const vec3& D, const point3& C) {
    vec3 R0 = O - C;
    vec3 d  = safe_unit(D);

    vec3 N = cross(R0, d);
    double nlen = N.length();
    if (nlen < 1e-12) {
        N = fabs(d.x()) < 0.9 ? cross(d, vec3(1,0,0)) : cross(d, vec3(0,1,0));
    }
    N = safe_unit(N);

    vec3 e_r0   = safe_unit(R0);
    vec3 e_phi0 = safe_unit(cross(N, e_r0));

    return {e_r0, e_phi0};
}

// Convert (r, phi) in the plane to world coordinates and plane tangents
inline void plane_to_world(
    const plane_basis& B, const point3& C, double r, double phi,
    point3& out_pos, vec3& rhat, vec3& phihat
) {
    double cph = std::cos(phi), sph = std::sin(phi);
    vec3 pos   = r * (cph * B.e_r0 + sph * B.e_phi0);
    out_pos    = C + pos;
    rhat       = safe_unit(cph * B.e_r0 + sph * B.e_phi0);
    phihat     = safe_unit(-sph * B.e_r0 + cph * B.e_phi0);
}

struct GeodesicResult {
    bool black;                     // true => ray captured or perfect loop -> "black"
    std::optional<ray> out_ray;     // valid when black == false (escaped)
    int  loops;
};

inline GeodesicResult trace_null_geodesic_schwarzschild(
    const point3& camO, const vec3& dir, const black_hole& bh,
    double dphi = 1e-3,
    int max_steps = 20'000,      // Increase max steps
    double R_far = 100.0         // Reduce escape radius for your scene
) {
    const double M  = bh.getMass();
    const double rs = bh.rs();

    auto B = make_plane_basis(camO, dir, bh.getCenter());

    vec3 R0v = camO - bh.getCenter();
    double r0 = R0v.length();

    vec3 e_r0 = safe_unit(R0v);
    vec3 e_phi0 = B.e_phi0;

    vec3 v0 = safe_unit(dir);
    double v_r   = dot(v0, e_r0);
    double v_phi = dot(v0, e_phi0);

    double phi = 0.0;
    double u = 1.0 / r0;
    double du_dphi = (std::fabs(v_phi) > 1e-8) ? (- (1.0/r0) * (v_r / v_phi)) : - (1.0/r0) * v_r;

    int loops = 0;

    auto acc = [&](double u_) { return 3.0 * M * u_ * u_ - u_; };

    for (int i = 0; i < max_steps; ++i) {
        double r = (u > 1e-30) ? 1.0 / u : R_far;

        // Absorbed by the event horizon
        const double epsilon = 1e-6;
        if (r <= rs * (1.0 - epsilon)) {
            return GeodesicResult{ true, std::nullopt, loops };
        }

        // Count loops
        loops = int(std::floor(std::fabs(phi) / (2.0 * pi)));
        if (loops >= 5) return GeodesicResult{ true, std::nullopt, loops };

        // Escaped to “far away” (for debug, just R_far ~ 100)
        if (r >= R_far) {
            point3 posW;
            vec3 rhat, phihat;
            plane_to_world(B, bh.getCenter(), r, phi, posW, rhat, phihat);
            double dr_dphi = - (r * r) * du_dphi;
            vec3 tangent = safe_unit(dr_dphi * rhat + r * phihat);
            ray escaped_ray(posW, tangent , random_double());
            return GeodesicResult{ false, escaped_ray, loops };
        }

        // RK4 integration
        double v = du_dphi;
        double k1_u = v;
        double k1_v = acc(u);

        double k2_u = v + 0.5 * dphi * k1_v;
        double k2_v = acc(u + 0.5 * dphi * k1_u);

        double k3_u = v + 0.5 * dphi * k2_v;
        double k3_v = acc(u + 0.5 * dphi * k2_u);

        double k4_u = v + dphi * k3_v;
        double k4_v = acc(u + dphi * k3_u);

        u       += (dphi/6.0) * (k1_u + 2*k2_u + 2*k3_u + k4_u);
        du_dphi += (dphi/6.0) * (k1_v + 2*k2_v + 2*k3_v + k4_v);

        phi += dphi;
    }

    // Fallback: treat as escaped if still far enough after max_steps
    double r_final = (u > 1e-30) ? 1.0 / u : R_far;
    if (r_final >= 10.0) {  // anything beyond 10 units from BH counts as escape
        point3 posW;
        vec3 rhat, phihat;
        plane_to_world(B, bh.getCenter(), r_final, phi, posW, rhat, phihat);
        double dr_dphi = - (r_final * r_final) * du_dphi;
        vec3 tangent = safe_unit(dr_dphi * rhat + r_final * phihat);
        return GeodesicResult{ false, ray(posW, tangent , random_double()), loops };
    }

    return GeodesicResult{ true, std::nullopt, loops };
}

#endif