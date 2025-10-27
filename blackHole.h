#ifndef BLACK_HOLE_H
#define BLACK_HOLE_H

#include "hittable.h"
#include "vec3.h"
#include "ray.h"
#include "material.h"

class black_hole : public hittable {
public:
    black_hole() {}
    black_hole(point3 cen, double r, double mass)
        : center(cen), radius(r), M(mass) {
        mat = std::make_shared<material>(); //default material has no emissions and allways returns black
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        vec3 oc = r.origin() - center;
        auto a = dot(r.direction(), r.direction());
        auto half_b = dot(oc, r.direction());
        auto c = dot(oc, oc) - radius*radius;

        auto discriminant = half_b*half_b - a*c;
        if (discriminant < 0) return false;
        auto sqrtd = sqrt(discriminant);

        auto root = (-half_b - sqrtd) / a;
        if (!ray_t.surrounds(root)) {
            root = (-half_b + sqrtd) / a;
            if (!ray_t.surrounds(root))
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        rec.normal = (rec.p - center) / radius;
        rec.set_face_normal(r, rec.normal);

        // force black material
        rec.mat = mat;

        return true;
    }

    double rs() const { return 2.0 * M; }

    point3 getCenter() const { return(center); }
    double getMass() const { return(M); }

    virtual aabb bounding_box() const override {
        return aabb(center - vec3(rs(), rs(), rs()),
                    center + vec3(rs(), rs(), rs()));
    }

private:
    point3 center;
    double radius;
    double M;
    std::shared_ptr<material> mat;
};

#endif