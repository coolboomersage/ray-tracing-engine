#ifndef HITTABLE_LIST_H
#define HITTABLE_LIST_H

#include "aabb.h"
#include "hittable.h"
#include "ray.h"
#include "tri.h"
#include "material.h"

#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <memory>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

using std::make_shared;
using std::shared_ptr;

class hittable_list : public hittable {
  public:
    std::vector<shared_ptr<hittable>> objects;

    hittable_list() {}
    hittable_list(shared_ptr<hittable> object) { add(object); }

    void clear() { objects.clear(); }

    void add(shared_ptr<hittable> object) {
        objects.push_back(object);
        bbox = aabb(bbox, object->bounding_box());
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        hit_record temp_rec;
        bool hit_anything = false;
        auto closest_so_far = ray_t.max;

        for (const auto& object : objects) {
            if (object->hit(r, interval(ray_t.min, closest_so_far), temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }

        return hit_anything;
    }
    
    aabb bounding_box() const override { return bbox; }

    double pdf_value(const point3& origin, const vec3& direction) const override {
        auto weight = 1.0 / objects.size();
        auto sum = 0.0;

        for (const auto& object : objects)
            sum += weight * object->pdf_value(origin, direction);

        return sum;
    }

    vec3 random(const point3& origin) const override {
        auto int_size = int(objects.size());
        return objects[random_int(0, int_size-1)]->random(origin);
    }

    void addFromFile(std::string fileName ,  float scale = 1 , vec3 position = vec3 {0,0,0} , vec3 rotation = vec3{0,0,0}){
        if (fileName.substr(fileName.size() - 4) == ".fbx"){
            std::cout << "converting " << fileName << " to a .obj file\n";

            Assimp::Importer importer;

            const aiScene* scene = importer.ReadFile(
                fileName,
                aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
            );
            if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
                std::cerr << "Assimp failed to load file: " << importer.GetErrorString() << std::endl;
                return;
            }

            Assimp::Exporter exporter;

            fileName = fileName.erase(fileName.length() - 4) + ".obj";

            aiReturn result = exporter.Export(scene, "obj", fileName);

            if (result != aiReturn_SUCCESS) {
                std::cerr << "Assimp failed to export OBJ: " << exporter.GetErrorString() << std::endl;
                return;
            }
        }
        if (fileName.substr(fileName.size() - 4) == ".obj"){
            std::cout << "parsing " << fileName << "\n";

            float rx = radians(rotation.x());
            float ry = radians(rotation.y());
            float rz = radians(rotation.z());

            mat3 rotX = mat3{
                {1, 0, 0},
                {0, cos(rx), -sin(rx)},
                {0, sin(rx), cos(rx)}
            };

            mat3 rotY = mat3{
                {cos(ry), 0, sin(ry)},
                {0, 1, 0},
                {-sin(ry), 0, cos(ry)}
            };

            mat3 rotZ = mat3{
                {cos(rz), -sin(rz), 0},
                {sin(rz), cos(rz), 0},
                {0, 0, 1}
            };

            mat3 rotationMatrix = rotZ * rotY * rotX;

            std::ifstream file(fileName);
            std::string line;
            auto purple = make_shared<lambertian>(color(0.5, 0, 0.5));

            int trisAdded =  0; //DEBUG TOOL, delete later

            std::vector<point3> positions; //TODO incorperate normals and tex cords

            while (std::getline(file, line)) {
                std::istringstream iss(line);
                std::string prefix;
                iss >> prefix;

                if (prefix == "v") {
                    float x, y, z;
                    iss >> x >> y >> z;
                    positions.push_back(point3{x, y, z} * scale);
                }
            }

            // Compute bounding box center
            point3 min_point = positions[0];
            point3 max_point = positions[0];

            for (const auto& p : positions) {
                min_point = min(min_point, p);
                max_point = max(max_point, p);
            }

            point3 center = point3{
                (min_point.x() + max_point.x()) * 0.5,
                min_point.y(), //use min y value ST the midpoint of the base is the "origin"
                (min_point.z() + max_point.z()) * 0.5
            };
            vec3 offset = position - center;

            // Apply offset and rotation matrix to all vertices
            for (auto& p : positions) {
                p = rotationMatrix * (p - center); // Rotate around object's center
                p += position;                     // Translate to world position
            }

            // Second pass: Parse faces
            file.clear();
            file.seekg(0);
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                std::string prefix;
                iss >> prefix;

                if (prefix == "f") {
                    std::vector<int> indices;
                    std::string x;

                    while (iss >> x) {
                        int index = std::stoi(x.substr(0, x.find('/')));
                        indices.push_back(index - 1);
                    }

                    if (indices.size() == 3) {
                        add(make_shared<tri>(
                            positions[indices[0]],
                            positions[indices[1]] - positions[indices[0]],
                            positions[indices[2]] - positions[indices[0]],
                            purple
                        ));
                        trisAdded ++;
                    } else if (indices.size() == 4) {
                        add(make_shared<tri>(
                            positions[indices[0]],
                            positions[indices[1]] - positions[indices[0]],
                            positions[indices[2]] - positions[indices[0]],
                            purple
                        ));
                        trisAdded ++;
                        add(make_shared<tri>(
                            positions[indices[0]],
                            positions[indices[2]] - positions[indices[0]],
                            positions[indices[3]] - positions[indices[0]],
                            purple
                        ));
                        trisAdded ++;
                    }
                }
            }
            std::cout << "finished parsing " << fileName << " with " << trisAdded << " tris added\n\n";
        }
    }

  private:
    aabb bbox;

    vec3 min(const vec3& a, const vec3& b) {
        return vec3(
            std::min(a.e[0], b.e[0]),
            std::min(a.e[1], b.e[1]),
            std::min(a.e[2], b.e[2])
        );
    }

    vec3 max(const vec3& a, const vec3& b) {
        return vec3(
            std::max(a.e[0], b.e[0]),
            std::max(a.e[1], b.e[1]),
            std::max(a.e[2], b.e[2])
        );
    }

    double radians(double degree){
        return(degree * (pi / 180.0));
    }

    class mat3{
        public:
            vec3 r1 = {0,0,0};
            vec3 r2 = {0,0,0};
            vec3 r3 = {0,0,0};

            mat3(vec3 r1i , vec3 r2i , vec3 r3i):
            r1(r1i) , r2(r2i) , r3(r3i)
            {}

            inline mat3 operator*(const mat3& other) const {
                vec3 col1 = {other.r1.e[0], other.r2.e[0], other.r3.e[0]};
                vec3 col2 = {other.r1.e[1], other.r2.e[1], other.r3.e[1]};
                vec3 col3 = {other.r1.e[2], other.r2.e[2], other.r3.e[2]};

                vec3 new_r1 = {
                    dot(r1, col1),
                    dot(r1, col2),
                    dot(r1, col3)
                };

                vec3 new_r2 = {
                    dot(r2, col1),
                    dot(r2, col2),
                    dot(r2, col3)
                };

                vec3 new_r3 = {
                    dot(r3, col1),
                    dot(r3, col2),
                    dot(r3, col3)
                };

                return mat3(new_r1, new_r2, new_r3);
            }

            inline vec3 operator*(const vec3& v) {
            return vec3{
                dot(r1 , v),
                dot(r2 , v),
                dot(r3 , v)
            };
        }
    };
};

#endif