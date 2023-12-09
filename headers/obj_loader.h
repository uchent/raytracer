#pragma once
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "hittable_list.h"
#include "primitives.h"

class obj_loader {
public:
	static hittable_list load_model(shared_ptr<material> m, int scale) {
		hittable_list list;

		std::string path = "C:\\Users\\Tai\\Desktop\\New folder\\bunny.obj";
		std::ifstream obj(path);
		std::string type, x, y, z;
		std::vector<point3> vertices;

		while (obj >> type) {
			if (type == "v") {
				obj >> x >> y >> z;
				point3 p(std::stod(x) * scale, std::stod(y) * scale, std::stod(z) * scale);
				vertices.push_back(p);
			}
			else if (type == "f") {
				obj >> x >> y >> z;
				std::vector<std::string> vx = split(x, '/');
				std::vector<std::string> vy = split(y, '/');
				std::vector<std::string> vz = split(z, '/');

				point3 p1 = vertices[std::stoi(vx[0]) - 1];
				point3 p2 = vertices[std::stoi(vy[0]) - 1];
				point3 p3 = vertices[std::stoi(vz[0]) - 1];

				vec3 face_n = cal_face_normal(p1, p2, p3);

				list.add(make_shared<triangle>(p1, p2, p3, face_n, m));
			}
		}

		obj.close();
		return list;
	}

	static std::vector<std::string> split(const std::string& s, char delim) {
		std::vector<std::string> result;
		std::stringstream ss(s);
		std::string item;

		while (getline(ss, item, delim)) {
			result.push_back(item);
		}

		return result;
	}

	static vec3 cal_face_normal(point3 p1, point3 p2, point3 p3)
	{
		vec3 v0 = p2 - p1;
		vec3 v1 = p3 - p1;
		vec3 face_normal = unit_vector(cross(v0, v1));
		return face_normal;
	}
};

