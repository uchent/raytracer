#include "util.h"

#include "bvh.h"
#include "camera.h"
#include "color.h"
#include "constant_medium.h"
#include "hittable_list.h"
#include "material.h"
#include "primitives.h"
#include "obj_loader.h"
#include "texture.h"

void render() {
	auto purple = make_shared<lambertian>(color(0.2, 0.1, 0.4));
	auto d_green = make_shared<lambertian>(color(0.1, 0.4, 0.2));
	auto light = make_shared<diffuse_light>(color(7.0, 7.0, 7.0));
	auto checker_floor = make_shared<checker_texture>(0.8, color(0.4, 0.2, 0.1), color(.9, .9, .9));
	auto metal_t = make_shared<metal>(color::random(0.5, 1), 0);

	// bunny
	hittable_list world = obj_loader::load_model(metal_t, 15);
	world = hittable_list(make_shared<bvh_node>(world));

	// balls
	world.add(make_shared<sphere>(point3(1.5, 1.0, 2.5), 0.3, light));
	world.add(make_shared<sphere>(point3(-2.5, 1.0, 0.0), 0.3, light));
	world.add(make_shared<sphere>(point3(1.5, 1.0, -4.0), 0.3, light));

	// back
	world.add(make_shared<triangle>(point3(-8, 0.5, -9), point3(20, 0.5, 0), point3(20, 10.0, 0), vec3(0, 0, 1),
		d_green));

	// left
	world.add(make_shared<triangle>(point3(-1.0, 0.5, -9), point3(-5, 0.5, 5), point3(-1.0, 10.0, -9), vec3(1, 0, 0),
		purple));
	
	// floor
	world.add(make_shared<triangle>(point3(-8, 0.5, -9), point3(20, 0.5, 0), point3(-5, 0.5, 5), vec3(0, 1, 0),
		make_shared<lambertian>(checker_floor)));

	camera cam;

	cam.aspect_ratio = 16.0 / 9.0;
	cam.image_width = 800;
	cam.samples_per_pixel = 10;
	cam.max_depth = 10;
	cam.background = color(0.0, 0.0, 0.0);

	cam.vfov = 30;
	cam.lookfrom = point3(-2.5, 7, 7);
	cam.lookat = point3(0, 1.5, 0);
	cam.vup = vec3(0, 1, 0);

	cam.defocus_angle = 0;

	cam.render(world);
}


int main() {
	render();
}