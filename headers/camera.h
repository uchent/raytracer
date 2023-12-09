#pragma once

#include "util.h"

#include "color.h"
#include "hittable.h"
#include "material.h"
#include "sdl.h"

#include <iostream>


class camera {
public:
	double aspect_ratio = 1.0;    // Ratio of image width over height
	int    image_width = 100;    // Rendered image width in pixel count
	int    samples_per_pixel = 10;     // Count of random samples for each pixel
	int    max_depth = 10;     // Maximum number of ray bounces into scene
	color  background;                 // Scene background color

	double vfov = 90;              // Vertical view angle (field of view)
	point3 lookfrom = point3(0, 0, -1);  // Point camera is looking from
	point3 lookat = point3(0, 0, 0);   // Point camera is looking at
	vec3   vup = vec3(0, 1, 0);     // Camera-relative "up" direction

	double defocus_angle = 0;  // Variation angle of rays through each pixel
	double focus_dist = 10;    // Distance from camera lookfrom point to plane of perfect focus

	void render(const hittable& world) {
		initialize();
		for (int j = 0; j < image_height; ++j) {
			std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
			for (int i = 0; i < image_width; ++i) {
				color accumulate_color(0, 0, 0);
				for (int sample = 0; sample < samples_per_pixel; ++sample) {
					ray r = get_ray(i, j);
					accumulate_color += ray_color(r, max_depth, world);
				}
				color pixel_color = get_pixel_color(std::cout, accumulate_color, samples_per_pixel);
				sdl::setDrawColor(sdl::createColor(pixel_color.x(), pixel_color.y(), pixel_color.z(), 255));
				sdl::drawPoint(i, j);
			}
		}
		while (sdl::running) {
			sdl::loop();
		}
	}

private:
	int    image_height;    // Rendered image height
	point3 center;          // Camera center
	point3 pixel00_loc;     // Location of pixel 0, 0
	vec3   pixel_delta_u;   // Offset to pixel to the right
	vec3   pixel_delta_v;   // Offset to pixel below
	vec3   u, v, w;         // Camera frame basis vectors
	vec3   defocus_disk_u;  // Defocus disk horizontal radius
	vec3   defocus_disk_v;  // Defocus disk vertical radius

	void initialize() {
		image_height = static_cast<int>(image_width / aspect_ratio);
		image_height = (image_height < 1) ? 1 : image_height;

		sdl::sdl("Ray Tracer", image_width, image_height);
		sdl::loop();

		center = lookfrom;

		auto theta = degrees_to_radians(vfov);
		auto h = tan(theta / 2);
		auto viewport_height = 2 * h * focus_dist;
		auto viewport_width = viewport_height * (static_cast<double>(image_width) / image_height);

		w = unit_vector(lookfrom - lookat);
		u = unit_vector(cross(vup, w));
		v = cross(w, u);

		vec3 viewport_u = viewport_width * u;
		vec3 viewport_v = viewport_height * -v;

		pixel_delta_u = viewport_u / image_width;
		pixel_delta_v = viewport_v / image_height;

		auto viewport_upper_left = center - (focus_dist * w) - viewport_u / 2 - viewport_v / 2;
		pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

		auto defocus_radius = focus_dist * tan(degrees_to_radians(defocus_angle / 2));
		defocus_disk_u = u * defocus_radius;
		defocus_disk_v = v * defocus_radius;
	}

	ray get_ray(int i, int j) const {
		auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
		auto pixel_sample = pixel_center + pixel_sample_square();

		auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
		auto ray_direction = pixel_sample - ray_origin;
		auto ray_time = random_double();

		return ray(ray_origin, ray_direction, ray_time);
	}

	vec3 pixel_sample_square() const {
		auto px = -0.5 + random_double();
		auto py = -0.5 + random_double();
		return (px * pixel_delta_u) + (py * pixel_delta_v);
	}

	vec3 pixel_sample_disk(double radius) const {
		auto p = radius * random_in_unit_disk();
		return (p[0] * pixel_delta_u) + (p[1] * pixel_delta_v);
	}

	point3 defocus_disk_sample() const {
		auto p = random_in_unit_disk();
		return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
	}

	color ray_color(const ray& r, int depth, const hittable& world) const {
		if (depth <= 0)
			return color(0, 0, 0);

		hit_record rec;

		if (!world.hit(r, interval(0.001, infinity), rec))
			return background;

		ray scattered;
		color attenuation;
		color color_from_emission = rec.mat->emitted(rec.u, rec.v, rec.p);

		if (!rec.mat->scatter(r, rec, attenuation, scattered))
			return color_from_emission;

		color color_from_scatter = attenuation * ray_color(scattered, depth - 1, world);

		return color_from_emission + color_from_scatter;
	}
};