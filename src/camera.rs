use std::f64::consts::PI;
use std::thread;
use std::sync::Arc;

use ultraviolet::DVec3;
use ndarray::Array3;
use crate::raytracing::{HittableList, square_samp, ray_color, depth_check, Ray};

const THREAD_COUNT: i32 = 15;

pub struct Camera {
    pub width: i32,
    pub height: i32,
    pub samples: i32,
    pub max_depth: i32,
    pub vfov: f64,
}

fn unit_disk_samp() -> DVec3 {
    loop {
        let x = fastrand::f64() * 2.0 - 1.0;
        let y = fastrand::f64() * 2.0 - 1.0;
        let v = DVec3::new(x, y, 0.0);
        if v.mag_sq() < 1.0 {
            return v;
        }
    }
}

fn deg_to_rad(angle: f64) -> f64 {
    angle * PI / 180.0
}

impl Camera {

    pub fn render(self, world:HittableList, lookfrom:DVec3, lookat:DVec3, up:DVec3, defocus_angle:f64, focus_dist:f64) -> Array3<f32> {
        // Camera stuff
        let theta = deg_to_rad(self.vfov);
        let h = (theta / 2.0).tan();
        let view_height = 2.0 * h * focus_dist;
        let view_width = view_height * (self.width as f64 / self.height as f64);

        let camera_center = lookfrom;
        
        let w = (lookfrom - lookat).normalized();
        let u = up.cross(w).normalized();
        let v = w.cross(u);

        let viewport_u = view_width * u;
        let viewport_v = view_height * -v;
    
        let pixel_delta_u = viewport_u / self.width as f64;
        let pixel_delta_v = viewport_v / self.height as f64;
    
        let view_upper_left = camera_center - (focus_dist * w) - viewport_u/2.0 - viewport_v/2.0;
        let pixel_zero_loc = view_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);
    
        let defocus_radius = focus_dist * (deg_to_rad(defocus_angle) / 2.0).tan();
        let defocus_disk_u = u * defocus_radius;
        let defocus_disk_v = v * defocus_radius;

        

        let world_arc = Arc::new(world);

        let mut img: Array3<f64> = Array3::zeros((self.height as usize, self.width as usize, 3));

        let mut v = Vec::<thread::JoinHandle<Array3<f64>>>::new();
        
        for thread_no in 0..THREAD_COUNT {
            let world = Arc::clone(&world_arc);
            
            let jh = thread::spawn(move || {
                fastrand::seed(thread_no as u64);
                let mut sub_img: Array3<f64> = Array3::zeros((self.height as usize, self.width as usize, 3));
                for j in 0..self.height as usize {
                    if thread_no == 0 {println!("Scanline {}", j)};
                    for i in 0..self.width as usize {
                        let pixel_center = pixel_zero_loc + (i as f64 * pixel_delta_u) + (j as f64 * pixel_delta_v);
                        let mut pixel_color = DVec3::new(0.0, 0.0, 0.0);
                        
                        let p = unit_disk_samp();
                        let disk_sample = camera_center + (p.x * defocus_disk_u) + (p.y * defocus_disk_v);
                        let ray_origin = if defocus_angle <= 0.0 {camera_center} else {disk_sample};
    
                        let ray_direction = pixel_center + square_samp(pixel_delta_u, pixel_delta_v) - ray_origin;
    
                        let r = Ray::new(ray_origin, ray_direction, DVec3::one());

                        let depth = depth_check(r , &world, self.max_depth) as f64;

                        let num_samples = (self.samples as f64 * (depth / self.max_depth as f64)).round();

                        for _ in 0..num_samples as i32 {
                            let p = unit_disk_samp();
                            let disk_sample = camera_center + (p.x * defocus_disk_u) + (p.y * defocus_disk_v);
                            let ray_origin = if defocus_angle <= 0.0 {camera_center} else {disk_sample};
        
                            let ray_direction = pixel_center + square_samp(pixel_delta_u, pixel_delta_v) - ray_origin;
        
                            let r = Ray::new(ray_origin, ray_direction, DVec3::one());
                            
                            let sample_color = ray_color(r, &world, self.max_depth);
                            pixel_color += sample_color;
                        }
                        sub_img[(j, i, 0)] = pixel_color.x / num_samples as f64;
                        sub_img[(j, i, 1)] = pixel_color.y / num_samples as f64;
                        sub_img[(j, i, 2)] = pixel_color.z / num_samples as f64;
                    }
                }
                sub_img
            });
            v.push(jh);
        }

        for jh in v.into_iter() {
            let sub_img:Array3<f64> = jh.join().unwrap();
            for ((x, y, z), v) in img.indexed_iter_mut() {
                *v += sub_img[(x, y, z)] / (THREAD_COUNT as f64);
            }
        }

        let mut final_img:Array3<f32> = Array3::zeros((self.height as usize, self.width as usize, 3));
        for ((x, y, z), v) in final_img.indexed_iter_mut() {
            *v = img[(x, y, z)].sqrt() as f32;
        }
        final_img
    }
}