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

pub struct CameraConfig {
    pub pixel_zero_loc: DVec3,
    pub pixel_delta_u: DVec3,
    pub pixel_delta_v: DVec3,
    pub camera_center: DVec3,
    pub defocus_angle: f64,
    pub defocus_disk_u: DVec3,
    pub defocus_disk_v: DVec3,
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
    pub fn get_config(&self, lookfrom:DVec3, lookat:DVec3, up:DVec3, defocus_angle:f64, focus_dist:f64) -> CameraConfig {
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

        CameraConfig {
            pixel_zero_loc,
            pixel_delta_u,
            pixel_delta_v,
            camera_center,
            defocus_angle,
            defocus_disk_u,
            defocus_disk_v,
        }
    }

    pub fn render_pass(&self, world: &Arc<HittableList>, config: &Arc<CameraConfig>) -> Array3<f64> {
        let mut img: Array3<f64> = Array3::zeros((self.height as usize, self.width as usize, 3));
        
        let num_chunks = THREAD_COUNT as usize;
        let chunk_height = (self.height as usize + num_chunks - 1) / num_chunks;
        
        let mut v = Vec::new();

        for chunk_idx in 0..num_chunks {
            let world = Arc::clone(world);
            let config = Arc::clone(config);
            let width = self.width as usize;
            let height = self.height as usize;
            let max_depth = self.max_depth;
            let samples = self.samples;

            let start_y = chunk_idx * chunk_height;
            let end_y = (start_y + chunk_height).min(height);

            if start_y >= height { break; }

            let jh = thread::spawn(move || {
                fastrand::seed(chunk_idx as u64 + fastrand::u64(..));
                let mut sub_img: Array3<f64> = Array3::zeros((end_y - start_y, width, 3));
                
                for j in start_y..end_y {
                    for i in 0..width {
                        let pixel_center = config.pixel_zero_loc + (i as f64 * config.pixel_delta_u) + (j as f64 * config.pixel_delta_v);
                        
                        let p = unit_disk_samp();
                        let disk_sample = config.camera_center + (p.x * config.defocus_disk_u) + (p.y * config.defocus_disk_v);
                        let ray_origin = if config.defocus_angle <= 0.0 {config.camera_center} else {disk_sample};
                        let ray_direction = pixel_center + square_samp(config.pixel_delta_u, config.pixel_delta_v) - ray_origin;
                        let r = Ray::new(ray_origin, ray_direction, DVec3::one());

                        let depth = depth_check(r , &world, max_depth) as f64;
                        let num_samples = (samples as f64 * (depth / max_depth as f64)).round().max(1.0) as i32;

                        let mut pixel_color = DVec3::new(0.0, 0.0, 0.0);
                        for _ in 0..num_samples {
                            let p = unit_disk_samp();
                            let disk_sample = config.camera_center + (p.x * config.defocus_disk_u) + (p.y * config.defocus_disk_v);
                            let ray_origin = if config.defocus_angle <= 0.0 {config.camera_center} else {disk_sample};
                            let ray_direction = pixel_center + square_samp(config.pixel_delta_u, config.pixel_delta_v) - ray_origin;
                            let r = Ray::new(ray_origin, ray_direction, DVec3::one());
                            
                            let sample_color = ray_color(r, &world, max_depth);
                            pixel_color += sample_color;
                        }
                        sub_img[(j - start_y, i, 0)] = pixel_color.x / num_samples as f64;
                        sub_img[(j - start_y, i, 1)] = pixel_color.y / num_samples as f64;
                        sub_img[(j - start_y, i, 2)] = pixel_color.z / num_samples as f64;
                    }
                }
                (start_y, end_y, sub_img)
            });
            v.push(jh);
        }

        for jh in v {
            let (start_y, end_y, sub_img) = jh.join().unwrap();
            for j in start_y..end_y {
                for i in 0..self.width as usize {
                    img[(j, i, 0)] = sub_img[(j - start_y, i, 0)];
                    img[(j, i, 1)] = sub_img[(j - start_y, i, 1)];
                    img[(j, i, 2)] = sub_img[(j - start_y, i, 2)];
                }
            }
        }
        img
    }

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
                        sub_img[(j, i, 0)] = pixel_color.x / num_samples;
                        sub_img[(j, i, 1)] = pixel_color.y / num_samples;
                        sub_img[(j, i, 2)] = pixel_color.z / num_samples;
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