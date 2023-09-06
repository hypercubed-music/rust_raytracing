use image::RgbImage;
use ndarray::Array3;
use fltk::{app, prelude::*, window::Window, image::PngImage, frame::Frame};
extern crate oidn;

mod camera;
mod materials;
mod raytracing;

use camera::Camera;
use raytracing::{HittableList, Sphere, unit_samp};
use ultraviolet::DVec3;

const WIDTH: i32 = 1920;
const HEIGHT: i32 = 1080;

fn main() {

    let mut world = HittableList::new();

        let mat_ground = materials::Lambertian{albedo: DVec3::new(0.5, 0.5, 0.5)};
        world.push(Box::new(Sphere{center: DVec3::new(0.0, -1000.0, 0.0), radius: 1000.0, mat:Box::new(mat_ground)}));
    
        for a in -11..11 {
            for b in -11..11 {
                let choose_mat = fastrand::f64();
                let center = DVec3::new(a as f64 + 0.9 * fastrand::f64(), 0.2, b as f64 + 0.9 * fastrand::f64());
                if (center - DVec3::new(4.0, 0.2, 0.0)).mag() > 0.9 {
                    if choose_mat < 0.8 {
                        let albedo = unit_samp();
                        let sphere_mat = materials::Lambertian{albedo};
                        world.push(
                            Box::new(Sphere{center, radius:0.2, mat:Box::new(sphere_mat)})
                        );
                    } else if choose_mat < 0.8 {
                        let albedo = unit_samp();
                        let fuzz = fastrand::f64() * 0.5;
                        let sphere_mat = materials::Metal{albedo, fuzz};
                        world.push(
                            Box::new(Sphere{center, radius:0.2, mat:Box::new(sphere_mat)})
                        );
                    } else {
                        let sphere_mat = materials::Dielectric{ior:1.5};
                        world.push(
                            Box::new(Sphere{center, radius:0.2, mat:Box::new(sphere_mat)})
                        );
                    }
                }
            }
        }

        let mat1 = materials::Emissive{color:DVec3::new(1.0, 1.0, 1.0),strength:10.0};
        world.push(
            Box::new(Sphere{center:DVec3::new(0.0, 1.0, 0.0), radius:1.0, mat:Box::new(mat1)})
        );

        let mat2 = materials::Lambertian{albedo:DVec3::new(0.4, 0.2, 0.1)};
        world.push(
            Box::new(Sphere{center:DVec3::new(-4.0, 1.0, 0.0), radius:1.0, mat:Box::new(mat2)})
        );

        let mat3 = materials::Metal{albedo:DVec3::new(0.7, 0.6, 0.5), fuzz: 0.0};
        world.push(
            Box::new(Sphere{center:DVec3::new(4.0, 1.0, 0.0), radius:1.0, mat:Box::new(mat3)})
        );

    let camera = Camera{width: WIDTH, height:HEIGHT, samples:20, max_depth:10, vfov:20.0};

    let lookfrom = DVec3::new(13.0, 2.0, 3.0);
    let lookat = DVec3::new(0.0, 0.0, -1.0);
    let vup = DVec3::new(0.0, 1.0, 0.0);

    let array: Array3<f32> = camera.render(world, lookfrom, lookat, vup, 0.6, 10.0);
    let array_vec = array.into_raw_vec();
    
    let mut filtered_out = vec![0.0_f32; array_vec.len()];

    let device = oidn::Device::new();
    oidn::RayTracing::new(&device)
        .srgb(true)
        .image_dimensions(WIDTH as usize, HEIGHT as usize)
        .filter(&array_vec[..], &mut filtered_out[..])
        .expect("Filter config error!");

    let filtered_u8 = filtered_out.iter().map(|&e| (e * 255.0) as u8).collect();
    let image = RgbImage::from_raw(WIDTH as u32, HEIGHT as u32, filtered_u8)
        .expect("container should have the right size for the image dimensions");
    let _ = image.save("out.png");

    let loaded_img = PngImage::load("out.png").unwrap();

    let app = app::App::default();
    let mut wind = Window::new(100, 100, WIDTH, HEIGHT, "Hello from rust");
    let mut frame = Frame::default().with_size(WIDTH, HEIGHT).center_of(&wind);
    frame.set_image(Some(loaded_img));
    
    wind.end();
    wind.show();
    app.run().unwrap();
}
