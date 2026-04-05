use std::f64::consts::PI;
use std::sync::{Arc, Mutex};
use ndarray::Array3;
use fltk::{app, prelude::*, window::Window, image::RgbImage as FltkRgbImage, frame::Frame};
//extern crate oidn;

mod camera;
mod materials;
mod raytracing;
mod obj_loader;

use camera::Camera;
use obj_loader::load_mesh;
use raytracing::{HittableList, Sphere, unit_samp, Mesh};
use ultraviolet::{DVec3, DRotor3};

const WIDTH: i32 = 640;
const HEIGHT: i32 = 480;

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

    let mat1 = materials::Emissive{color:DVec3::new(1.0, 1.0, 1.0),strength:20.0};
    world.push(
        Box::new(Sphere{center:DVec3::new(2.0, 3.0, -1.0), radius:1.0, mat:Box::new(mat1)})
    );

    let mat2 = materials::Metal{albedo:DVec3::new(0.4, 0.2, 0.1), fuzz:1.0};
    /*world.push(
        Box::new(Sphere{center:DVec3::new(-4.0, 1.0, 0.0), radius:1.0, mat:Box::new(mat2)})
    );*/
    let mut suzanne = Mesh::new(
        load_mesh("C:\\Users\\joshu\\Documents\\rust_projects\\rust_raytracing\\suzanne.obj"), Box::new(mat2)
    );
    suzanne.transform(DVec3::new(1.0, 0.5, -1.0), DRotor3::from_euler_angles(0.0, -PI / 4.0, -PI / 4.0));
    world.push(Box::new(suzanne));

    /*let mat3 = materials::Metal{albedo:DVec3::new(0.7, 0.6, 0.5), fuzz: 0.0};
    world.push(
        Box::new(Sphere{center:DVec3::new(4.0, 1.0, 0.0), radius:1.0, mat:Box::new(mat3)})
    );*/

    let lookfrom = DVec3::new(13.0, 2.0, 3.0);
    let lookat = DVec3::new(0.0, 0.5, -1.0);
    let vup = DVec3::new(0.0, 1.0, 0.0);

    let camera = Camera{width: WIDTH, height:HEIGHT, samples:1, max_depth:2, vfov:20.0};
    let config = Arc::new(camera.get_config(lookfrom, lookat, vup, 0.1, 10.0));
    let world = Arc::new(world);

    let accum_img = Arc::new(Mutex::new(Array3::<f64>::zeros((HEIGHT as usize, WIDTH as usize, 3))));
    let pass_count = Arc::new(Mutex::new(0));

    let app = app::App::default();
    let mut wind = Window::new(100, 100, WIDTH, HEIGHT, "Ray Tracing Progress");
    let mut frame = Frame::default().with_size(WIDTH, HEIGHT).center_of(&wind);
    wind.show();

    let (s, r) = app::channel::<()>();

    let accum_img_render = Arc::clone(&accum_img);
    let pass_count_render = Arc::clone(&pass_count);
    let world_render = Arc::clone(&world);
    let config_render = Arc::clone(&config);

    std::thread::spawn(move || {
        loop {
            let pass_img = camera.render_pass(&world_render, &config_render);
            let mut accum = accum_img_render.lock().unwrap();
            let mut count = pass_count_render.lock().unwrap();
            *count += 1;
            
            for ((y, x, c), val) in accum.indexed_iter_mut() {
                *val += pass_img[(y, x, c)];
            }
            s.send(());
        }
    });

    while app.wait() {
        if let Some(_) = r.recv() {
            let accum = accum_img.lock().unwrap();
            let count = *pass_count.lock().unwrap();
            
            println!("{} passes completed", count);
            
            let mut buffer = vec![0u8; (WIDTH * HEIGHT * 3) as usize];
            for y in 0..HEIGHT as usize {
                for x in 0..WIDTH as usize {
                    for c in 0..3 {
                        let val = accum[(y, x, c)] / count as f64;
                        let color = (val.sqrt().clamp(0.0, 1.0) * 255.0) as u8;
                        buffer[(y * WIDTH as usize + x) * 3 + c] = color;
                    }
                }
            }
            let fltk_img = FltkRgbImage::new(&buffer, WIDTH, HEIGHT, fltk::enums::ColorDepth::Rgb8).unwrap();
            frame.set_image(Some(fltk_img));
            wind.redraw();
        }
    }
}
