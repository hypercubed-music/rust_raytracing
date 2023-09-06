use ultraviolet::*;
use crate::materials::Material;

#[derive(Clone)]
pub struct Ray {
    pub origin: DVec3,
    pub direction: DVec3,
    pub color: DVec3,
    pub emissive: bool
}

impl Ray {
    pub fn new(origin: DVec3, direction: DVec3, color:DVec3) -> Ray{
        Ray {
            origin, direction, color, emissive:false
        }
    }

    pub fn at(self, t: f64) -> DVec3 {
        self.origin + t * self.direction
    }
}

#[derive(Clone)]
pub struct RayHit {
    pub p: DVec3,
    pub normal: DVec3,
    pub mat: Box<dyn Material + Sync>,
    pub t: f64,
    pub front: bool
}

impl RayHit {
    fn set_face_normal(&mut self, r:&Ray, outward_normal: DVec3) {
        self.front = r.direction.dot(outward_normal) < 0.0;
        self.normal = if self.front {outward_normal} else {-outward_normal};
    }
}

pub trait Hittable {
    fn hit(&self, r: &Ray, ray_tmin: f64, ray_tmax: f64) -> Option<RayHit>;
}

pub type HittableList =  Vec<Box<dyn Hittable + Sync + Send>>;

fn get_world_hit(r: &Ray, ray_tmin: f64, ray_tmax: f64, world:&HittableList) -> Option<RayHit> {
    let mut closest_so_far = ray_tmax;
    let mut closest_rec: Option<RayHit> = None;

    for obj in world {
        if let Some(rec) = obj.hit(r, ray_tmin, closest_so_far)  {
            closest_so_far = rec.t;
            closest_rec = Some(rec);
        }
    }
    closest_rec
}

pub struct Sphere {
    pub center: DVec3,
    pub radius: f64,
    pub mat: Box<dyn Material + Sync + Send>
}

impl Hittable for Sphere {
    fn hit(&self, r: &Ray, ray_tmin: f64, ray_tmax: f64) -> Option<RayHit> {
        let oc = r.origin - self.center;
        let a = r.direction.mag_sq();
        let half_b = oc.dot(r.direction);
        let c = oc.mag_sq() - self.radius * self.radius;
        let disc = half_b * half_b - a * c;
        if disc < 0.0 { return None};
        
        let sqrtd = disc.sqrt();
        let mut root = (-half_b - sqrtd) / a;
        if root <= ray_tmin || ray_tmax <= root {
            root = (-half_b + sqrtd) / a;
            if root <= ray_tmin || ray_tmax <= root {
                return None;
            }
        }

        let p = r.clone().at(root);
        let outward_normal = (p - self.center) / self.radius;
 
        let mut rec = RayHit {
            t: root,
            p,
            mat: self.mat.clone(),
            normal: outward_normal,
            front: true
        };
        rec.set_face_normal(&r, outward_normal);
        

        Some(rec)
    }
}

pub struct Triangle {
    pub pos1: DVec3,
    pub pos2: DVec3,
    pub pos3: DVec3,
    pub norm1: DVec3,
    pub norm2: DVec3,
    pub norm3: DVec3,
    pub mat: Box<dyn Material + Sync + Send>
}

impl Hittable for Triangle {
    fn hit(&self, r: &Ray, ray_tmin: f64, ray_tmax: f64) -> Option<RayHit> {
        let edge12 = self.pos2 - self.pos1;
        let edge13 = self.pos3 - self.pos1;
        let norm = edge12.cross(edge13);
        let ao = r.origin - self.pos1;
        let dao = ao.cross(r.direction);

        let det = -(r.direction.dot(norm));
        let inv_det = 1.0 / det;

        let dst = ao.dot(norm) * inv_det;
        let u = edge13.dot(dao) * inv_det;
        let v = -(edge12.dot(dao)) * inv_det;
        let w = 1.0 - u - v;

        if det >= 1e-6 && dst >= 0.0 && u >= 0.0 && v >= 0.0 && w >= 0.0 {
            let hit = RayHit{
                p: r.origin + r.direction * dst,
                mat: self.mat.clone(),
                normal: (self.norm1 * w + self.norm2 * u + self.norm3 * v).normalized(),
                t: dst,
                front: true
            };
            Some(hit)
        } else {
            None
        }
    }
}

pub fn unit_samp() -> DVec3 {
    loop {
        let x = fastrand::f64() * 2.0 - 1.0;
        let y = fastrand::f64() * 2.0 - 1.0;
        let z = fastrand::f64() * 2.0 - 1.0;
        let v = DVec3::new(x, y, z);
        if v.mag_sq() < 1.0 {
            return v.normalized();
        }
    }
}

fn unit_hemi_samp(normal: DVec3) -> DVec3 {
    let v = unit_samp();
    return if v.dot(normal) > 0.0 {v} else {-v};
}

fn environment_light(ray:Ray) -> DVec3 {
    let unit_direction = ray.direction.normalized();
    let a = 0.5 * (unit_direction.y + 1.0);
    let sky_gradient = (1.0 - a) * DVec3::new(1.0, 1.0, 1.0) + a * DVec3::new(0.5, 0.7, 1.0);
    
    let sun = ray.direction.dot(DVec3::new(0.5, 0.5, 0.0)).max(0.0).powf(10.0) * 1.0;
    sky_gradient + DVec3::one() * sun
}

pub fn ray_color(ray: Ray, world: &HittableList, depth:i32) -> DVec3 {
    
    if depth == 0 {return DVec3::zero()};

    if let Some(rec) = get_world_hit(&ray, 0.001, f64::INFINITY, world)  {
        if let Some(scattered) = rec.mat.scatter(&ray, &rec) {
            if scattered.emissive {
                return scattered.color;
            } else {
                let current_color = ray.color * scattered.color;
                if current_color.x < 0.01 && current_color.y < 0.01 && current_color.z < 0.01 {
                    return current_color;
                } else {
                    return current_color * ray_color(scattered, world, depth - 1);
                }
            }
        }
    }

    //environment_light(ray)
    DVec3::zero()
}

pub fn depth_check(ray: Ray, world: &HittableList, depth:i32) -> i32 {
    if depth == 0 {return 0};

    if let Some(rec) = get_world_hit(&ray, 0.001, f64::INFINITY, world)  {
        if let Some(scattered) = rec.mat.scatter(&ray, &rec) {
            if scattered.emissive {
                return 1;
            } else {
                return depth_check(scattered, world, depth - 1) + 1;
            }
        }
    }
    0
}

pub fn square_samp(u:DVec3, v:DVec3) -> DVec3 {
    let x = fastrand::f64() - 0.5;
    let y = fastrand::f64() - 0.5;
    (x * u) + (y * v)
}