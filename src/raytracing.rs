use ultraviolet::*;
use crate::materials::Material;
use crate::obj_loader::MeshTriangle;

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
    pub hit_point: DVec3,
    pub normal: DVec3,
    pub mat: Box<dyn Material + Sync>,
    pub hit_time: f64,
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
    fn bounding_box_hit(&self, r: &Ray, ray_tmin: f64, ray_tmax: f64) -> bool;
}

pub type HittableList =  Vec<Box<dyn Hittable + Sync + Send>>;

fn get_world_hit(r: &Ray, ray_tmin: f64, ray_tmax: f64, world:&HittableList) -> Option<RayHit> {
    let mut closest_so_far = ray_tmax;
    let mut closest_rec: Option<RayHit> = None;

    for obj in world {
        if !obj.bounding_box_hit(r, ray_tmin, closest_so_far) {continue};
        if let Some(rec) = obj.hit(r, ray_tmin, closest_so_far)  {
            closest_so_far = rec.hit_time;
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
            hit_time: root,
            hit_point: p,
            mat: self.mat.clone(),
            normal: outward_normal,
            front: true
        };
        rec.set_face_normal(&r, outward_normal);
        

        Some(rec)
    }

    fn bounding_box_hit(&self, r: &Ray, ray_tmin: f64, ray_tmax: f64) -> bool {
        let oc = r.origin - self.center;
        let a = r.direction.mag_sq();
        let half_b = oc.dot(r.direction);
        let c = oc.mag_sq() - self.radius * self.radius;
        let disc = half_b * half_b - a * c;
        if disc < 0.0 { return false};

        let sqrtd = disc.sqrt();
        let mut root = (-half_b - sqrtd) / a;
        if root <= ray_tmin || ray_tmax <= root {
            root = (-half_b + sqrtd) / a;
            if root <= ray_tmin || ray_tmax <= root {return false};
        }
        true
    }
}

pub struct BoundingBox {
    pub min: DVec3,
    pub max: DVec3
}

pub struct Mesh {
    pub tris: Vec<MeshTriangle>,
    pub mat: Box<dyn Material + Sync + Send>,
    pub position: DVec3,
    pub rotation: DRotor3,
    pub transformed_tris: Vec<MeshTriangle>,
    pub bounding_box: BoundingBox
}

fn calcuate_bounding_box(tris: Vec<MeshTriangle>) -> BoundingBox {
    let mut min = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = DVec3::new(-f64::INFINITY, -f64::INFINITY, -f64::INFINITY);
    for tri in tris.iter() {
        min.x = min.x.min(tri.pos1.x).min(tri.pos2.x).min(tri.pos3.x);
        min.y = min.y.min(tri.pos1.y).min(tri.pos2.y).min(tri.pos3.y);
        min.z = min.z.min(tri.pos1.z).min(tri.pos2.z).min(tri.pos3.z);
        max.x = max.x.max(tri.pos1.x).max(tri.pos2.x).max(tri.pos3.x);
        max.y = max.y.max(tri.pos1.y).max(tri.pos2.y).max(tri.pos3.y);
        max.z = max.z.max(tri.pos1.z).max(tri.pos2.z).max(tri.pos3.z);
    }
    BoundingBox{min, max}
}

impl Mesh {
    pub fn new(tris: Vec<MeshTriangle>, mat: Box<dyn Material + Sync + Send>) -> Mesh {
        Mesh {
            tris:tris.clone(),
            mat, 
            position: DVec3::zero(),
            rotation: DRotor3::identity(),
            transformed_tris: tris.clone(),
            bounding_box: calcuate_bounding_box(tris)
        }
    }

    pub fn transform(&mut self, pos: DVec3, rot:DRotor3) {
        self.position = pos;
        self.rotation = rot;
        // transforms the mesh
        let mut transformed_tri_list: Vec<MeshTriangle> = vec![];
        for tri in self.tris.iter() {
            let new_tri = MeshTriangle {
                pos1: tri.pos1.rotated_by(self.rotation) + self.position,
                pos2: tri.pos2.rotated_by(self.rotation) + self.position,
                pos3: tri.pos3.rotated_by(self.rotation) + self.position,
                norm1: tri.norm1.rotated_by(self.rotation),
                norm2: tri.norm2.rotated_by(self.rotation),
                norm3: tri.norm3.rotated_by(self.rotation)
            };
            transformed_tri_list.push(new_tri);
        }
        self.transformed_tris = transformed_tri_list;
        self.bounding_box = calcuate_bounding_box(self.transformed_tris.clone());
    }   
}

#[derive(PartialEq)]
enum bbPosition {
    Left, Middle, Right
}

impl Hittable for Mesh {
    fn hit(&self, r: &Ray, ray_tmin: f64, ray_tmax: f64) -> Option<RayHit> {
        let mut current_hit: Option<RayHit> = None;
        for tri in self.transformed_tris.iter() {
            let edge12 = tri.pos2 - tri.pos1;
            let edge13 = tri.pos3 - tri.pos1;
            let norm = edge12.cross(edge13);
            if norm.dot(r.direction) > 0.0 {
                // backface
                continue;
            }
            let ao = r.origin - tri.pos1;
            let dao = ao.cross(r.direction);

            let det = -(r.direction.dot(norm));
            let inv_det = 1.0 / det;

            let dst = ao.dot(norm) * inv_det;
            if dst <= ray_tmin || ray_tmax <= dst {
                continue;
            }
            let u = edge13.dot(dao) * inv_det;
            let v = -(edge12.dot(dao)) * inv_det;
            let w = 1.0 - u - v;

            if det >= 1e-6 && dst >= 0.0 && u >= 0.0 && v >= 0.0 && w >= 0.0 {
                let hit = RayHit{
                    hit_point: r.origin + r.direction * dst,
                    mat: self.mat.clone(),
                    normal: (tri.norm1 * w + tri.norm2 * u + tri.norm3 * v).normalized(),
                    hit_time: dst,
                    front: true
                };
                if let Some(other_hit) = current_hit.clone() {
                    // compare distances
                    if dst < other_hit.hit_time {
                        current_hit = Some(hit);
                    }
                } else {
                    current_hit = Some(hit);
                }
            }
        }
        current_hit
    }

    fn bounding_box_hit(&self, r: &Ray, ray_tmin: f64, ray_tmax: f64) -> bool {
        // fast ray-aabb intersection by andrew woo
        let mut inside = true;
        let mut quadrant = [ const { bbPosition::Middle }; 3];
        let mut which_plane = 0;
        let mut max_t = DVec3::new(0.0, 0.0, 0.0);
        let mut candidate_plane = DVec3::new(0.0, 0.0, 0.0);

        for i in 0..3 {
            if r.origin[i] < self.bounding_box.min[i] {
                quadrant[i] = bbPosition::Left; // LEFT
                candidate_plane[i] = self.bounding_box.min[i];
                inside = false;
            } else if r.origin[i] > self.bounding_box.max[i] {
                quadrant[i] = bbPosition::Right; // RIGHT
                candidate_plane[i] = self.bounding_box.max[i];
                inside = false;
            } else {
                quadrant[i] = bbPosition::Middle; // MIDDLE
            }
        }

        if (inside) {return true}

        for i in 0..3 {
            if quadrant[i] != bbPosition::Middle && r.direction[i] != 0.0 {
                max_t[i] = (candidate_plane[i] - r.origin[i]) / r.direction[i];
            } else {
                max_t[i] = -1.0;
            }
        }

        for i in 1..3 {
            if max_t[i] > max_t[which_plane] {
                which_plane = i;
            }
        }

        if max_t[which_plane] < 0.0 {return false}
        for i in 0..3 {
            if which_plane != i {
                let coord = r.origin[i] + max_t[which_plane] * r.direction[i];
                if coord < self.bounding_box.min[i] || coord > self.bounding_box.max[i] {
                    return false;
                }
            }
        }

        true
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
    if v.dot(normal) > 0.0 {v} else {-v}
}

fn environment_light(ray:Ray) -> DVec3 {
    let unit_direction = ray.direction.normalized();
    let a = 0.5 * (unit_direction.y + 1.0);
    let sky_gradient = (1.0 - a) * DVec3::new(0.68, 0.98, 1.00) + a * DVec3::new(0.5, 0.66, 1.0);
    
    let sun = ray.direction.dot(DVec3::new(0.5, 0.5, 0.0)).max(0.0).powf(10.0) * 1.0;
    //sky_gradient + DVec3::one() * sun
    sky_gradient
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

    environment_light(ray)
    //DVec3::zero()
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
    1
}

pub fn square_samp(u:DVec3, v:DVec3) -> DVec3 {
    let x = fastrand::f64() - 0.5;
    let y = fastrand::f64() - 0.5;
    (x * u) + (y * v)
}