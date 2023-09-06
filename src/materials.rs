
use crate::raytracing::{Ray, RayHit, unit_samp};
use ultraviolet::*;
use dyn_clone::DynClone;

fn near_zero(v: DVec3) -> bool {
    let eps: f64 = 1e-8;
    v.x < eps && v.y < eps && v.z < eps
}

fn reflect(v:DVec3, normal:DVec3) -> DVec3 {
    v - (2.0 * v.dot(normal)) * normal
}

fn refract(uv: DVec3, n: DVec3, etai_over_etat: f64) -> DVec3 {
    let cos_theta = n.dot(-uv).min(1.0);
    let r_out_perp = etai_over_etat * (uv + cos_theta*n);
    let r_out_parallel = (-(1.0 - r_out_perp.mag_sq()).abs().sqrt()) * n;
    r_out_parallel + r_out_perp
}

fn reflectance(cos: f64, ref_idx: f64) -> f64 {
    let mut r0 = (1.0 - ref_idx) / (1.0 + ref_idx);
    r0 = r0.powi(2);
    r0 + (1.0-r0) * (1.0-cos).powi(5)
}

pub trait Material: DynClone {
    fn scatter(&self, r_in: &Ray, rec: &RayHit) -> Option<Ray>;
}

dyn_clone::clone_trait_object!(Material);

#[derive(Clone)]
pub struct Lambertian{
    pub albedo: DVec3
}

impl Material for Lambertian {
    fn scatter(&self, r_in: &Ray, rec:&RayHit) -> Option<Ray> {
        let mut scatter_direction = rec.normal + unit_samp();

        if near_zero(scatter_direction) {
            scatter_direction = rec.normal;
        }

        let scatter_ray = Ray::new(rec.p, scatter_direction, self.albedo * r_in.color);
        Some(scatter_ray)
    }
}

#[derive(Clone)]
pub struct Metal{
    pub albedo: DVec3,
    pub fuzz: f64
}

impl Material for Metal {
    fn scatter(&self, r_in: &Ray, rec:&RayHit) -> Option<Ray> {
        let reflected = reflect(r_in.direction.normalized(), rec.normal);
        let color = r_in.color * self.albedo;
        let scattered = Ray::new(rec.p, reflected + self.fuzz * unit_samp(), color);
        if scattered.direction.dot(rec.normal) > 0.0 {Some(scattered)} else {None}
    }
}

#[derive(Clone)]
pub struct Dielectric {
    pub ior: f64
}

impl Material for Dielectric {
    fn scatter(&self, r_in: &Ray, rec: &RayHit) -> Option<Ray> {
        let attenuation = DVec3::one();
        let refr_ratio = if rec.front {1.0/self.ior} else {self.ior};
        let unit_direction = r_in.direction.normalized();

        let cos_theta = rec.normal.dot(-unit_direction).min(1.0);
        let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();

        let cannot_refract = refr_ratio * sin_theta > 1.0;

        let direction = if cannot_refract || (reflectance(cos_theta, refr_ratio) > fastrand::f64()) {
            reflect(unit_direction, rec.normal)
        } else {
            refract(unit_direction, rec.normal, refr_ratio)
        };

        let scattered = Ray::new(rec.p, direction, r_in.color * attenuation);
        
        Some(scattered)
    }
}

#[derive(Clone)]
pub struct Emissive {
    pub strength: f64,
    pub color: DVec3
}

impl Material for Emissive {
    fn scatter(&self, r_in: &Ray, rec: &RayHit) -> Option<Ray> {
        let color = self.color * self.strength;
        let mut scattered = Ray::new(rec.p, rec.normal, color);
        scattered.emissive = true;
        Some(scattered)
    }
}