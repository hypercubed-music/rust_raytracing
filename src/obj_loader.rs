use std::fs::File;
use std::io::BufReader;
use obj::{load_obj, Obj};
use ultraviolet::DVec3;

#[derive(Clone, Debug)]
pub struct MeshTriangle {
    pub pos1: DVec3,
    pub pos2: DVec3,
    pub pos3: DVec3,
    pub norm1: DVec3,
    pub norm2: DVec3,
    pub norm3: DVec3,
}

pub fn load_mesh(filename: &str) -> Vec<MeshTriangle> {
    let file = BufReader::new(File::open(filename).expect("Couldn't open file"));
    let object: Obj = load_obj(file).expect("couldn't load an object from this file");

    let mut tri_list: Vec<MeshTriangle> = vec![];
    for i in (0..object.indices.len()).step_by(3) {
        let tri = MeshTriangle {
            pos1: DVec3::from(object.vertices[object.indices[i] as usize].position.map(|e| e as f64)),
            pos2: DVec3::from(object.vertices[object.indices[i+1] as usize].position.map(|e| e as f64)),
            pos3: DVec3::from(object.vertices[object.indices[i+2] as usize].position.map(|e| e as f64)),
            norm1: DVec3::from(object.vertices[object.indices[i] as usize].normal.map(|e| e as f64)),
            norm2: DVec3::from(object.vertices[object.indices[i+1] as usize].normal.map(|e| e as f64)),
            norm3: DVec3::from(object.vertices[object.indices[i+2] as usize].normal.map(|e| e as f64))
        };
        tri_list.push(tri);
    }
    tri_list
}