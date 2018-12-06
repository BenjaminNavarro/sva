use nalgebra::{Vector3, Vector6, U3};

pub type Vec3 = Vector3<f64>;
pub type Vec6 = Vector6<f64>;

pub mod force_vec;
pub use self::force_vec::*;

pub mod motion_vec;
pub use self::motion_vec::*;

pub mod admittance_vec;
pub use self::admittance_vec::*;

pub mod impedance_vec;
pub use self::impedance_vec::*;

pub fn get_first_vec3(vector: &Vec6) -> Vec3 {
    vector.fixed_rows::<U3>(0).into()
}

pub fn get_second_vec3(vector: &Vec6) -> Vec3 {
    vector.fixed_rows::<U3>(3).into()
}
