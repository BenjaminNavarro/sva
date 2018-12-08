use nalgebra::{Matrix3, Matrix6, Rotation3, UnitQuaternion, Vector3, Vector6, U3};

pub type Vec3 = Vector3<f64>;
pub type Vec6 = Vector6<f64>;
pub type Mat3 = Matrix3<f64>;
pub type Mat6 = Matrix6<f64>;
pub type Rot3 = Rotation3<f64>;
pub type Quat = UnitQuaternion<f64>;

pub mod utility;
pub use self::utility::*;

pub mod force_vec;
pub use self::force_vec::*;

pub mod motion_vec;
pub use self::motion_vec::*;

pub mod admittance_vec;
pub use self::admittance_vec::*;

pub mod impedance_vec;
pub use self::impedance_vec::*;

pub mod ptransform;
pub use self::ptransform::*;

pub fn get_first_vec3(vector: &Vec6) -> Vec3 {
    vector.fixed_rows::<U3>(0).into()
}

pub fn get_second_vec3(vector: &Vec6) -> Vec3 {
    vector.fixed_rows::<U3>(3).into()
}
