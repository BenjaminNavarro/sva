use nalgebra::{Vector3, Vector6};

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

mod sva {}
