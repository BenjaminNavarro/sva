use std::fmt;

use sva::*;

#[derive(Clone, Copy, Debug)]
pub struct RBInertia {
    pub mass: f64,
    pub momentum: Vec3,
    inertia: Mat3,
}

impl RBInertia {
    // The inertia matrix lower triangle will be extracted
    pub fn new(mass: f64, momentum: Vec3, inertia: Mat3) -> Self {
        let rbi = Self {
            mass: mass,
            momentum: momentum,
            inertia: Mat3::zeros(),
        };
        rbi.inertia
            .lower_triangle()
            .copy_from(&inertia.lower_triangle());
        rbi
    }

    // The inertia matrix is expected to have zeros on its upper triangle
    pub fn from_lower_triangle(mass: f64, momentum: Vec3, inertia: Mat3) -> Self {
        Self {
            mass: mass,
            momentum: momentum,
            inertia: inertia,
        }
    }

    pub fn lower_triangular_inertia(&self) -> Mat3 {
        self.inertia
    }

    pub fn inertia(&self) -> Mat3 {
        let inertia = self.inertia;
        inertia
            .upper_triangle()
            .copy_from(&self.inertia.transpose());
        inertia
    }
}

impl fmt::Display for RBInertia {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.inertia)
    }
}

pub fn inertia_to_origin(inertia: &Mat3, mass: f64, com: &Vec3, rotation: &Rot3) -> Mat3 {
    let trans = vector3_to_cross_matrix(&(mass * com)) * vector3_to_cross_matrix(com).transpose();
    rotation.matrix() * (inertia + trans) * rotation.matrix().transpose()
}
