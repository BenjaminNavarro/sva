use std::fmt;
use std::ops;

use super::{Vec3, Vec6};

#[derive(Clone, Copy, Debug)]
pub struct ForceVector {
    pub couple: Vec3,
    pub force: Vec3,
}

impl ForceVector {
    pub fn zero() -> Self {
        Self {
            couple: Vec3::zeros(),
            force: Vec3::zeros(),
        }
    }

    pub fn new() -> Self {
        ForceVector::zero()
    }

    pub fn from_vector(vector: Vec6) -> Self {
        Self {
            couple: Vec3::new(vector[0], vector[1], vector[2]),
            force: Vec3::new(vector[3], vector[4], vector[5]),
        }
    }

    pub fn from_vectors(couple: Vec3, force: Vec3) -> Self {
        Self {
            couple: couple,
            force: force,
        }
    }

    pub fn vector(&self) -> Vec6 {
        Vec6::new(
            self.couple[0],
            self.couple[1],
            self.couple[2],
            self.force[0],
            self.force[1],
            self.force[2],
        )
    }
}

impl ops::Add<ForceVector> for ForceVector {
    type Output = ForceVector;

    fn add(self, other: ForceVector) -> ForceVector {
        ForceVector::from_vectors(self.couple + other.couple, self.force + other.force)
    }
}

impl ops::AddAssign for ForceVector {
    fn add_assign(&mut self, other: ForceVector) {
        *self = ForceVector::from_vectors(self.couple + other.couple, self.force + other.force)
    }
}

impl ops::Sub<ForceVector> for ForceVector {
    type Output = ForceVector;

    fn sub(self, other: ForceVector) -> ForceVector {
        ForceVector::from_vectors(self.couple - other.couple, self.force - other.force)
    }
}

impl ops::SubAssign for ForceVector {
    fn sub_assign(&mut self, other: ForceVector) {
        *self = ForceVector::from_vectors(self.couple - other.couple, self.force - other.force)
    }
}

impl ops::Neg for ForceVector {
    type Output = ForceVector;

    fn neg(self) -> ForceVector {
        ForceVector::from_vectors(-self.couple, -self.force)
    }
}

impl ops::Mul<f64> for ForceVector {
    type Output = ForceVector;

    fn mul(self, scalar: f64) -> ForceVector {
        ForceVector::from_vectors(scalar * self.couple, scalar * self.force)
    }
}

impl ops::Mul<ForceVector> for f64 {
    type Output = ForceVector;

    fn mul(self, motion_vector: ForceVector) -> ForceVector {
        motion_vector * self
    }
}

impl ops::MulAssign<f64> for ForceVector {
    fn mul_assign(&mut self, scalar: f64) {
        *self = ForceVector::from_vectors(scalar * self.couple, scalar * self.force)
    }
}

impl ops::Div<f64> for ForceVector {
    type Output = ForceVector;

    fn div(self, scalar: f64) -> ForceVector {
        ForceVector::from_vectors(self.couple / scalar, self.force / scalar)
    }
}

impl ops::DivAssign<f64> for ForceVector {
    fn div_assign(&mut self, scalar: f64) {
        *self = ForceVector::from_vectors(self.couple / scalar, self.force / scalar)
    }
}

impl std::cmp::PartialEq for ForceVector {
    fn eq(&self, other: &ForceVector) -> bool {
        self.couple == other.couple && self.force == other.force
    }
}

impl fmt::Display for ForceVector {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "(couple: [{ax} {ay} {az}], force: [{lx} {ly} {lz}])",
            ax = self.couple[0],
            ay = self.couple[1],
            az = self.couple[2],
            lx = self.force[0],
            ly = self.force[1],
            lz = self.force[2],
        )
    }
}
