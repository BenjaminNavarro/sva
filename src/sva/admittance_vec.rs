use std::fmt;
use std::ops;

use sva::*;

#[derive(Clone, Copy, Debug)]
pub struct AdmittanceVector {
    pub angular: Vec3,
    pub linear: Vec3,
}

impl AdmittanceVector {
    pub fn zero() -> Self {
        Self {
            angular: Vec3::zeros(),
            linear: Vec3::zeros(),
        }
    }

    pub fn new() -> Self {
        AdmittanceVector::zero()
    }

    pub fn from_vector(vector: Vec6) -> Self {
        Self {
            angular: get_first_vec3(&vector),
            linear: get_second_vec3(&vector),
        }
    }

    pub fn from_vectors(angular: Vec3, linear: Vec3) -> Self {
        Self {
            angular: angular,
            linear: linear,
        }
    }

    pub fn from_scalars(angular: f64, linear: f64) -> Self {
        Self {
            angular: Vec3::new(angular, angular, angular),
            linear: Vec3::new(linear, linear, linear),
        }
    }

    pub fn vector(&self) -> Vec6 {
        Vec6::new(
            self.angular[0],
            self.angular[1],
            self.angular[2],
            self.linear[0],
            self.linear[1],
            self.linear[2],
        )
    }

    pub fn cross(&self, other: AdmittanceVector) -> AdmittanceVector {
        AdmittanceVector::from_vectors(
            self.angular.cross(&other.angular),
            self.angular.cross(&other.linear) + self.linear.cross(&other.angular),
        )
    }

    pub fn cross_dual(&self, other: ForceVector) -> ForceVector {
        ForceVector::from_vectors(
            self.angular.cross(&other.couple) + self.linear.cross(&other.force),
            self.angular.cross(&other.force),
        )
    }

    pub fn dot(&self, other: ForceVector) -> f64 {
        self.angular.dot(&other.couple) + self.linear.dot(&other.force)
    }
}

impl ops::Add<AdmittanceVector> for AdmittanceVector {
    type Output = AdmittanceVector;

    fn add(self, other: AdmittanceVector) -> AdmittanceVector {
        AdmittanceVector::from_vectors(self.angular + other.angular, self.linear + other.linear)
    }
}

impl ops::AddAssign for AdmittanceVector {
    fn add_assign(&mut self, other: AdmittanceVector) {
        *self =
            AdmittanceVector::from_vectors(self.angular + other.angular, self.linear + other.linear)
    }
}

impl ops::Sub<AdmittanceVector> for AdmittanceVector {
    type Output = AdmittanceVector;

    fn sub(self, other: AdmittanceVector) -> AdmittanceVector {
        AdmittanceVector::from_vectors(self.angular - other.angular, self.linear - other.linear)
    }
}

impl ops::SubAssign for AdmittanceVector {
    fn sub_assign(&mut self, other: AdmittanceVector) {
        *self =
            AdmittanceVector::from_vectors(self.angular - other.angular, self.linear - other.linear)
    }
}

impl ops::Neg for AdmittanceVector {
    type Output = AdmittanceVector;

    fn neg(self) -> AdmittanceVector {
        AdmittanceVector::from_vectors(-self.angular, -self.linear)
    }
}

impl ops::Mul<f64> for AdmittanceVector {
    type Output = AdmittanceVector;

    fn mul(self, scalar: f64) -> AdmittanceVector {
        AdmittanceVector::from_vectors(scalar * self.angular, scalar * self.linear)
    }
}

impl ops::Mul<AdmittanceVector> for f64 {
    type Output = AdmittanceVector;

    fn mul(self, admittance_vector: AdmittanceVector) -> AdmittanceVector {
        admittance_vector * self
    }
}

impl ops::Mul<ForceVector> for AdmittanceVector {
    type Output = MotionVector;

    fn mul(self, force_vector: ForceVector) -> MotionVector {
        MotionVector::from_vectors(
            self.angular.component_mul(&force_vector.couple),
            self.linear.component_mul(&force_vector.force),
        )
    }
}

impl ops::Mul<AdmittanceVector> for ForceVector {
    type Output = MotionVector;

    fn mul(self, admittance_vector: AdmittanceVector) -> MotionVector {
        MotionVector::from_vectors(
            self.couple.component_mul(&admittance_vector.angular),
            self.force.component_mul(&admittance_vector.linear),
        )
    }
}

impl ops::MulAssign<f64> for AdmittanceVector {
    fn mul_assign(&mut self, scalar: f64) {
        *self = AdmittanceVector::from_vectors(scalar * self.angular, scalar * self.linear)
    }
}

impl ops::Div<f64> for AdmittanceVector {
    type Output = AdmittanceVector;

    fn div(self, scalar: f64) -> AdmittanceVector {
        AdmittanceVector::from_vectors(self.angular / scalar, self.linear / scalar)
    }
}

impl ops::DivAssign<f64> for AdmittanceVector {
    fn div_assign(&mut self, scalar: f64) {
        *self = AdmittanceVector::from_vectors(self.angular / scalar, self.linear / scalar)
    }
}

impl std::cmp::PartialEq for AdmittanceVector {
    fn eq(&self, other: &AdmittanceVector) -> bool {
        self.angular == other.angular && self.linear == other.linear
    }
}

impl fmt::Display for AdmittanceVector {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "(angular: [{ax} {ay} {az}], linear: [{lx} {ly} {lz}])",
            ax = self.angular[0],
            ay = self.angular[1],
            az = self.angular[2],
            lx = self.linear[0],
            ly = self.linear[1],
            lz = self.linear[2],
        )
    }
}
