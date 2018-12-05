use std::fmt;
use std::ops;

use sva::ForceVector;
use sva::{Vec3, Vec6};

#[derive(Clone, Copy, Debug)]
pub struct MotionVector {
    pub angular: Vec3,
    pub linear: Vec3,
}

impl MotionVector {
    pub fn zero() -> Self {
        Self {
            angular: Vec3::zeros(),
            linear: Vec3::zeros(),
        }
    }

    pub fn new() -> Self {
        MotionVector::zero()
    }

    pub fn from_vector(vector: Vec6) -> Self {
        Self {
            angular: Vec3::new(vector[0], vector[1], vector[2]),
            linear: Vec3::new(vector[3], vector[4], vector[5]),
        }
    }

    pub fn from_vectors(angular: Vec3, linear: Vec3) -> Self {
        Self {
            angular: angular,
            linear: linear,
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

    pub fn cross(&self, other: MotionVector) -> MotionVector {
        MotionVector::from_vectors(
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

impl ops::Add<MotionVector> for MotionVector {
    type Output = MotionVector;

    fn add(self, other: MotionVector) -> MotionVector {
        MotionVector::from_vectors(self.angular + other.angular, self.linear + other.linear)
    }
}

impl ops::AddAssign for MotionVector {
    fn add_assign(&mut self, other: MotionVector) {
        *self = MotionVector::from_vectors(self.angular + other.angular, self.linear + other.linear)
    }
}

impl ops::Sub<MotionVector> for MotionVector {
    type Output = MotionVector;

    fn sub(self, other: MotionVector) -> MotionVector {
        MotionVector::from_vectors(self.angular - other.angular, self.linear - other.linear)
    }
}

impl ops::SubAssign for MotionVector {
    fn sub_assign(&mut self, other: MotionVector) {
        *self = MotionVector::from_vectors(self.angular - other.angular, self.linear - other.linear)
    }
}

impl ops::Neg for MotionVector {
    type Output = MotionVector;

    fn neg(self) -> MotionVector {
        MotionVector::from_vectors(-self.angular, -self.linear)
    }
}

impl ops::Mul<f64> for MotionVector {
    type Output = MotionVector;

    fn mul(self, scalar: f64) -> MotionVector {
        MotionVector::from_vectors(scalar * self.angular, scalar * self.linear)
    }
}

impl ops::Mul<MotionVector> for f64 {
    type Output = MotionVector;

    fn mul(self, motion_vector: MotionVector) -> MotionVector {
        motion_vector * self
    }
}

impl ops::MulAssign<f64> for MotionVector {
    fn mul_assign(&mut self, scalar: f64) {
        *self = MotionVector::from_vectors(scalar * self.angular, scalar * self.linear)
    }
}

impl ops::Div<f64> for MotionVector {
    type Output = MotionVector;

    fn div(self, scalar: f64) -> MotionVector {
        MotionVector::from_vectors(self.angular / scalar, self.linear / scalar)
    }
}

impl ops::DivAssign<f64> for MotionVector {
    fn div_assign(&mut self, scalar: f64) {
        *self = MotionVector::from_vectors(self.angular / scalar, self.linear / scalar)
    }
}

impl std::cmp::PartialEq for MotionVector {
    fn eq(&self, other: &MotionVector) -> bool {
        self.angular == other.angular && self.linear == other.linear
    }
}

impl fmt::Display for MotionVector {
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
