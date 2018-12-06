use std::fmt;
use std::ops;

use sva::*;

#[derive(Clone, Copy, Debug)]
pub struct ImpedanceVector {
    pub angular: Vec3,
    pub linear: Vec3,
}

impl ImpedanceVector {
    pub fn zero() -> Self {
        Self {
            angular: Vec3::zeros(),
            linear: Vec3::zeros(),
        }
    }

    pub fn new() -> Self {
        ImpedanceVector::zero()
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

    pub fn cross(&self, other: ImpedanceVector) -> ImpedanceVector {
        ImpedanceVector::from_vectors(
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

impl ops::Add<ImpedanceVector> for ImpedanceVector {
    type Output = ImpedanceVector;

    fn add(self, other: ImpedanceVector) -> ImpedanceVector {
        ImpedanceVector::from_vectors(self.angular + other.angular, self.linear + other.linear)
    }
}

impl ops::AddAssign for ImpedanceVector {
    fn add_assign(&mut self, other: ImpedanceVector) {
        *self =
            ImpedanceVector::from_vectors(self.angular + other.angular, self.linear + other.linear)
    }
}

impl ops::Sub<ImpedanceVector> for ImpedanceVector {
    type Output = ImpedanceVector;

    fn sub(self, other: ImpedanceVector) -> ImpedanceVector {
        ImpedanceVector::from_vectors(self.angular - other.angular, self.linear - other.linear)
    }
}

impl ops::SubAssign for ImpedanceVector {
    fn sub_assign(&mut self, other: ImpedanceVector) {
        *self =
            ImpedanceVector::from_vectors(self.angular - other.angular, self.linear - other.linear)
    }
}

impl ops::Neg for ImpedanceVector {
    type Output = ImpedanceVector;

    fn neg(self) -> ImpedanceVector {
        ImpedanceVector::from_vectors(-self.angular, -self.linear)
    }
}

impl ops::Mul<f64> for ImpedanceVector {
    type Output = ImpedanceVector;

    fn mul(self, scalar: f64) -> ImpedanceVector {
        ImpedanceVector::from_vectors(scalar * self.angular, scalar * self.linear)
    }
}

impl ops::Mul<MotionVector> for ImpedanceVector {
    type Output = ForceVector;

    fn mul(self, motion_vector: MotionVector) -> ForceVector {
        ForceVector::from_vectors(
            self.angular.component_mul(&motion_vector.angular),
            self.linear.component_mul(&motion_vector.linear),
        )
    }
}

impl ops::Mul<ImpedanceVector> for MotionVector {
    type Output = ForceVector;

    fn mul(self, impedance_vector: ImpedanceVector) -> ForceVector {
        ForceVector::from_vectors(
            self.angular.component_mul(&impedance_vector.angular),
            self.linear.component_mul(&impedance_vector.linear),
        )
    }
}

impl ops::MulAssign<f64> for ImpedanceVector {
    fn mul_assign(&mut self, scalar: f64) {
        *self = ImpedanceVector::from_vectors(scalar * self.angular, scalar * self.linear)
    }
}

impl ops::Div<f64> for ImpedanceVector {
    type Output = ImpedanceVector;

    fn div(self, scalar: f64) -> ImpedanceVector {
        ImpedanceVector::from_vectors(self.angular / scalar, self.linear / scalar)
    }
}

impl ops::DivAssign<f64> for ImpedanceVector {
    fn div_assign(&mut self, scalar: f64) {
        *self = ImpedanceVector::from_vectors(self.angular / scalar, self.linear / scalar)
    }
}

impl std::cmp::PartialEq for ImpedanceVector {
    fn eq(&self, other: &ImpedanceVector) -> bool {
        self.angular == other.angular && self.linear == other.linear
    }
}

impl fmt::Display for ImpedanceVector {
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
