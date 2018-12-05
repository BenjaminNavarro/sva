use std::ops;

use na::FiniteDimVectorSpace;

type Vector3 = FiniteDimVectorSpace<nalgebra::U3>;
type Vector6 = FiniteDimVectorSpace<nalgebra::U6>;

#[derive(Clone, Copy, Debug)]
pub struct MotionVector {
    pub angular: Vector3,
    pub linear: Vector3,
}

impl MotionVector {
    pub fn zero() -> Self {
        Self {
            angular: Vector3::zeros(),
            linear: Vector3::zeros(),
        }
    }

    pub fn new() -> Self {
        MotionVector::zero()
    }

    pub fn from_vector(vector: Vector6) -> Self {
        Self {
            angular: Vector3::new(vector[0], vector[1], vector[2]),
            linear: Vector3::new(vector[3], vector[4], vector[5]),
        }
    }

    pub fn from_vectors(angular: Vector3, linear: Vector3) -> Self {
        Self {
            angular: angular,
            linear: linear,
        }
    }

    pub fn vector(&self) -> Vector6 {
        Vector6::new(
            self.angular[0],
            self.angular[1],
            self.angular[2],
            self.linear[0],
            self.linear[1],
            self.linear[2],
        )
    }
}

impl ops::Add<MotionVector> for MotionVector {
    type Output = MotionVector;

    fn add(self, other: MotionVector) -> MotionVector {
        MotionVector::from_vectors(self.angular + other.angular, self.linear + other.linear)
    }
}
