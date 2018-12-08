use std::fmt;
use std::ops;

use sva::*;

pub struct PTransform {
    rotation: Rot3,
    translation: Vec3,
}

impl PTransform {
    pub fn identity() -> Self {
        Self {
            rotation: Rot3::identity(),
            translation: Vec3::zeros(),
        }
    }

    pub fn from_mat_vec(rot: Rot3, trans: Vec3) -> Self {
        Self {
            rotation: rot,
            translation: trans,
        }
    }

    pub fn from_mat(rot: Rot3) -> Self {
        Self {
            rotation: rot,
            translation: Vec3::zeros(),
        }
    }

    pub fn from_quat_vec(rot: Quat, trans: Vec3) -> Self {
        Self {
            rotation: rot.to_rotation_matrix().into(),
            translation: trans,
        }
    }

    pub fn from_quat(rot: Quat) -> Self {
        Self {
            rotation: rot.to_rotation_matrix().into(),
            translation: Vec3::zeros(),
        }
    }

    pub fn from_vec(trans: Vec3) -> Self {
        Self {
            rotation: Quat::identity().to_rotation_matrix().into(),
            translation: trans,
        }
    }

    pub fn matrix(&self) -> Mat6 {
        let mut m = Mat6::zeros();

        m.fixed_slice_mut::<U3, U3>(0, 0)
            .copy_from(&self.rotation.matrix());

        m.fixed_slice_mut::<U3, U3>(3, 0)
            .copy_from(&(-self.rotation.matrix() * vector3_to_cross_matrix(&self.translation)));

        m.fixed_slice_mut::<U3, U3>(3, 3)
            .copy_from(&self.rotation.matrix());
        m
    }

    pub fn dual_matrix(&self) -> Mat6 {
        let mut m = Mat6::zeros();

        m.fixed_slice_mut::<U3, U3>(0, 0)
            .copy_from(&self.rotation.matrix());

        m.fixed_slice_mut::<U3, U3>(0, 3)
            .copy_from(&(-self.rotation.matrix() * vector3_to_cross_matrix(&self.translation)));

        m.fixed_slice_mut::<U3, U3>(3, 3)
            .copy_from(&self.rotation.matrix());
        m
    }

    pub fn angular_mul(&self, mv: &MotionVector) -> Vec3 {
        self.rotation * mv.angular
    }

    pub fn linear_mul(&self, mv: &MotionVector) -> Vec3 {
        self.rotation * (mv.linear - self.translation.cross(&mv.angular))
    }

    pub fn inv_mul(&self, mv: &MotionVector) -> MotionVector {
        let mut ret = MotionVector::from_vectors(
            self.rotation.transpose() * mv.angular,
            self.rotation.transpose() * mv.linear,
        );
        ret.linear += self.translation.cross(&ret.angular);
        ret
    }

    pub fn angular_inv_mul(&self, mv: &MotionVector) -> Vec3 {
        self.rotation.transpose() * mv.angular
    }

    pub fn linear_inv_mul(&self, mv: &MotionVector) -> Vec3 {
        self.rotation.transpose() * mv.linear + self
            .translation
            .cross(&(self.rotation.transpose() * mv.angular))
    }

    pub fn dual_mul(&self, fv: &ForceVector) -> ForceVector {
        ForceVector::from_vectors(self.couple_dual_mul(fv), self.force_dual_mul(fv))
    }

    pub fn couple_dual_mul(&self, fv: &ForceVector) -> Vec3 {
        self.rotation.matrix() * (fv.couple - self.translation.cross(&fv.force))
    }

    pub fn force_dual_mul(&self, fv: &ForceVector) -> Vec3 {
        self.rotation.matrix() * fv.force
    }

    pub fn trans_mul(&self, fv: &ForceVector) -> ForceVector {
        let mut ret = ForceVector::from_vectors(
            self.rotation.transpose() * fv.couple,
            self.rotation.transpose() * fv.force,
        );
        ret.couple += self.translation.cross(&ret.force);
        ret
    }

    pub fn couple_trans_mul(&self, fv: &ForceVector) -> Vec3 {
        self.rotation.transpose() * fv.couple + self
            .translation
            .cross(&(self.rotation.transpose() * fv.force))
    }

    pub fn force_trans_mul(&self, fv: &ForceVector) -> Vec3 {
        self.rotation.transpose() * fv.force
    }

    // TODO functions on inertia

    pub fn inv(&self) -> Self {
        Self {
            rotation: self.rotation.transpose(),
            translation: -self.rotation.matrix() * self.translation,
        }
    }
}

impl ops::Mul<PTransform> for PTransform {
    type Output = PTransform;

    fn mul(self, other: PTransform) -> PTransform {
        PTransform {
            rotation: self.rotation * other.rotation,
            translation: other.translation + other.rotation.transpose() * self.translation,
        }
    }
}

impl<'a, 'b> ops::Mul<&'b PTransform> for &'a PTransform {
    type Output = PTransform;

    fn mul(self, other: &'b PTransform) -> PTransform {
        PTransform {
            rotation: self.rotation * other.rotation,
            translation: other.translation + other.rotation.transpose() * self.translation,
        }
    }
}

impl ops::Mul<MotionVector> for PTransform {
    type Output = MotionVector;

    fn mul(self, mv: MotionVector) -> MotionVector {
        MotionVector::from_vectors(self.angular_mul(&mv), self.linear_mul(&mv))
    }
}

impl<'a, 'b> ops::Mul<&'b MotionVector> for &'a PTransform {
    type Output = MotionVector;

    fn mul(self, mv: &'b MotionVector) -> MotionVector {
        MotionVector::from_vectors(self.angular_mul(&mv), self.linear_mul(&mv))
    }
}

impl std::cmp::PartialEq for PTransform {
    fn eq(&self, other: &PTransform) -> bool {
        self.rotation == other.rotation && self.translation == other.translation
    }
}

impl fmt::Display for PTransform {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.matrix())
    }
}

pub fn rot_x(theta: f64) -> Rot3 {
    let s = theta.sin();
    let c = theta.cos();
    Rot3::from_matrix_unchecked(Mat3::new(1., 0., 0., 0., c, s, 0., -s, c))
}

pub fn rot_y(theta: f64) -> Rot3 {
    let s = theta.sin();
    let c = theta.cos();
    Rot3::from_matrix_unchecked(Mat3::new(c, 0., -s, 0., 1., 0., s, 0., c))
}

pub fn rot_z(theta: f64) -> Rot3 {
    let s = theta.sin();
    let c = theta.cos();
    Rot3::from_matrix_unchecked(Mat3::new(c, s, 0., -s, c, 0., 0., 0., 1.))
}

#[allow(non_snake_case)]
pub fn rotation_velocity(E_a_b: &Rot3) -> Vec3 {
    let mut w: Vec3;
    let acos_v = (E_a_b[(0, 0)] + E_a_b[(1, 1)] + E_a_b[(2, 2)] - 1.) * 0.5;
    let theta = acos_v.max(-1.).min(1.).acos();

    w = Vec3::new(
        -E_a_b[(2, 1)] + E_a_b[(1, 2)],
        -E_a_b[(0, 2)] + E_a_b[(2, 0)],
        -E_a_b[(1, 0)] + E_a_b[(0, 1)],
    );
    w *= sinc_inv(theta) * 0.5;
    w
}

#[allow(non_snake_case)]
pub fn rotation_error(E_a_b: &Rot3, E_a_c: &Rot3) -> Vec3 {
    let E_b_c = E_a_c * E_a_b.transpose();
    E_a_b.transpose() * rotation_velocity(&E_b_c)
}

#[allow(non_snake_case)]
pub fn transform_velocity(X_a_b: &PTransform) -> MotionVector {
    MotionVector::from_vectors(rotation_velocity(&X_a_b.rotation), X_a_b.translation)
}

#[allow(non_snake_case)]
pub fn transform_error(X_a_b: &PTransform, X_a_c: &PTransform) -> MotionVector {
    let X_b_c = X_a_c * &X_a_b.inv();
    PTransform::from_mat(X_a_b.rotation.transpose()) * transform_velocity(&X_b_c)
}

// interpolate between transformations, t must be between 0 and 1
pub fn interpolate(from: &PTransform, to: &PTransform, t: f64) -> PTransform {
    let q_from = Quat::from_rotation_matrix(&from.rotation);
    let q_to = Quat::from_rotation_matrix(&to.rotation);
    PTransform::from_quat_vec(
        q_from.slerp(&q_to, t),
        from.translation * t + to.translation * (1. - t),
    )
}
