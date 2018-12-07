use sva::*;

pub struct PTransform {
    rotation: Mat3,
    translation: Vec3
}

impl PTransform {
    pub fn from_mat_vec(rot: Mat3, trans: Vec3) -> Self {
        Self{rotation: rot, translation: trans}
    }

    pub fn from_mat(rot: Mat3) -> Self {
        Self{rotation: rot, translation: Vec3::zeros()}
    }

    pub fn from_quat_vec(rot: Quat, trans: Vec3) -> Self {
        Self{rotation: rot.to_rotation_matrix().into(), translation: trans}
    }

    pub fn from_quat(rot: Quat) -> Self {
        Self{rotation: rot.to_rotation_matrix().into(), translation: Vec3::zeros()}
    }

    pub fn from_vec(trans: Vec3) -> Self {
        Self{rotation: Quat::identity().to_rotation_matrix().into(), translation: trans}
    }

    pub fn matrix(&self) -> Mat6 {
        let mut m = Mat6::zeros();
        m.fixed_slice_mut::<U3,U3>(0,0).copy_from(&self.rotation);
        m.fixed_slice_mut::<U3,U3>(3,0).copy_from(&(-self.rotation*vector3_to_cross_matrix(&self.translation)));
        m.fixed_slice_mut::<U3,U3>(3,3).copy_from(&self.rotation);
        m
    }

    pub fn dual_matrix(&self) -> Mat6 {
        let mut m = Mat6::zeros();
        m.fixed_slice_mut::<U3,U3>(0,0).copy_from(&self.rotation);
        m.fixed_slice_mut::<U3,U3>(0,3).copy_from(&(-self.rotation*vector3_to_cross_matrix(&self.translation)));
        m.fixed_slice_mut::<U3,U3>(3,3).copy_from(&self.rotation);
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
            self.rotation.transpose()*mv.angular,
            self.rotation.transpose()*mv.linear
        );
        ret.linear += self.translation.cross(&ret.angular);
        ret
    }

    pub fn angular_inv_mul(&self, mv: &MotionVector) -> Vec3 {
        self.rotation.transpose()*mv.angular
    }

    pub fn linear_inv_mul(&self, mv: &MotionVector) -> Vec3 {
        self.rotation.transpose()*mv.linear + self.translation.cross(&(self.rotation.transpose()*mv.angular))
    }
}

pub fn rot_x(theta: f64) -> Mat3 {
    let s = theta.sin();
    let c = theta.cos();
    Mat3::new(1., 0., 0., 0., c, s, 0., -s, c)
}

pub fn rot_y(theta: f64) -> Mat3 {
    let s = theta.sin();
    let c = theta.cos();
    Mat3::new(c, 0., -s, 0., 1., 0., s, 0., c)
}

pub fn rot_z(theta: f64) -> Mat3 {
    let s = theta.sin();
    let c = theta.cos();
    Mat3::new(c, s, 0., -s, c, 0., 0., 0., 1.)
}

#[allow(non_snake_case)]
pub fn rotation_velocity(E_a_b: &Mat3) -> Vec3 {
    let mut w: Vec3;
    let acos_v = (E_a_b[(0,0)] + E_a_b[(1,1)] + E_a_b[(2,2)] - 1.) * 0.5;
    let theta = acos_v.max(-1.).min(1.).acos();

    w = Vec3::new(-E_a_b[(2,1)] + E_a_b[(1,2)], -E_a_b[(0,2)] + E_a_b[(2,0)], -E_a_b[(1,0)] + E_a_b[(0,1)]);
    w *= sinc_inv(theta)*0.5;
    w
}

#[allow(non_snake_case)]
pub fn rotation_error(E_a_b: &Mat3, E_a_c: &Mat3) -> Vec3 {
    let E_b_c = E_a_c * E_a_b.transpose();
    E_a_b.transpose() * rotation_velocity(&E_b_c)
}
