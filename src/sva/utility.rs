use nalgebra::U3;
use sva::*;

pub fn vector3_to_cross_matrix(vec: &Vec3) -> Mat3 {
    Mat3::new(
        0., -vec[2], vec[1], vec[2], 0., -vec[0], -vec[1], vec[0], 0.,
    )
}

pub fn vector6_to_cross_matrix(vec: &Vec6) -> Mat6 {
    let mut mat = Mat6::zeros();
    let c13 = vector3_to_cross_matrix(&get_first_vec3(vec));
    let c31 = vector3_to_cross_matrix(&get_second_vec3(vec));

    mat.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&c13);
    mat.fixed_slice_mut::<U3, U3>(3, 0).copy_from(&c31);
    mat.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&c13);
    mat
}

pub fn vector6_to_cross_dual_matrix(vec: &Vec6) -> Mat6 {
    -vector6_to_cross_matrix(vec).transpose()
}
