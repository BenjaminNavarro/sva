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

/**
 * Compute 1/sinc(x).
 * This code is inspired by boost/math/special_functions/sinc.hpp.
 */
pub fn sinc_inv(x: f64) -> f64 {
    let taylor_0_bound = std::f64::EPSILON;
    let taylor_2_bound = taylor_0_bound.sqrt();
    let taylor_n_bound = taylor_2_bound.sqrt();

    // We use the 4th order taylor series around 0 of x/sin(x) to compute
    // this function:
    //      2      4
    //     x    7⋅x     ⎛ 6⎞
    // 1 + ── + ──── + O⎝x ⎠
    //     6    360
    // this approximation is valid around 0.
    // if x is far from 0, our approximation is not valid
    // since x^6 becomes non negligable we use the normal computation of the function
    // (i.e. taylor_2_bound^6 + taylor_0_bound == taylor_0_bound but
    //       taylor_n_bound^6 + taylor_0_bound != taylor_0).

    if x.abs() >= taylor_n_bound {
        x / x.sin()
    }
    else {
        let mut result = 1.;

        if x.abs() >= taylor_0_bound {
            let x2 = x*x;
            result += x2/6.;

            if x.abs() >= taylor_2_bound {
                result += 7.*(x2*x2)/360.
            }
        }

        result
    }
}
