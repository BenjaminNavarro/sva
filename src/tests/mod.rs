#[cfg(test)]
mod tests {
    use std::f64;
    use sva;

    static TOL: f64 = 0.00001;

    #[test]
    fn motion_vector_test() {
        let w = sva::Vec3::new_random();
        let v = sva::Vec3::new_random();

        let vec = sva::MotionVector::from_vectors(w, v);
        let m = vec.vector();

        // angular
        assert_eq!(w, vec.angular);

        // linear
        assert_eq!(v, vec.linear);

        // vector
        assert_eq!(
            m,
            sva::Vec6::new(
                vec.angular[0],
                vec.angular[1],
                vec.angular[2],
                vec.linear[0],
                vec.linear[1],
                vec.linear[2]
            )
        );

        // alpha*M
        assert_eq!((5. * vec).vector(), 5. * m);

        // M*alpha
        assert_eq!((vec * 5.).vector(), 5. * m);

        // M/alpha
        assert_eq!((vec / 5.).vector(), m / 5.);

        // -M
        assert_eq!((-vec).vector(), -m);

        let w2 = sva::Vec3::new_random();
        let v2 = sva::Vec3::new_random();
        let vec2 = sva::MotionVector::from_vectors(w2, v2);
        let m2 = vec2.vector();

        // M + M
        assert_eq!((vec + vec2).vector(), m + m2);

        // M - M
        assert_eq!((vec - vec2).vector(), m - m2);

        // M += M
        let mut vec_pluseq = vec;
        vec_pluseq += vec2;
        assert_eq!(vec_pluseq, vec + vec2);

        // M -= M
        let mut vec_minuseq = vec;
        vec_minuseq -= vec2;
        assert_eq!(vec_minuseq, vec - vec2);

        // ==
        assert_eq!(vec, vec);
        assert_ne!(vec, -vec);

        // !=
        assert!(vec != (-vec));
        assert!(!(vec != vec));

        // zero
        assert_eq!(sva::MotionVector::zero().vector(), sva::Vec6::zeros());
    }

    #[test]
    fn force_vector_test() {
        let n = sva::Vec3::new_random();
        let f = sva::Vec3::new_random();

        let vec = sva::ForceVector::from_vectors(n, f);
        let m = vec.vector();

        // couple
        assert_eq!(n, vec.couple);

        // force
        assert_eq!(f, vec.force);

        // vector
        assert_eq!(
            m,
            sva::Vec6::new(
                vec.couple[0],
                vec.couple[1],
                vec.couple[2],
                vec.force[0],
                vec.force[1],
                vec.force[2]
            )
        );

        // alpha*F
        assert_eq!((5. * vec).vector(), 5. * m);

        // F*alpha
        assert_eq!((vec * 5.).vector(), 5. * m);

        // F/alpha
        assert_eq!((vec / 5.).vector(), m / 5.);

        // -F
        assert_eq!((-vec).vector(), -m);

        let n2 = sva::Vec3::new_random();
        let f2 = sva::Vec3::new_random();
        let vec2 = sva::ForceVector::from_vectors(n2, f2);
        let m2 = vec2.vector();

        // F + F
        assert_eq!((vec + vec2).vector(), m + m2);

        // F - F
        assert_eq!((vec - vec2).vector(), m - m2);

        // F += F
        let mut vec_pluseq = vec;
        vec_pluseq += vec2;
        assert_eq!(vec_pluseq, vec + vec2);

        // F -= F
        let mut vec_minuseq = vec;
        vec_minuseq -= vec2;
        assert_eq!(vec_minuseq, vec - vec2);

        // ==
        assert_eq!(vec, vec);
        assert_ne!(vec, -vec);

        // !=
        assert!(vec != (-vec));
        assert!(!(vec != vec));

        // zero
        assert_eq!(sva::ForceVector::zero().vector(), sva::Vec6::zeros());
    }

    #[test]
    fn motion_vector_left_operators_test() {
        let w = sva::Vec3::new_random() * 100.;
        let v = sva::Vec3::new_random() * 100.;
        let n = sva::Vec3::new_random() * 100.;
        let f = sva::Vec3::new_random() * 100.;

        let m_vec = sva::MotionVector::from_vectors(w, v);
        let f_vec = sva::ForceVector::from_vectors(n, f);
        let mm = m_vec.vector();
        let mf = f_vec.vector();

        // dot(MotionVecd, ForceVecd)
        assert!((m_vec.dot(f_vec) - (mm.transpose() * mf)[0]).abs() < TOL);

        // cross(MotionVecd, MotionVecd)
        let w2 = sva::Vec3::new_random() * 100.;
        let v2 = sva::Vec3::new_random() * 100.;
        let m_vec2 = sva::MotionVector::from_vectors(w2, v2);
        let mm2 = m_vec2.vector();

        let cross_m = m_vec.cross(m_vec2);
        assert!((cross_m.vector() - sva::vector6_to_cross_matrix(&mm) * mm2).norm() < TOL);

        // crossDual(MotionVecd, ForceVecd)
        let cross_f = m_vec.cross_dual(f_vec);
        assert!((cross_f.vector() - sva::vector6_to_cross_dual_matrix(&mm) * mf).norm() < TOL);
    }

    #[test]
    fn impedance_vector_test() {
        let w = sva::Vec3::new_random();
        let v = sva::Vec3::new_random();

        let vec = sva::ImpedanceVector::from_vectors(w, v);
        let z = vec.vector();

        // angular
        assert_eq!(w, vec.angular);

        // linear
        assert_eq!(v, vec.linear);

        // vector
        assert_eq!(
            z,
            sva::Vec6::new(
                vec.angular[0],
                vec.angular[1],
                vec.angular[2],
                vec.linear[0],
                vec.linear[1],
                vec.linear[2]
            )
        );

        // alpha*M
        assert_eq!((5. * vec).vector(), 5. * z);

        // M*alpha
        assert_eq!((vec * 5.).vector(), 5. * z);

        // M/alpha
        assert_eq!((vec / 5.).vector(), z / 5.);

        // ==
        assert_eq!(vec, vec);
        assert_ne!(vec, -vec);

        // !=
        assert!(vec != (-vec));
        assert!(!(vec != vec));

        // Copy
        let mut vec_tmp = vec;
        assert_eq!(vec, vec_tmp);

        // *= alpha
        vec_tmp *= 5.;
        assert_eq!(vec_tmp.vector(), 5. * z);

        // /= alpha
        vec_tmp /= 5.;
        assert!((vec_tmp - vec).vector().norm() < TOL);

        // -M
        assert_eq!((-vec).vector(), -z);

        let w2 = sva::Vec3::new_random();
        let v2 = sva::Vec3::new_random();
        let vec2 = sva::ImpedanceVector::from_vectors(w2, v2);
        let z2 = vec2.vector();

        // M + M
        assert_eq!((vec + vec2).vector(), z + z2);

        // M - M
        assert_eq!((vec - vec2).vector(), z - z2);

        // M += M
        let mut vec_pluseq = vec;
        vec_pluseq += vec2;
        assert_eq!(vec_pluseq, vec + vec2);

        // M -= M
        let mut vec_minuseq = vec;
        vec_minuseq -= vec2;
        assert_eq!(vec_minuseq, vec - vec2);

        // operator *
        let mv = sva::MotionVector::from_vector(sva::Vec6::new_random());
        let fv = vec * mv;
        assert_eq!(fv.force, vec.linear.component_mul(&mv.linear));
        assert_eq!(fv.couple, vec.angular.component_mul(&mv.angular));

        let fv2 = mv * vec;
        assert_eq!(fv, fv2);

        // homogeneous constructor
        let hiv = sva::ImpedanceVector::from_scalars(11., 42.);
        assert_eq!(hiv.angular, sva::Vec3::repeat(11.));
        assert_eq!(hiv.linear, sva::Vec3::repeat(42.));

        // zero
        assert_eq!(sva::ImpedanceVector::zero().vector(), sva::Vec6::zeros());
    }

    #[test]
    fn admittance_vector_test() {
        let w = sva::Vec3::new_random();
        let v = sva::Vec3::new_random();

        let vec = sva::AdmittanceVector::from_vectors(w, v);
        let a = vec.vector();

        // angular
        assert_eq!(w, vec.angular);

        // linear
        assert_eq!(v, vec.linear);

        // vector
        assert_eq!(
            a,
            sva::Vec6::new(
                vec.angular[0],
                vec.angular[1],
                vec.angular[2],
                vec.linear[0],
                vec.linear[1],
                vec.linear[2]
            )
        );

        // alpha*M
        assert_eq!((5. * vec).vector(), 5. * a);

        // M*alpha
        assert_eq!((vec * 5.).vector(), 5. * a);

        // M/alpha
        assert_eq!((vec / 5.).vector(), a / 5.);

        // ==
        assert_eq!(vec, vec);
        assert_ne!(vec, -vec);

        // !=
        assert!(vec != (-vec));
        assert!(!(vec != vec));

        // Copy
        let mut vec_tmp = vec;
        assert_eq!(vec, vec_tmp);

        // *= alpha
        vec_tmp *= 5.;
        assert_eq!(vec_tmp.vector(), 5. * a);

        // /= alpha
        vec_tmp /= 5.;
        assert!((vec_tmp - vec).vector().norm() < TOL);

        // -M
        assert_eq!((-vec).vector(), -a);

        let w2 = sva::Vec3::new_random();
        let v2 = sva::Vec3::new_random();
        let vec2 = sva::AdmittanceVector::from_vectors(w2, v2);
        let a2 = vec2.vector();

        // M + M
        assert_eq!((vec + vec2).vector(), a + a2);

        // M - M
        assert_eq!((vec - vec2).vector(), a - a2);

        // M += M
        let mut vec_pluseq = vec;
        vec_pluseq += vec2;
        assert_eq!(vec_pluseq, vec + vec2);

        // M -= M
        let mut vec_minuseq = vec;
        vec_minuseq -= vec2;
        assert_eq!(vec_minuseq, vec - vec2);

        // operator *
        let fv = sva::ForceVector::from_vector(sva::Vec6::new_random());
        let mv = vec * fv;
        assert_eq!(mv.linear, vec.linear.component_mul(&fv.force));
        assert_eq!(mv.angular, vec.angular.component_mul(&fv.couple));

        let mv2 = fv * vec;
        assert_eq!(mv, mv2);

        // homogeneous constructor
        let hav = sva::AdmittanceVector::from_scalars(11., 42.);
        assert_eq!(hav.angular, sva::Vec3::repeat(11.));
        assert_eq!(hav.linear, sva::Vec3::repeat(42.));

        // zero
        assert_eq!(sva::AdmittanceVector::zero().vector(), sva::Vec6::zeros());
    }
}
