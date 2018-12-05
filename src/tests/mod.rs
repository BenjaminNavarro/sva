#[cfg(test)]
mod tests {
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
}
