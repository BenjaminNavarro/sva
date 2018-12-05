#[cfg(test)]
mod tests {
    use sva;
    #[test]
    fn it_works() {
        let mut mvec1 = sva::MotionVector::new();
        let mut mvec2 = sva::MotionVector::zero();
        let mut fvec = sva::ForceVector::zero();
        mvec1.angular[0] = 1.;
        mvec2.linear[2] = -1.;
        fvec.couple[0] = 0.5;
        println!("{}", mvec1);
        println!("{}", mvec2);
        println!("{}", mvec1 + mvec2);
        println!("{}", mvec1.dot(fvec));
    }
}
