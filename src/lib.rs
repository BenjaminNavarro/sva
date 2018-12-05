extern crate nalgebra as na;
extern crate num_traits;

pub mod sva;

#[cfg(test)]
mod tests {
    use sva;
    // use na;
    #[test]
    fn it_works() {
        let mut mvec1 = sva::MotionVector::new();
        let mut mvec2 = sva::MotionVector::zero();
        mvec1.angular[0] = 1.;
        mvec2.linear[2] = -1.;
        println!("{:?}", mvec1);
        println!("{:?}", mvec2);
        println!("{:?}", mvec1+mvec2);
        assert_eq!(2 + 2, 4);
    }
}
