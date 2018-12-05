extern crate nalgebra as na;
extern crate num_traits;

pub mod sva;

#[cfg(test)]
mod tests {
    use sva;
    // use na;
    #[test]
    fn it_works() {
        let mvec1 = sva::MotionVectorf64::new();
        let mvec2 = sva::MotionVectorf64::zero();
        println!("{:?}", mvec1);
        println!("{:?}", mvec2);
        assert_eq!(2 + 2, 4);
    }
}
