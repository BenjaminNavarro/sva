use sva::*;

#[derive(Clone, Copy, Debug)]
#[allow(non_snake_case)]
pub struct ABInertia {
    pub M: Mat3,
    pub H: Mat3,
    pub I: Mat3,
}
