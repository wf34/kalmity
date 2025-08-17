
extern crate nalgebra as na;
use na::{Vector3, Quaternion};

struct NomState {
    p: Vector3<f32>,
    v: Vector3<f32>,
    q: Quaternion<f32>,
    a_bias: Vector3<f32>,
    omega_bias: Vector3<f32>,
    gravity: Vector3<f32>,
}

struct ErrState {
    dp: Vector3<f32>,
    dv: Vector3<f32>,
    dtheta: Vector3<f32>,
    da_bias: Vector3<f32>,
    dgravity: Vector3<f32>,
}

struct PoseEstimate {
    timestamp: f32,
    q: Quaternion<f32>,
    t: Vector3<f32>,
}

pub(crate) struct Filter {
    prop_time: Option<f32>,
    upd_time: Option<f32>,
    nom: Option<NomState>,
    err: Option<ErrState>,
}

impl NomState {
    fn new() -> Self {
        NomState{p: Vector3::<f32>::zeros(),
                 v: Vector3::<f32>::zeros(),
                 q: Quaternion::<f32>::identity(),
                 a_bias: Vector3::<f32>::zeros(),
                 omega_bias: Vector3::<f32>::zeros(),
                 gravity: Vector3::<f32>::zeros(),
        }
    }
}

impl ErrState {
    fn new() -> Self {
        ErrState{dp: Vector3::<f32>::zeros(),
                 dv: Vector3::<f32>::zeros(),
                 dtheta: Vector3::<f32>::zeros(),
                 da_bias: Vector3::<f32>::zeros(),
                 dgravity: Vector3::<f32>::zeros(),
        }
    }
}

impl Filter {
    pub fn new() -> Self {
        Filter{prop_time: None,
               upd_time: None,
               nom: Some(NomState::new()),
               err: Some(ErrState::new()),
        }
    }

    fn init(&mut self) {}
    pub fn propagate(&mut self, timestamp: f32, a_m: Vector3<f32>, omega_m: Vector3<f32>) {
    }

    pub fn get_estimate() -> Option<PoseEstimate> {
        None
    }
}
