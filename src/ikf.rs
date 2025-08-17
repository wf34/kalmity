
extern crate nalgebra as na;
use na::{Vector3, Quaternion};

struct NomState {
    p: Vector3<f64>,
    v: Vector3<f64>,
    q: Quaternion<f64>,
    a_bias: Vector3<f64>,
    omega_bias: Vector3<f64>,
    gravity: Vector3<f64>,
}

struct ErrState {
    delta_p: Vector3<f64>,
    delta_v: Vector3<f64>,
    delta_theta: Vector3<f64>,
    delta_a_bias: Vector3<f64>,
    delta_gravity: Vector3<f64>,
}

pub(crate) struct PoseEstimate {
    timestamp: f64,
    q: Quaternion<f64>,
    p: Vector3<f64>,
}

pub(crate) struct Filter {
    prop_time: Option<f64>,
    upd_time: Option<f64>,
    nom: Option<NomState>,
    err: Option<ErrState>,
}

impl NomState {
    fn new() -> Self {
        NomState{p: Vector3::<f64>::zeros(),
                 v: Vector3::<f64>::zeros(),
                 q: Quaternion::<f64>::identity(),
                 a_bias: Vector3::<f64>::zeros(),
                 omega_bias: Vector3::<f64>::zeros(),
                 gravity: Vector3::<f64>::zeros(),
        }
    }
}

impl ErrState {
    fn new() -> Self {
        ErrState{delta_p: Vector3::<f64>::zeros(),
                 delta_v: Vector3::<f64>::zeros(),
                 delta_theta: Vector3::<f64>::zeros(),
                 delta_a_bias: Vector3::<f64>::zeros(),
                 delta_gravity: Vector3::<f64>::zeros(),
        }
    }
}

impl PoseEstimate {
    pub fn to_vec(& self) -> Vec<f64> {
        let mut stack: Vec<f64> = Vec::new();
        stack.push(self.timestamp);
        stack.extend(self.q.coords.iter().cloned());
        stack.extend(self.p.iter().cloned());
        assert_eq!(8, stack.len());
        stack
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
