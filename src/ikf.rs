
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

    fn propagate()
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

    pub fn is_inited(& self) -> bool {
        assert!(self.prop_time.is_none() || (self.nom.is_some() && self.err.is_some()));
        self.prop_time.is_some()
    }

    pub fn propagate(&mut self, timestamp: f64, a_m: Vector3<f64>, omega_m: Vector3<f64>) {
        assert!(self.prop_time.is_none() || self.prop_time.unwrap() < timestamp);

        let (Some(_nom), Some(_err)) = (&mut self.nom, &mut self.err) else {
            panic!("`propagate` was called on the uninited `Filter`");
        };

        let Some(prev_time) = self.prop_time {
            let delta_t: f64 = timestamp - prev_time;

            _nom.propagate(delta_t, a_m, omega_m);
            _err.propagate(delta_t, a_m, omega_m, _nom.a_bias, _nom.omaga_bias);
        };

        self.prop_time = Some(timestamp);
        assert_eq!(_nom.q.norm(), 1.0);
    }

    pub fn get_estimate(& self) -> Option<PoseEstimate> {
        let (Some(_prop_time), Some(_nom), Some(_err)) = (self.prop_time, &self.nom, &self.err) else {
            return None
        };

        Some(PoseEstimate{timestamp: _prop_time,
                          q: _nom.q,
                          p: _nom.p})
    }
}
