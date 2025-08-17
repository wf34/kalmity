extern crate nalgebra as na;
use na::Vector3;

use rand::prelude::*;
use pyo3::prelude::*;

use pyo3::types::PyList;
use pyo3::types::PyTuple;

pub mod oracle;
pub mod measurement;
mod ikf;
pub mod front;

use crate::oracle::GtObservation;
use crate::measurement::InertialMeasurement;
use ikf::Filter;

const SEED: u8 = 34;
const SEED_N: usize = 32;

#[pyclass]
struct Runtime {
    rng: StdRng,
    label: String,
    pose_callback: Option<PyObject>,
    ptime: Option<f64>,  // TODO remove
    pose_estimator: Filter,
}

#[pymethods]
impl Runtime {
    fn set_pose_callback(&mut self, callback: PyObject) {
        self.pose_callback = Some(callback);
    }

    fn trigger_pose_callback(&mut self) -> PyResult<()> {
        let estimate = self.pose_estimator.get_estimate();
        match (&self.pose_callback, &estimate) {
            (Some(_pose_callback), Some(_estimate)) => {
                Python::with_gil(|py| {
                    let ts_and_pose = PyList::new(py, _estimate.to_vec())?;
                    let args = PyTuple::new(py, &[ts_and_pose])?;
                    match _pose_callback.call1(py, args) {
                        Ok(_) => Ok(()),
                        Err(e) => {
                            eprintln!("Pose callback error: {}", e);
                            Err(e)
                        }
                    }
                })
            }
            _ => {
                Ok(())
            }
        }
    }

    fn add_inertial_measurement(&mut self, observation: &InertialMeasurement) {
        if !self.pose_estimator.is_inited()  {
           println!("{0} got IMU at {1}", self.label, observation.ts);
        }

        let a = Vector3::<f64>::new(observation.acceleration.0,
                                    observation.acceleration.1,
                                    observation.acceleration.2);
        let omega = Vector3::<f64>::new(observation.gyroscope.0,
                                        observation.gyroscope.1,
                                        observation.gyroscope.2);
        self.pose_estimator.propagate(observation.ts, a, omega);
        let _ = self.trigger_pose_callback();
    }

    fn add_observation(&self, _observation: &GtObservation) {
        // let timestamp = observation.ts;
        // println!("{0} got obs {x}", self.label);
    }

    #[new]
    fn new(label: String) -> Self {
        println!("creates kalmity runtime {label}");
        Runtime{rng: StdRng::from_seed([SEED; SEED_N]),
                label: label,
                pose_callback: None,
                ptime: None,
                pose_estimator: Filter::new()}
    }
}

#[pymodule]
fn kalmity(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Runtime>()?;
    m.add_class::<GtObservation>()?;
    m.add_class::<InertialMeasurement>()?;
    Ok(())
}
