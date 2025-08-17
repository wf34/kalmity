use rand::prelude::*;
use pyo3::prelude::*;

use rand_distr::Normal;

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
        match(&self.pose_callback, &self.ptime) {
            (Some(this_callback), Some(propagate_time)) => {
                Python::with_gil(|py| {
                    let n_a = Normal::new(0.0, 0.01).unwrap();
                    let n_t = Normal::new(0.0, 0.1).unwrap();

                    let mut arr = vec![
                        *propagate_time
                      , 1.0
                      , n_a.sample(&mut self.rng)
                      , n_a.sample(&mut self.rng)
                      , n_a.sample(&mut self.rng)
                      , n_t.sample(&mut self.rng)
                      , n_t.sample(&mut self.rng)
                      , n_t.sample(&mut self.rng)];

                    let norm: f32 = (&arr[1].powf(2.) +
                                     &arr[2].powf(2.) +
                                     &arr[3].powf(2.) +
                                     &arr[4].powf(2.)).sqrt();
                    arr[1] /= norm;
                    arr[2] /= norm;
                    arr[3] /= norm;
                    arr[4] /= norm;

                    let ts_and_pose = PyList::new(py, arr)?;
                    let args = PyTuple::new(py, &[ts_and_pose])?;
                    match this_callback.call1(py, args) {
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

    fn add_inertial_measurement(&mut self, _observation: &InertialMeasurement) {
        let curr_ts = _observation.ts;
        // println!("{0} got IMU {curr_ts}", self.label);

        assert!(self.ptime.is_none() || self.ptime.unwrap() < curr_ts);
        self.ptime = Some(curr_ts);

        let _ = self.trigger_pose_callback();
    }

    fn add_observation(&self, observation: &GtObservation) {
        let x = observation.ts;
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
