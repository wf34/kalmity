use pyo3::prelude::*;

use pyo3::types::PyList;
use pyo3::types::PyTuple;

pub mod oracle;
pub mod measurement;
pub mod front;

use crate::oracle::GtObservation;
use crate::measurement::InertialMeasurement;

#[pyclass]
struct Runtime {
    label: String,
    pose_callback: Option<PyObject>,
}

#[pymethods]
impl Runtime {
    fn set_pose_callback(&mut self, callback: PyObject) {
        self.pose_callback = Some(callback);
    }

    fn trigger_pose_callback(&self) -> PyResult<()> {
        if let Some(this_callback) = &self.pose_callback {
             Python::with_gil(|py| {
                 let rust_array = vec![0.0, 1.0, 2.5, 3.7, 4.2, 0., 0., 0.];
                 let ts_and_pose = PyList::new(py, rust_array)?;
                 let args = PyTuple::new(py, ts_and_pose)?;
                 match this_callback.call1(py, args) {
                     Ok(_) => Ok(()),
                     Err(e) => {
                         eprintln!("Pose callback error: {}", e);
                         Err(e)
                     }
                 }
             })
        } else {
            Ok(())
        }
    }

    fn add_inertial_measurement(&self, _observation: &InertialMeasurement) {
        let x = _observation.ts;
        println!("{0} got IMU {x}", self.label);
    }

    fn add_observation(&self, observation: &GtObservation) {
        let x = observation.ts;
        println!("{0} got obs {x}", self.label);
    }

    #[new]
    fn new(label: String) -> Self {
        println!("creates kalmity runtime {label}");
        Runtime{label: label, pose_callback: None}
    }
}

#[pymodule]
fn kalmity(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Runtime>()?;
    m.add_class::<GtObservation>()?;
    m.add_class::<InertialMeasurement>()?;
    Ok(())
}
