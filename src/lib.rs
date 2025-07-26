use pyo3::prelude::*;

use pyo3::types::PyList;
use pyo3::types::PyTuple;

pub mod oracle;
pub mod front;

use crate::oracle::GtObservation;

#[pyclass]
struct Runtime {
    fake_state: String,
    pose_callback: Option<PyObject>,
    counter: i32,
}

#[pymethods]
impl Runtime {
    fn hello_world(&mut self) {
        println!("The measurement is: {}", self.fake_state);

        if 0 == self.counter % 10 {
            let _ = self.trigger_pose_callback();
        }

        self.counter += 1;
    }

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

    fn add_observation(&self, observation: &GtObservation) {
        let x = observation.ts;
        println!("got obs {x}");
    }

    #[new]
    fn new(a: String) -> Self {
        Runtime{fake_state: a, pose_callback: None, counter: 0}
    }
}

/// A Python module implemented in Rust.
#[pymodule]
fn kalmity(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Runtime>()?;
    m.add_class::<GtObservation>()?;
    Ok(())
}
