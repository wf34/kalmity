use pyo3::prelude::*;

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
                 // let args = PyTuple::new(py, &[value]);
                 // call1(py, args)
                 match this_callback.call0(py) {
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
