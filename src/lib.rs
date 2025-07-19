use pyo3::prelude::*;

pub mod oracle;
pub mod front;

use crate::oracle::GtObservation;

#[pyclass]
struct Runtime {
    fake_state: String,
}

#[pymethods]
impl Runtime {
    fn hello_world(&self) {
        println!("The measurement is: {}", self.fake_state);
    }

    fn add_observation(&self, observation: &GtObservation) {
        let x = observation.ts;
        println!("got obs {x}");
    }

    #[new]
    fn new(a: String) -> Self {
        Runtime{fake_state: a}
    }
}

/// A Python module implemented in Rust.
#[pymodule]
fn kalmity(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Runtime>()?;
    m.add_class::<GtObservation>()?;
    Ok(())
}
