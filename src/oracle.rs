use pyo3::prelude::*;

#[pyclass]
pub struct GtObservation {
    pub ts : f32,
    pub id : u32,
    pub xy : (f32, f32),
}

#[pymethods]
impl GtObservation {
    #[new]
    fn new(time: f32, id_: u32, xy_tup: (f32, f32)) -> Self {
        GtObservation{ts : time,
                      id : id_,
                      xy : xy_tup,}
    }
}
