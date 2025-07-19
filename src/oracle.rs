use pyo3::prelude::*;

const INVALID_FLOAT_ENTRY: f32 = f32::MIN;
const INVALID_ID_ENTRY: u32 = u32::MAX;

#[pyclass]
pub struct GtObservation {
    #[pyo3(get)]
    pub ts : f32,
    #[pyo3(get)]
    pub id : u32,
    #[pyo3(get)]
    pub xy : (f32, f32),
}

#[pymethods]
impl GtObservation {
    #[new]
    #[pyo3(signature = (time = None, id_ = None, xy_tup = None))]
    fn new(time: Option<f32>, id_: Option<u32>, xy_tup: Option<(f32, f32)>) -> Self {
        Self{ts : time.unwrap_or(INVALID_FLOAT_ENTRY),
             id : id_.unwrap_or(INVALID_ID_ENTRY),
             xy : xy_tup.unwrap_or((INVALID_FLOAT_ENTRY, INVALID_FLOAT_ENTRY))}
    }

    fn is_valid(&self) -> bool {
        INVALID_FLOAT_ENTRY != self.ts && 
        INVALID_ID_ENTRY != self.id &&
        INVALID_FLOAT_ENTRY != self.xy.0 &&
        INVALID_FLOAT_ENTRY != self.xy.1
    }

    fn __getstate__(&self) -> PyResult<(f32, u32, f32, f32)> {
        Ok((self.ts, self.id, self.xy.0, self.xy.1))
    }

    fn __setstate__(&mut self, state: (f32, u32, f32, f32)) -> PyResult<()> {
        self.ts = state.0;
        self.id = state.1;
        self.xy.0 = state.2;
        self.xy.1 = state.3;
        Ok(())
    }
}
