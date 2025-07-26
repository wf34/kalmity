use pyo3::prelude::*;

const INVALID_FLOAT_ENTRY: f32 = f32::MIN;

#[pyclass]
pub struct InertialMeasurement {
    #[pyo3(get)]
    pub ts : f32,

    #[pyo3(get)]
    pub gyroscope : (f32, f32, f32),

    #[pyo3(get)]
    pub acceleration : (f32, f32, f32),
}

#[pymethods]
impl InertialMeasurement {
    #[new]
    #[pyo3(signature = (time = None, gyro = None, accel = None))]
    fn new(time: Option<f32>,
           gyro: Option<(f32, f32, f32)>,
           accel: Option<(f32, f32, f32)>) -> Self {
        Self {ts : time.unwrap_or(INVALID_FLOAT_ENTRY),
              gyroscope : gyro.unwrap_or((INVALID_FLOAT_ENTRY, INVALID_FLOAT_ENTRY, INVALID_FLOAT_ENTRY)),
              acceleration : accel.unwrap_or((INVALID_FLOAT_ENTRY, INVALID_FLOAT_ENTRY, INVALID_FLOAT_ENTRY))}
    }

    fn is_valid(&self) -> bool {
        INVALID_FLOAT_ENTRY != self.ts && 
        INVALID_FLOAT_ENTRY != self.gyroscope.0 &&
        INVALID_FLOAT_ENTRY != self.gyroscope.1 &&
        INVALID_FLOAT_ENTRY != self.gyroscope.2 &&
        INVALID_FLOAT_ENTRY != self.acceleration.0 &&
        INVALID_FLOAT_ENTRY != self.acceleration.1 &&
        INVALID_FLOAT_ENTRY != self.acceleration.2
    }

    fn __getstate__(&self) -> PyResult<(f32, f32, f32, f32, f32, f32, f32)> {
        Ok((self.ts,
            self.gyroscope.0, self.gyroscope.1, self.gyroscope.2,
            self.acceleration.0, self.acceleration.1, self.acceleration.2))
    }

    fn __setstate__(&mut self, state: (f32, f32, f32, f32, f32, f32, f32)) -> PyResult<()> {
        self.ts = state.0;
        self.gyroscope.0 = state.1;
        self.gyroscope.1 = state.2;
        self.gyroscope.2 = state.3;
        self.acceleration.0 = state.4;
        self.acceleration.1 = state.5;
        self.acceleration.2 = state.6;
        Ok(())
    }
}
