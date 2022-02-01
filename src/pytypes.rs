use glam::Vec3A;
use pyo3::{pyclass, pymethods};
use rl_ball_sym::simulation::ball::Ball;

use crate::utils::get_tuple_from_vec3;

#[pyclass]
#[allow(dead_code)]
pub struct BasicShotInfo {
    #[pyo3(get)]
    found: bool,
    #[pyo3(get)]
    time: Option<f32>,
}

impl BasicShotInfo {
    pub const fn not_found() -> Self {
        BasicShotInfo { found: false, time: None }
    }

    pub const fn found(time: f32) -> Self {
        BasicShotInfo { found: true, time: Some(time) }
    }
}

#[pymethods]
impl BasicShotInfo {
    fn __str__(&self) -> String {
        match self.time {
            Some(time) => format!("Found at time: {:.2}", time),
            None => String::from("Not found"),
        }
    }

    fn __repr__(&self) -> String {
        match self.time {
            Some(time) => format!("BasicShotInfo(found=True, time={})", time),
            None => String::from("BasicShotInfo(found=False)"),
        }
    }
}

#[pyclass]
#[allow(dead_code)]
pub struct BallSlice {
    #[pyo3(get)]
    time: f32,
    #[pyo3(get)]
    location: (f32, f32, f32),
    #[pyo3(get)]
    velocity: (f32, f32, f32),
    #[pyo3(get)]
    angular_velocity: (f32, f32, f32),
}

#[pymethods]
impl BallSlice {
    fn __str__(&self) -> String {
        format!(
            "Ball @{:.2}s - location: {:?}, velocity: {:?}, angular velocity: {:?}",
            self.time, self.location, self.velocity, self.angular_velocity
        )
    }

    fn __repr__(&self) -> String {
        format!(
            "BallSlice(time={}, location={:?}, velocity={:?}, angular_velocity={:?})",
            self.time, self.location, self.velocity, self.angular_velocity
        )
    }
}

impl BallSlice {
    pub fn from(ball: &Ball) -> Self {
        let time = ball.time;
        let location = get_tuple_from_vec3(ball.location);
        let velocity = get_tuple_from_vec3(ball.velocity);
        let angular_velocity = get_tuple_from_vec3(ball.angular_velocity);

        BallSlice {
            time,
            location,
            velocity,
            angular_velocity,
        }
    }
}

#[pyclass]
#[allow(dead_code)]
pub struct AdvancedShotInfo {
    #[pyo3(get)]
    final_target: (f32, f32, f32),
    #[pyo3(get)]
    distance_remaining: f32,
    #[pyo3(get)]
    path_samples: Vec<(f32, f32)>,
}

#[pymethods]
impl AdvancedShotInfo {
    fn __str__(&self) -> String {
        format!("Final target: {:?}, distance remaining: {:.2}", self.final_target, self.distance_remaining)
    }

    fn __repr__(&self) -> String {
        format!(
            "AdvancedShotInfo(final_target={:?}, distance_remaining={}, path_samples=[{} items])",
            self.final_target,
            self.distance_remaining,
            self.path_samples.len()
        )
    }
}

impl AdvancedShotInfo {
    pub fn from(target: Vec3A, distance_remaining: f32, path_samples: Vec<(f32, f32)>) -> Self {
        AdvancedShotInfo {
            final_target: (target.x, target.y, 0.),
            distance_remaining,
            path_samples,
        }
    }
}
