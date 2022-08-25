use crate::utils::{flatten, get_tuple_from_vec3};
use glam::Vec3A;
use pyo3::{pyclass, pymethods};
use rl_ball_sym::simulation::ball::Ball;

#[pyclass(frozen)]
#[derive(Clone, Copy)]
pub struct ShotType;

#[pymethods]
impl ShotType {
    #[classattr]
    pub const GROUND: usize = 0;
    #[classattr]
    pub const JUMP: usize = 1;
    #[classattr]
    pub const DOUBLE_JUMP: usize = 2;
}

fn get_str_from_shot_type(type_: usize) -> &'static str {
    match type_ {
        ShotType::GROUND => "GROUND",
        ShotType::JUMP => "JUMP",
        ShotType::DOUBLE_JUMP => "DOUBLE_JUMP",
        _ => unreachable!(),
    }
}

#[pyclass(frozen)]
#[derive(Clone, Copy, Debug, Default)]
pub struct TargetOptions {
    pub min_slice: Option<usize>,
    pub max_slice: Option<usize>,
    pub use_absolute_max_values: Option<bool>,
    pub all: Option<bool>,
}

#[pymethods]
impl TargetOptions {
    #[new]
    const fn __new__(min_slice: Option<usize>, max_slice: Option<usize>, use_absolute_max_values: Option<bool>, all: Option<bool>) -> Self {
        Self {
            min_slice,
            max_slice,
            use_absolute_max_values,
            all,
        }
    }

    fn __str__(&self) -> String {
        let mut s = Vec::with_capacity(4);

        if let Some(min_slice) = self.min_slice {
            s.push(format!("min_slice=={}", min_slice));
        }

        if let Some(max_slice) = self.max_slice {
            s.push(format!("max_slice=={}", max_slice));
        }

        if let Some(use_absolute_max_values) = self.use_absolute_max_values {
            s.push(format!("use_absolute_max_values=={}", use_absolute_max_values));
        }

        if let Some(all) = self.all {
            s.push(format!("all=={}", all));
        }

        s.join(", ")
    }

    fn __repr__(&self) -> String {
        format!(
            "TargetOptions(min_slice={:?}, max_slice={:?}, use_absolute_max_values={:?}, all={:?})",
            self.min_slice, self.max_slice, self.use_absolute_max_values, self.all
        )
    }
}

#[pyclass(frozen)]
#[allow(dead_code)]
pub struct BasicShotInfo {
    #[pyo3(get)]
    found: bool,
    #[pyo3(get)]
    time: f32,
    #[pyo3(get)]
    shot_type: Option<usize>,
    #[pyo3(get)]
    shot_vector: (f32, f32, f32),
    #[pyo3(get)]
    is_forwards: bool,
}

impl BasicShotInfo {
    pub const fn not_found() -> Self {
        BasicShotInfo {
            found: false,
            time: -1.,
            shot_type: None,
            shot_vector: (0., 0., 0.),
            is_forwards: true,
        }
    }

    pub const fn found(time: f32, shot_type: usize, shot_vector: Vec3A, is_forwards: bool) -> Self {
        BasicShotInfo {
            found: true,
            time,
            shot_type: Some(shot_type),
            shot_vector: get_tuple_from_vec3(shot_vector),
            is_forwards,
        }
    }
}

#[pymethods]
impl BasicShotInfo {
    fn __str__(&self) -> String {
        match self.shot_type {
            Some(shot_type) => format!("{} found at time: {:.2}", get_str_from_shot_type(shot_type), self.time),
            None => String::from("Not found"),
        }
    }

    fn __repr__(&self) -> String {
        match self.shot_type {
            Some(shot_type) => format!("BasicShotInfo(found=True, time={}, type={})", self.time, get_str_from_shot_type(shot_type)),
            None => String::from("BasicShotInfo(found=False)"),
        }
    }
}

#[pyclass(frozen)]
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
    pub const fn from(ball: &Ball) -> Self {
        BallSlice {
            time: ball.time,
            location: get_tuple_from_vec3(ball.location),
            velocity: get_tuple_from_vec3(ball.velocity),
            angular_velocity: get_tuple_from_vec3(ball.angular_velocity),
        }
    }
}

#[pyclass(frozen)]
#[allow(dead_code)]
pub struct AdvancedShotInfo {
    #[pyo3(get)]
    shot_vector: (f32, f32, f32),
    #[pyo3(get)]
    final_target: (f32, f32, f32),
    #[pyo3(get)]
    distance_remaining: f32,
    #[pyo3(get)]
    required_jump_time: Option<f32>,
    #[pyo3(get)]
    path_samples: Vec<(f32, f32)>,
    #[pyo3(get)]
    current_path_point: (f32, f32, f32),
}

impl AdvancedShotInfo {
    pub const fn get_distance_remaining(&self) -> f32 {
        self.distance_remaining
    }
}

#[pymethods]
impl AdvancedShotInfo {
    fn __str__(&self) -> String {
        match self.required_jump_time {
            Some(required_jump_time) => format!(
                "Shot vector: {:?}, Final target: {:?}, distance remaining: {:.0}, required jump time: {:.1}",
                self.shot_vector, self.final_target, self.distance_remaining, required_jump_time
            ),
            None => format!(
                "Shot vector: {:?}, Final target: {:?}, distance remaining: {:.0}",
                self.shot_vector, self.final_target, self.distance_remaining
            ),
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "AdvancedShotInfo(shot_vector={:?}, final_target={:?}, distance_remaining={}, required_jump_time: {:?}, path_samples=[{} items], current_path_point:{:?})",
            self.shot_vector,
            self.final_target,
            self.distance_remaining,
            self.required_jump_time,
            self.path_samples.len(),
            self.current_path_point,
        )
    }
}

impl AdvancedShotInfo {
    pub const fn from(
        shot_vector: Vec3A,
        target: Vec3A,
        distance_remaining: f32,
        path_samples: Vec<(f32, f32)>,
        required_jump_time: Option<f32>,
        current_path_point: Vec3A,
    ) -> Self {
        AdvancedShotInfo {
            shot_vector: get_tuple_from_vec3(shot_vector),
            final_target: get_tuple_from_vec3(flatten(target)),
            distance_remaining,
            path_samples,
            required_jump_time,
            current_path_point: get_tuple_from_vec3(flatten(current_path_point)),
        }
    }
}
