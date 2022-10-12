use crate::{
    car::Car,
    constants::*,
    shot::{AirBasedShot, GroundBasedShot},
    utils::{flatten, get_tuple_from_vec3},
};
use glam::Vec3A;
use pyo3::{pyclass, pymethods};
use rl_ball_sym::simulation::ball::Ball;

#[pyclass(frozen)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShotType {
    Ground = 0,
    Jump,
    DoubleJump,
    Aerial,
}

impl ShotType {
    #[inline]
    pub const fn to_str(self) -> &'static str {
        match self {
            ShotType::Ground => "Ground",
            ShotType::Jump => "Jump",
            ShotType::DoubleJump => "DoubleJump",
            ShotType::Aerial => "Aerial",
        }
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
    #[inline]
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
    shot_type: Option<ShotType>,
    #[pyo3(get)]
    shot_vector: (f32, f32, f32),
    #[pyo3(get)]
    is_forwards: bool,
}

impl BasicShotInfo {
    #[inline]
    pub const fn not_found() -> Self {
        BasicShotInfo {
            found: false,
            time: -1.,
            shot_type: None,
            shot_vector: (0., 0., 0.),
            is_forwards: true,
        }
    }

    #[inline]
    pub const fn found(time: f32, shot_type: ShotType, shot_vector: Vec3A, is_forwards: bool) -> Self {
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
            Some(shot_type) => format!("{} shot found at time: {:.2}", shot_type.to_str(), self.time),
            None => String::from("Not found"),
        }
    }

    fn __repr__(&self) -> String {
        match self.shot_type {
            Some(shot_type) => format!(
                "BasicShotInfo(found=True, time={}, type={}, shot_vector={:?})",
                self.time,
                shot_type.to_str(),
                self.shot_vector
            ),
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
    #[inline]
    pub const fn from(ball: Ball) -> Self {
        BallSlice {
            time: ball.time,
            location: get_tuple_from_vec3(ball.location),
            velocity: get_tuple_from_vec3(ball.velocity),
            angular_velocity: get_tuple_from_vec3(ball.angular_velocity),
        }
    }
}

type PyVec3A = (f32, f32, f32);

#[pyclass(frozen)]
#[allow(dead_code)]
pub struct AdvancedShotInfo {
    #[pyo3(get)]
    final_target: PyVec3A,
    #[pyo3(get)]
    distance_remaining: f32,
    #[pyo3(get)]
    required_jump_time: Option<f32>,
    #[pyo3(get)]
    path_samples: Vec<(f32, f32)>,
    #[pyo3(get)]
    current_path_point: PyVec3A,
    #[pyo3(get)]
    turn_targets: Option<(PyVec3A, PyVec3A)>,
    #[pyo3(get)]
    num_jumps: Option<u8>,
}

impl AdvancedShotInfo {
    #[inline]
    pub const fn get_distance_remaining(&self) -> f32 {
        self.distance_remaining
    }
}

#[pymethods]
impl AdvancedShotInfo {
    fn __str__(&self) -> String {
        if let Some(required_jump_time) = self.required_jump_time {
            format!(
                "Final target: {:?}, distance remaining: {:.0}, required jump time: {:.1}, num_jumps: {:?}",
                self.final_target, self.distance_remaining, required_jump_time, self.num_jumps
            )
        } else {
            format!("Final target: {:?}, distance remaining: {:.0}", self.final_target, self.distance_remaining)
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "AdvancedShotInfo(final_target={:?}, distance_remaining={}, required_jump_time: {:?}, num_jumps: {:?}, path_samples=[{} items], current_path_point:{:?})",
            self.final_target,
            self.distance_remaining,
            self.required_jump_time,
            self.num_jumps,
            self.path_samples.len(),
            self.current_path_point,
        )
    }
}

impl AdvancedShotInfo {
    pub fn get_from_ground(car: &Car, shot: &GroundBasedShot) -> Option<Self> {
        let (segment, pre_index) = shot.find_min_distance_index(car.location);
        let (distance_along, index) = shot.get_distance_along_shot_and_index(segment, pre_index);
        let current_path_point = shot.samples[segment][pre_index];

        if current_path_point.distance(flatten(car.location)) > car.hitbox.length / 2. {
            return None;
        }

        let distance = car.local_velocity.x.max(500.) * STEER_REACTION_TIME;

        let path_length = shot.distances[0] + shot.distances[1] + shot.distances[2];
        let max_path_distance = path_length - distance_along;
        let distance_to_ball = path_length + shot.distances[3] - distance_along;

        let target = if distance > max_path_distance {
            let distance_remaining = distance - max_path_distance;
            let additional_space = shot.direction * distance_remaining;

            shot.path_endpoint.pos + additional_space
        } else {
            shot.path.sample(distance_along + distance).pos
        };

        // get all the samples from the vec after index
        let samples = shot.all_samples.iter().skip(index / GroundBasedShot::ALL_STEP).copied().collect();

        Some(Self {
            final_target: get_tuple_from_vec3(flatten(target)),
            distance_remaining: distance_to_ball,
            path_samples: samples,
            required_jump_time: shot.jump_time,
            current_path_point: get_tuple_from_vec3(flatten(current_path_point)),
            turn_targets: if let Some((a, b)) = shot.turn_targets {
                Some((get_tuple_from_vec3(flatten(a)), get_tuple_from_vec3(flatten(b))))
            } else {
                None
            },
            num_jumps: None,
        })
    }

    #[inline]
    pub fn get_from_air(car: &Car, shot: &AirBasedShot) -> Option<Self> {
        Some(Self {
            final_target: get_tuple_from_vec3(shot.final_target),
            distance_remaining: car.location.distance(shot.final_target),
            path_samples: Vec::new(),
            required_jump_time: None,
            current_path_point: get_tuple_from_vec3(car.location),
            turn_targets: None,
            num_jumps: Some(shot.jump_type as u8),
        })
    }
}
