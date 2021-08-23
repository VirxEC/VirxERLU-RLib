use std::f32::consts::PI;

use cpython::{exc, ObjectProtocol, PyErr, PyObject, PyResult, Python};
use rl_ball_sym::{linear_algebra::vector::Vec3, simulation::ball::Ball};

static MAX_TURN_RADIUS: f32 = 1. / 0.0017996;

#[derive(Clone)]
pub struct Slice {
    pub ball: Box<Ball>,
    pub distance_remaining: f32,
    pub speed: f32,
    pub shot_vector: Vec3,
}

pub struct Hitbox {
    pub length: f32,
    pub width: f32,
    pub height: f32,
}

impl Default for Hitbox {
    fn default() -> Self {
        Self {
            length: 0.,
            width: 0.,
            height: 0.,
        }
    }
}

impl Hitbox {
    pub fn from_vec3(vec: Vec3) -> Self {
        Self {
            length: vec.x,
            width: vec.y,
            height: vec.z,
        }
    }
}

pub struct Car {
    pub location: Vec3,
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    pub hitbox: Hitbox,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub forward: Vec3,
    pub right: Vec3,
    pub up: Vec3,
}

impl Default for Car {
    fn default() -> Self {
        Self {
            location: Vec3::default(),
            velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            hitbox: Hitbox::default(),
            pitch: 0.,
            yaw: 0.,
            roll: 0.,
            forward: Vec3::default(),
            right: Vec3::default(),
            up: Vec3::default(),
        }
    }
}

impl Car {
    pub fn calculate_orientation_matrix(&mut self) {
        let c_p = self.pitch.cos();
        let s_p = self.pitch.sin();
        let c_y = self.yaw.cos();
        let s_y = self.yaw.sin();
        let c_r = self.roll.cos();
        let s_r = self.roll.sin();

        self.forward.x = c_p * c_y;
        self.forward.y = c_p * s_y;
        self.forward.z = s_p;

        self.right.x = c_y * s_p * s_r - c_r * s_y;
        self.right.y = s_y * s_p * s_r + c_r * c_y;
        self.right.z = -c_p * c_r;

        self.up.x = -c_r * c_y * s_p - s_r * s_y;
        self.up.y = -c_r * s_y * s_p + s_r * c_y;
        self.up.z = c_p * c_r;
    }
}

pub fn get_vec3(py: Python, py_vec: &PyObject) -> PyResult<Vec3> {
    let mut vec = Vec3::default();

    if let Ok(x) = py_vec.get_item(py, 0) {
        vec.x = x.extract(py)?;
    } else {
        return Err(PyErr::new::<exc::IndexError, _>(py, "Key 'location' in 'ball' needs to be a list of at least 3 numbers"));
    }

    if let Ok(y) = py_vec.get_item(py, 1) {
        vec.y = y.extract(py)?;
    } else {
        return Err(PyErr::new::<exc::IndexError, _>(py, "Key 'location' in 'ball' needs to be a list of at least 3 numbers"));
    }

    if let Ok(z) = py_vec.get_item(py, 2) {
        vec.z = z.extract(py)?;
    } else {
        return Err(PyErr::new::<exc::IndexError, _>(py, "Key 'location' in 'ball' needs to be a list of at least 3 numbers"));
    }

    Ok(vec)
}

pub fn get_vec_from_vec3(vec: Vec3) -> Vec<f32> {
    vec![vec.x, vec.y, vec.z]
}

pub fn flattened(vec: &Vec3) -> Vec3 {
    Vec3 {
        x: vec.x,
        y: vec.y,
        z: 0.,
    }
}

// pub fn angle(vec1: &Vec3, vec2: &Vec3) -> f32 {
//     vec1.dot(vec2).clamp(-1., 1.).acos()
// }

pub fn angle_tau_2d(vec1: &Vec3, vec2: &Vec3) -> f32 {
    let angle = vec2.y.atan2(vec2.x) - vec1.y.atan2(vec1.x);

    if angle < 0. {
        return angle + 2. * PI;
    }

    angle
}

// pub fn dist(vec1: &Vec3, vec2: &Vec3) -> f32 {
//     (*vec2 - *vec1).magnitude()
// }

pub fn dist_2d(vec1: &Vec3, vec2: &Vec3) -> f32 {
    incomplete_dist_2d(vec1, vec2).sqrt()
}

pub fn incomplete_dist_2d(vec1: &Vec3, vec2: &Vec3) -> f32 {
    let vec = *vec2 - *vec1;
    vec.x * vec.x + vec.y * vec.y
}

pub fn rotate_2d(vec: &Vec3, angle: &f32) -> Vec3 {
    Vec3 {
        x: angle.cos() * vec.x - angle.sin() * vec.y,
        y: angle.sin() * vec.x + angle.cos() * vec.y,
        z: vec.z,
    }
}

pub fn get_distance_remaining(ball: Box<Ball>, car: &Car, shot_vector: &Vec3) -> f32 {
    let offset_target = ball.location - (*shot_vector * ball.collision_radius);
    let mut distance_remaining = -car.hitbox.length / 2.;

    let exit_turn_point = offset_target - (flattened(&shot_vector) * 320.);

    if car.forward.dot(&exit_turn_point) <= 0. && car.forward.dot(&offset_target) > 0. && car.right.dot(&offset_target).abs() < ball.collision_radius * 1.2 {
        distance_remaining += dist_2d(&car.location, &offset_target);
    } else {
        let inv_shot_vector = -*shot_vector;
        let half_pi = PI / 2.;

        let mut inv_shot_vector_perp = rotate_2d(&inv_shot_vector, &half_pi);
        let mut side = false;

        let inv_shot_vector_perp_2 = rotate_2d(&inv_shot_vector, &-half_pi);

        if incomplete_dist_2d(&(inv_shot_vector_perp_2 + exit_turn_point), &car.location) < incomplete_dist_2d(&(inv_shot_vector_perp + exit_turn_point), &car.location) {
            inv_shot_vector_perp = inv_shot_vector_perp_2;
            side = true;
        }

        let circle_center = exit_turn_point + inv_shot_vector_perp * MAX_TURN_RADIUS;
        let cc_to_cl = car.location - circle_center;

        let tangent_angle = (MAX_TURN_RADIUS / cc_to_cl.magnitude()).clamp(-1., 1.).acos();
        let cc_to_cl = cc_to_cl.normalize();

        let tangent_lines = [rotate_2d(&cc_to_cl, &-tangent_angle), rotate_2d(&cc_to_cl, &tangent_angle)];

        let cc_to_etp = (exit_turn_point - circle_center).normalize();
        let enter_turn_line;

        if side {
            enter_turn_line = tangent_lines.iter().max_by(|a, b| angle_tau_2d(&cc_to_etp, a).partial_cmp(&angle_tau_2d(&cc_to_etp, b)).unwrap_or(std::cmp::Ordering::Equal)).unwrap();
        } else {
            enter_turn_line = tangent_lines.iter().min_by(|a, b| angle_tau_2d(&cc_to_etp, a).partial_cmp(&angle_tau_2d(&cc_to_etp, b)).unwrap_or(std::cmp::Ordering::Equal)).unwrap();
        }

        let enter_turn_point = *enter_turn_line * MAX_TURN_RADIUS + circle_center;
        let turn_angle = angle_tau_2d(&enter_turn_line, &cc_to_etp);
        let turn_distance_remaining = turn_angle * MAX_TURN_RADIUS;

        distance_remaining += turn_distance_remaining + dist_2d(&exit_turn_point, &offset_target);

        if dist_2d(&car.location, &enter_turn_point) > car.hitbox.width / 2. {
            distance_remaining += dist_2d(&car.location, &enter_turn_point);
        }
    }

    distance_remaining
}

pub fn is_slice_viable(_target: Vec3, _gravity: &Vec3, _ball: Box<Ball>, _car: &Car, _distance_remaining: f32, minimum_speed_required: f32, _shot_vector: &Vec3) -> bool {
    minimum_speed_required < 1600.
}
