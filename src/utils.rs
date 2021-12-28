use std::f32::{consts::FRAC_PI_2, INFINITY};

use cpython::{exc, ObjectProtocol, PyDict, PyErr, PyObject, PyResult, Python};
use dubins_paths::{intermediate_results, mod2pi, path_length, path_sample, segment_length, word, DubinsError, DubinsPath, DubinsPathType, SegmentType, DIRDATA};
use rl_ball_sym::simulation::ball::Ball;

use glam::Vec3A;

pub const MAX_SPEED: f32 = 2300.;
pub const MAX_SPEED_NO_BOOST: f32 = 1410.;
pub const MIN_SPEED: f32 = -MAX_SPEED_NO_BOOST;
pub const TPS: f32 = 120.;
pub const SIMULATION_DT: f32 = 1. / TPS;
pub const BOOST_CONSUMPTION: f32 = 33.3 + 1. / 33.;
pub const BRAKE_ACC: f32 = 3500.;
pub const COAST_ACC: f32 = 525.;
pub const MIN_BOOST_TIME: f32 = 3. / 120.;
pub const REACTION_TIME: f32 = 0.04;

pub const MIN_BOOST_CONSUMPTION: f32 = BOOST_CONSUMPTION * MIN_BOOST_TIME;
pub const BOOST_CONSUMPTION_DT: f32 = BOOST_CONSUMPTION * SIMULATION_DT;
pub const BRAKE_ACC_DT: f32 = BRAKE_ACC * SIMULATION_DT;

pub const BRAKE_COAST_TRANSITION: f32 = -(0.45 * BRAKE_ACC + 0.55 * COAST_ACC);
pub const COASTING_THROTTLE_TRANSITION: f32 = -0.5 * COAST_ACC;

pub const THROTTLE_ACCEL_DIVISION: f32 = 1400.;
pub const START_THROTTLE_ACCEL_M: f32 = -36. / 35.;
pub const START_THROTTLE_ACCEL_B: f32 = 1600.;
pub const END_THROTTLE_ACCEL_M: f32 = -16.;
pub const END_THROTTLE_ACCEL_B: f32 = 160.;

const BOOST_ACCEL: f32 = 991. + 2. / 3.;
const BOOST_ACCEL_DT: f32 = BOOST_ACCEL * SIMULATION_DT;

const STEER_REACTION_TIME: f32 = 0.25;

#[derive(Clone, Copy, Debug, Default)]
pub struct Hitbox {
    pub length: f32,
    pub width: f32,
    pub height: f32,
}

impl Hitbox {
    pub fn from_vec3(vec: Vec3A) -> Self {
        Self {
            length: vec.x,
            width: vec.y,
            height: vec.z,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Car {
    pub location: Vec3A,
    pub velocity: Vec3A,
    pub local_velocity: Vec3A,
    pub angular_velocity: Vec3A,
    pub forward: Vec3A,
    pub right: Vec3A,
    pub up: Vec3A,
    pub hitbox: Hitbox,
    pub hitbox_offset: Vec3A,
    pub field: CarFieldRect,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub boost: u8,
    pub demolished: bool,
    pub airborne: bool,
    pub jumped: bool,
    pub doublejumped: bool,
    pub max_speed: f32,
    /// turn radius at calculated max speed
    pub ctrms: f32,
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
        self.right.z = -c_p * s_r;

        self.up.x = -c_r * c_y * s_p - s_r * s_y;
        self.up.y = -c_r * s_y * s_p + s_r * c_y;
        self.up.z = c_p * c_r;
    }

    pub fn calculate_max_values(&mut self) {
        self.max_speed = self.get_max_speed();
        self.ctrms = turn_radius(self.max_speed);
    }

    pub fn calculate_local_values(&mut self) {
        self.local_velocity = localize(self, self.velocity);
    }

    pub fn calculate_field(&mut self) {
        self.field = CarFieldRect::from(&self.hitbox);
    }

    fn get_max_speed(&mut self) -> f32 {
        let mut b = self.boost as f32;
        let mut v = self.velocity.dot(self.forward);

        loop {
            if v >= MAX_SPEED {
                return MAX_SPEED;
            }

            if b < BOOST_CONSUMPTION_DT {
                if v < MAX_SPEED_NO_BOOST {
                    return MAX_SPEED_NO_BOOST;
                } else {
                    return v;
                }
            }

            if v.signum() == -1. {
                v += BRAKE_ACC_DT;
            } else {
                v += throttle_acceleration(v) * SIMULATION_DT;
                v += BOOST_ACCEL_DT;
                b -= BOOST_CONSUMPTION_DT;
            }
        }
    }
}

pub fn get_vec3(py: Python, py_vec: &PyObject, too_few_vals_err_msg: &str) -> PyResult<Vec3A> {
    let mut vec = Vec3A::ZERO;

    if let Ok(x) = py_vec.get_item(py, 0) {
        vec.x = x.extract(py)?;
    } else {
        return Err(PyErr::new::<exc::IndexError, _>(py, too_few_vals_err_msg));
    }

    if let Ok(y) = py_vec.get_item(py, 1) {
        vec.y = y.extract(py)?;
    } else {
        return Err(PyErr::new::<exc::IndexError, _>(py, too_few_vals_err_msg));
    }

    if let Ok(z) = py_vec.get_item(py, 2) {
        vec.z = z.extract(py)?;
    } else {
        return Err(PyErr::new::<exc::IndexError, _>(py, too_few_vals_err_msg));
    }

    Ok(vec)
}

pub fn get_vec3_named(py: Python, py_vec: PyObject) -> PyResult<Vec3A> {
    Ok(Vec3A::new(py_vec.getattr(py, "x")?.extract(py)?, py_vec.getattr(py, "y")?.extract(py)?, py_vec.getattr(py, "z")?.extract(py)?))
}

pub fn get_vec3_from_dict(py: Python, py_dict: &PyDict, key: &str, name: &str) -> PyResult<Vec3A> {
    match py_dict.get_item(py, key) {
        Some(py_arr) => Ok(get_vec3(py, &py_arr, &format!("Key '{}' in '{}' needs to be a list of exactly 3 numbers", key, name))?),
        None => Err(PyErr::new::<exc::AttributeError, _>(py, format!("No key called '{}' in '{}'.", key, name))),
    }
}

pub fn get_f32_from_dict(py: Python, py_dict: &PyDict, key: &str, name: &str) -> PyResult<f32> {
    match py_dict.get_item(py, key) {
        Some(py_num) => Ok(py_num.extract(py)?),
        None => Err(PyErr::new::<exc::AttributeError, _>(py, format!("No key called '{}' in '{}'.", key, name))),
    }
}

pub fn get_usize_from_dict(py: Python, py_dict: &PyDict, key: &str, name: &str) -> PyResult<usize> {
    match py_dict.get_item(py, key) {
        Some(py_num) => Ok(py_num.extract(py)?),
        None => Err(PyErr::new::<exc::AttributeError, _>(py, format!("No key called '{}' in '{}'.", key, name))),
    }
}

pub fn get_u8_from_dict(py: Python, py_dict: &PyDict, key: &str, name: &str) -> PyResult<u8> {
    match py_dict.get_item(py, key) {
        Some(py_num) => Ok(py_num.extract(py)?),
        None => Err(PyErr::new::<exc::AttributeError, _>(py, format!("No key called '{}' in '{}'.", key, name))),
    }
}

pub fn get_bool_from_dict(py: Python, py_dict: &PyDict, key: &str, name: &str) -> PyResult<bool> {
    match py_dict.get_item(py, key) {
        Some(py_bool) => Ok(py_bool.extract(py)?),
        None => Err(PyErr::new::<exc::AttributeError, _>(py, format!("No key called '{}' in '{}'.", key, name))),
    }
}

pub fn get_vec_from_vec3(vec: Vec3A) -> Vec<f32> {
    vec![vec.x, vec.y, vec.z]
}

pub fn localize_2d_location(car: &Car, vec: Vec3A) -> Vec3A {
    localize_2d(car, vec - car.location)
}

pub fn localize_2d(car: &Car, vec: Vec3A) -> Vec3A {
    Vec3A::new(vec.dot(car.forward), vec.dot(car.right), 0.)
}

// pub fn localize_location(car: &Car, vec: Vec3A) -> Vec3A {
//     localize(car, vec - car.location)
// }

pub fn localize(car: &Car, vec: Vec3A) -> Vec3A {
    Vec3A::new(vec.dot(car.forward), vec.dot(car.right), vec.dot(car.up))
}

// pub fn globalize(car: &Car, vec: Vec3A) -> Vec3A {
//     car.forward * vec.x + car.right * vec.y + car.up * vec.z + car.location
// }

// fn lerp<T: Copy + Add<Output = T> + Sub<Output = T> + Mul<Output = T>>(a: T, b: T, t: T) -> T {
//     // Linearly interpolate from a to b using t
//     // For instance, when t == 0, a is returned, and when t is 1, b is returned
//     // Works for both numbers and Vectors
//     (b - a) * t + a
// }

// fn invlerp<T: Copy + Sub<Output = T> + Div<Output = T>>(a: T, b: T, v: T) -> T {
//     // Inverse linear interpolation from a to b with value v
//     // For instance, it returns 0 if v is a, and returns 1 if v is b, and returns 0.5 if v is exactly between a and b
//     // Works for both numbers and Vectors
//     (v - a) / (b - a)
// }

pub fn turn_radius(v: f32) -> f32 {
    1. / curvature(v)
}

pub fn curvature(v: f32) -> f32 {
    if (0. ..500.).contains(&v) {
        return 0.0069 - 5.84e-6 * v;
    }

    if (500. ..1000.).contains(&v) {
        return 0.00561 - 3.26e-6 * v;
    }

    if (1000. ..1500.).contains(&v) {
        return 0.0043 - 1.95e-6 * v;
    }

    if (1500. ..1750.).contains(&v) {
        return 0.003025 - 1.1e-6 * v;
    }

    if (1750. ..2500.).contains(&v) {
        return 0.0018 - 4e-7 * v;
    }

    panic!("Invalid input velocity");
}

fn clamp_2d(vec: Vec3A, start: Vec3A, end: Vec3A) -> Vec3A {
    let s = vec.normalize_or_zero();
    let right = s.dot(end.cross(-Vec3A::Z)) < 0.;
    let left = s.dot(start.cross(-Vec3A::Z)) > 0.;

    let return_original = if end.dot(start.cross(-Vec3A::Z)) > 0. {
        right && left
    } else {
        right || left
    };

    if return_original {
        vec
    } else if start.dot(s) < end.dot(s) {
        end
    } else {
        start
    }
}

fn angle_2d(vec1: Vec3A, vec2: Vec3A) -> f32 {
    flatten(vec1).dot(flatten(vec2)).clamp(-1., 1.).acos()
}

pub fn get_shot_vector_2d(direction: Vec3A, ball_location: Vec3A, target_left: Vec3A, target_right: Vec3A) -> Vec3A {
    clamp_2d(direction, (target_left - ball_location).normalize_or_zero(), (target_right - ball_location).normalize_or_zero())
}

pub fn flatten(vec: Vec3A) -> Vec3A {
    Vec3A::new(vec.x, vec.y, 0.)
}

// fn clockwise90_2d(vec: Vec3A) -> Vec3A {
//     Vec3A::new(vec.y, -vec.x, 0.)
// }

fn rotate_2d(vec: Vec3A, angle: f32) -> Vec3A {
    Vec3A::new(angle.cos() * vec.x - angle.sin() * vec.y, angle.sin() * vec.x + angle.cos() * vec.y, vec.z)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PostCorrection {
    pub target_left: Vec3A,
    pub target_right: Vec3A,
    pub fits: bool,
}

pub fn correct_for_posts(ball_location: Vec3A, ball_radius: f32, target_left: Vec3A, target_right: Vec3A) -> PostCorrection {
    let goal_line_perp = (target_right - target_left).cross(Vec3A::Z);

    let left_adjusted = target_left + (target_left - ball_location).normalize_or_zero().cross(-Vec3A::Z) * ball_radius;
    let right_adjusted = target_right + (target_right - ball_location).normalize_or_zero().cross(Vec3A::Z) * ball_radius;

    let left_corrected = if (left_adjusted - target_left).dot(goal_line_perp) > 0. {
        target_left
    } else {
        left_adjusted
    };

    let right_corrected = if (right_adjusted - target_right).dot(goal_line_perp) > 0. {
        target_right
    } else {
        right_adjusted
    };

    let left_to_right = right_corrected - left_corrected;
    let new_goal_line = left_to_right.normalize_or_zero();
    let new_goal_width = left_to_right.length();

    let new_goal_perp = new_goal_line.cross(Vec3A::Z);
    let goal_center = left_corrected + new_goal_line * new_goal_width * 0.5;
    let ball_to_goal = (goal_center - ball_location).normalize_or_zero();

    PostCorrection {
        target_left: left_corrected,
        target_right: right_corrected,
        fits: new_goal_width * new_goal_perp.dot(ball_to_goal).abs() > ball_radius * 2.,
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CarFieldRect {
    goal_x: f32,
    goal_y: f32,
    field_y: f32,
    field_x: f32,
}

impl CarFieldRect {
    pub fn from(car_hitbox: &Hitbox) -> Self {
        let half_car_len = car_hitbox.length / 2.;

        Self {
            goal_x: 893. - car_hitbox.width,
            goal_y: 6000. - car_hitbox.length,
            field_y: 5120. - half_car_len,
            field_x: 4093. - half_car_len,
        }
    }

    pub fn is_point_in(&self, p: &[f32; 3]) -> bool {
        if p[0].abs() > self.goal_x {
            return p[0].abs() < self.field_x && p[1].abs() < self.field_y;
        }

        p[1].abs() < self.goal_y
    }
}

fn path_point_to_vec3(endpoint: [f32; 3]) -> Vec3A {
    Vec3A::new(endpoint[0], endpoint[1], 0.)
}

fn shortest_path_in_validate(q0: [f32; 3], q1: [f32; 3], rho: f32, car_field: &CarFieldRect, max_distance: f32, validate: bool) -> Result<DubinsPath, DubinsError> {
    let mut best_cost = INFINITY;
    let mut best_path = None;

    let intermediate_results = intermediate_results(q0, q1, rho)?;

    for path_type in DubinsPathType::ALL {
        if let Ok(param) = word(&intermediate_results, path_type) {
            let cost = param[0] + param[1] + param[2];
            if cost < best_cost && (!validate || cost * rho <= max_distance) {
                let path = DubinsPath {
                    qi: q0,
                    rho,
                    param,
                    type_: path_type,
                };

                let mut valid = true;

                for dist in [path.param[0] * path.rho / 2., (path.param[0] + path.param[1]) * path.rho / 2., (path.param[0] + path.param[1] + path.param[2]) * path.rho / 2.] {
                    if !car_field.is_point_in(&path_sample(&path, dist)?) {
                        valid = false;
                        break;
                    }
                }

                if !valid {
                    continue;
                }

                for (i, dist) in [path.param[0] * path.rho, (path.param[0] + path.param[1]) * path.rho, (path.param[0] + path.param[1] + path.param[2]) * path.rho].iter().enumerate() {
                    if !car_field.is_point_in(&path_sample(&path, *dist)?) {
                        valid = false;
                        break;
                    }
                }

                if valid {
                    best_cost = cost;
                    best_path = Some(path);
                }
            }
        }
    }

    match best_path {
        Some(path) => Ok(path),
        None => Err(DubinsError::NoPath),
    }
}

// pub fn segment_angle(t: f32, angle: f32, type_: SegmentType) -> f32 {
//     (match type_ {
//         SegmentType::L => t,
//         SegmentType::R => -t,
//         SegmentType::S => 0.,
//     }) + angle
// }

// fn path_angle(path: &DubinsPath, t: f32) -> Result<f32, DubinsError> {
//     if t < 0. || t > path_length(path) {
//         return Err(DubinsError::Param);
//     }

//     /* tprime is the normalised variant of the parameter t */
//     let tprime = t / path.rho;
//     let types = DIRDATA[path.type_.to()];

//     /* generate the target configuration */
//     let p1 = path.param[0];
//     let p2 = path.param[1];

//     let q1 = segment_angle(p1, path.qi[2], types[0]); // end-of segment 1
//     let q2 = segment_angle(p2, q1, types[1]); // end-of segment 2

//     Ok(mod2pi(if tprime < p1 {
//         segment_angle(tprime, path.qi[2], types[0])
//     } else if tprime < p1 + p2 {
//         segment_angle(tprime - p1, q1, types[1])
//     } else {
//         segment_angle(tprime - p1 - p2, q2, types[2])
//     }))
// }

#[derive(Clone, Copy, Debug, Default)]
pub struct TargetInfo {
    pub distances: [f32; 4],
    pub target: Option<Vec3A>,
    pub angle: Option<f32>,
    pub path: Option<DubinsPath>,
}

impl TargetInfo {
    pub const fn new_dubin(distances: [f32; 4], path: DubinsPath) -> Self {
        Self {
            distances,
            target: None,
            angle: None,
            path: Some(path),
        }
    }
    pub const fn new_dubin_target(distances: [f32; 4], target: Vec3A, angle: Option<f32>, path: DubinsPath) -> Self {
        Self {
            distances,
            target: Some(target),
            path: Some(path),
            angle,
        }
    }

    pub const fn new(distances: [f32; 4]) -> Self {
        Self {
            distances,
            target: None,
            angle: None,
            path: None,
        }
    }

    pub const fn new_target(distances: [f32; 4], target: Vec3A, angle: Option<f32>) -> Self {
        Self {
            distances,
            target: Some(target),
            path: None,
            angle,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AnalyzeOptions {
    pub max_speed: f32,
    pub max_turn_radius: f32,
    pub get_target: bool,
    pub validate: bool,
}

pub fn analyze_target(ball: &Ball, car: &Car, shot_vector: Vec3A, time_remaining: f32, options: AnalyzeOptions) -> Result<TargetInfo, DubinsError> {
    let offset_target = ball.location - (shot_vector * ball.radius);
    let car_front_length = (car.hitbox_offset.x + car.hitbox.length) / 2.;

    let mut end_distance = -car_front_length;

    let offset_distance = 640.;
    let exit_turn_point = offset_target - (shot_vector * offset_distance);

    let local_offset = localize_2d_location(car, offset_target);

    let margin = car.hitbox.width / 3.;

    let target_angle = shot_vector.y.atan2(shot_vector.x);

    if local_offset.x > 0. {
        let car_to_shot = angle_2d(car.forward, shot_vector);

        if car_to_shot < 0.15 && local_offset.y.abs() < margin {
            end_distance += local_offset.x;

            let distances = [0., 0., 0., end_distance];

            return Ok(if options.get_target {
                let final_target = car.location + flatten(shot_vector) * local_offset.x;

                TargetInfo::new_target(distances, final_target, None)
            } else {
                TargetInfo::new(distances)
            });
        } else if car_to_shot < FRAC_PI_2 {
            let car_to_exit = exit_turn_point - car.location;
            if car.forward.dot(car_to_exit) > 0. {
                let turn_center = car.right * options.max_turn_radius;
                let turn_center_to_car_location = -turn_center;
                let turn_center_to_exit_turn_point = car_to_exit - turn_center;

                let turn_angle = angle_2d(turn_center_to_car_location.normalize(), turn_center_to_exit_turn_point.normalize());

                let exit_point = rotate_2d(turn_center_to_car_location, -turn_angle);

                if exit_turn_point.distance(exit_point) < margin / 2. {
                    end_distance += offset_distance;

                    let turn_distance = options.max_turn_radius * turn_angle;

                    let distances = [0., 0., turn_distance, end_distance];

                    return Ok(if options.get_target {
                        TargetInfo::new_target(distances, exit_turn_point, Some(target_angle))
                    } else {
                        TargetInfo::new(distances)
                    });
                }
            }
            // let car_location_2d = flatten(car.location);
            // let exit_2d = flatten(exit_turn_point);
            // let shot_vector_rotated = clockwise90_2d(shot_vector);

            // let exit_points = [exit_2d + shot_vector_rotated * (margin / 2.), exit_2d - shot_vector_rotated * (margin / 2.)];
            // let turn_radii = [
            //     radius_from_points_with_directions(car_location_2d, car.forward, exit_points[0], shot_vector),
            //     radius_from_points_with_directions(car_location_2d, car.forward, exit_points[1], shot_vector),
            // ];

            // if get_target {
            //     dbg!(car.ctrms);
            //     dbg!(radius_from_points_with_directions(car_location_2d, car.forward, exit_2d, shot_vector));
            //     dbg!(turn_radii[0].min(turn_radii[1]));
            //     dbg!(turn_radii[0].max(turn_radii[1]));
            // }

            // if (turn_radii[0].min(turn_radii[1])..turn_radii[0].max(turn_radii[1])).contains(&car.ctrms) {
            //     end_distance += offset_distance;

            //     let turn_rad = radius_from_points_with_directions(car_location_2d, car.forward, exit_2d, shot_vector);
            //     let turn_distance = turn_rad * car_to_shot;

            //     let distances = [0., 0., turn_distance, end_distance];

            //     return Ok(if get_target {
            //         TargetInfo::new_target(distances, exit_turn_point, Some(target_angle))
            //     } else {
            //         TargetInfo::new(distances)
            //     });
            // }
        }
    }

    end_distance += offset_distance;

    let max_distance = time_remaining * options.max_speed;

    if options.validate && (end_distance + flatten(car.location).distance(flatten(exit_turn_point))) > max_distance {
        return Err(DubinsError::NoPath);
    }

    let q0 = [car.location.x, car.location.y, car.yaw];
    let q1 = [exit_turn_point.x, exit_turn_point.y, target_angle];

    let path = shortest_path_in_validate(q0, q1, options.max_turn_radius, &car.field, max_distance, options.validate)?;

    let distances = [segment_length(&path, 0), segment_length(&path, 1), segment_length(&path, 2), end_distance];

    Ok(if options.get_target {
        let distance = (car.local_velocity.x.max(500.) * STEER_REACTION_TIME).min(path_length(&path));
        let sample = path_sample(&path, distance)?;

        let angle = if (distances[0]..(distances[0] + distances[1])).contains(&distance) && DIRDATA[path.type_.to()][1] == SegmentType::S {
            None
        } else {
            Some(sample[2])
        };

        let target = path_point_to_vec3(sample);

        TargetInfo::new_dubin_target(distances, target, angle, path)
    } else {
        TargetInfo::new_dubin(distances, path)
    })
}

fn throttle_acceleration(forward_velocity: f32) -> f32 {
    let x = forward_velocity.abs();

    if x >= MAX_SPEED_NO_BOOST {
        return 0.;
    }

    // use y = mx + b to find the throttle acceleration
    if x < THROTTLE_ACCEL_DIVISION {
        return START_THROTTLE_ACCEL_M * x + START_THROTTLE_ACCEL_B;
    }

    END_THROTTLE_ACCEL_M * (x - THROTTLE_ACCEL_DIVISION) + END_THROTTLE_ACCEL_B
}

pub fn can_reach_target(car: &Car, max_speed: f32, max_time: f32, distance_remaining: f32, is_forwards: bool) -> Result<f32, ()> {
    let mut d = distance_remaining;
    let mut t_r = max_time;
    let mut b = car.boost as f32;
    let mut v = car.local_velocity.x;

    let direction = is_forwards as u8 as f32;

    loop {
        if d <= 0. {
            return Ok(t_r);
        }

        let r = d * direction / t_r.max(0.);

        if t_r < -SIMULATION_DT || (is_forwards && r > max_speed) || (!is_forwards && MIN_SPEED > r) {
            return Err(());
        }

        let t = r - v;

        if t.abs() < 100. {
            break;
        }

        let acceleration = t / REACTION_TIME;

        let throttle_accel = throttle_acceleration(v);
        let throttle_boost_transition = throttle_accel + 0.5 * BOOST_ACCEL;

        let throttle: f32;

        if acceleration <= BRAKE_COAST_TRANSITION {
            throttle = -1.;
        } else if BRAKE_COAST_TRANSITION < acceleration && acceleration < COASTING_THROTTLE_TRANSITION {
            throttle = 0.;
        } else if COASTING_THROTTLE_TRANSITION <= acceleration && acceleration <= throttle_boost_transition {
            throttle = if throttle_accel == 0. {
                1.
            } else {
                (acceleration / throttle_accel).clamp(0.02, 1.)
            };
        } else if b >= MIN_BOOST_CONSUMPTION && throttle_boost_transition < acceleration {
            throttle = 1.;

            if t > 0. {
                v += BOOST_ACCEL_DT;
                b -= BOOST_CONSUMPTION_DT;
            }
        } else {
            throttle = 0.;
        }

        if throttle == 0. {
            v += COAST_ACC * SIMULATION_DT;
        } else if throttle.signum() == v.signum() {
            v += throttle_accel * SIMULATION_DT * throttle;
        } else {
            v += BRAKE_ACC_DT.copysign(throttle);
        }

        t_r -= SIMULATION_DT;
        d -= (v * direction) * SIMULATION_DT;
    }

    Ok(t_r)
}

#[derive(Clone, Copy, Debug)]
pub struct Options {
    pub all: bool,
    pub use_absolute_max_values: bool,
    pub min_slice: usize,
    pub max_slice: usize,
}

pub fn get_options_from_dict(py: Python, py_options: PyDict, max_slices: usize) -> PyResult<Options> {
    let all = get_bool_from_dict(py, &py_options, "all", "options").unwrap_or(false);

    let use_absolute_max_values = get_bool_from_dict(py, &py_options, "use_absolute_max_values", "options").unwrap_or(false);

    let min_slice = match get_usize_from_dict(py, &py_options, "min_slice", "options") {
        Ok(u) => u.max(0),
        Err(_) => 0,
    };

    let max_slice = match get_usize_from_dict(py, &py_options, "min_slice", "options") {
        Ok(u) => u.min(max_slices),
        Err(_) => max_slices,
    };

    Ok(Options {
        all,
        use_absolute_max_values,
        min_slice,
        max_slice,
    })
}
