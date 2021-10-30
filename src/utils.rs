// use std::{
//     cmp::Ordering,
//     f32::{
//         consts::{PI, TAU},
//         EPSILON,
//     },
//     io::{Cursor, ErrorKind},
// };

// use byteorder::{BigEndian, ReadBytesExt};
use cpython::{exc, ObjectProtocol, PyDict, PyErr, PyObject, PyResult, Python};
use dubins_paths::{path_sample, shortest_path, DubinsError, DubinsPath};
use rl_ball_sym::simulation::ball::Ball;

use vvec3::Vec3;

pub const MAX_SPEED: f32 = 2300.;
pub const MAX_SPEED_NO_BOOST: f32 = 1410.;
pub const MIN_SPEED: f32 = -MAX_SPEED_NO_BOOST;
pub const MAX_TURN_RADIUS: f32 = 1. / 0.00088;
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

// pub struct TurnLut {
//     pub time_sorted: Vec<TurnLutEntry>,
//     pub velocity_sorted: Vec<TurnLutEntry>,
// }

// impl TurnLut {
//     pub fn from(mut entries: Vec<TurnLutEntry>) -> Self {
//         entries.sort_unstable_by(|a, b| a.time.partial_cmp(&b.time).unwrap_or(Ordering::Equal));

//         for (time_index, entry) in entries.iter_mut().enumerate() {
//             entry.time_sorted_index = time_index;
//         }

//         let time_sorted = entries.clone();

//         entries.dedup_by_key(|e| e.velocity);
//         entries.sort_by(|a, b| a.velocity.partial_cmp(&b.velocity).unwrap_or(Ordering::Equal));
//         entries.dedup_by_key(|e| e.velocity);

//         Self {
//             time_sorted,
//             velocity_sorted: entries,
//         }
//     }
// }

// #[derive(Clone, Copy)]
// pub struct TurnLutEntry {
//     pub time: f32,
//     pub velocity: i16,
//     pub distance: u16,
//     pub location: Vec3,
//     pub yaw: f32,
//     pub time_sorted_index: usize,
// }

// pub fn read_turn_bin(bin_data: Vec<u8>) -> Vec<TurnLutEntry> {
//     let mut cursor = Cursor::new(bin_data);
//     let mut lut: Vec<TurnLutEntry> = Vec::new();

//     loop {
//         let time = match cursor.read_u16::<BigEndian>() {
//             Ok(num) => num as f32 / 7222.,
//             Err(error) => match error.kind() {
//                 ErrorKind::UnexpectedEof => break,
//                 other_error => {
//                     panic!("Problem parsing file: {:?}", other_error)
//                 }
//             },
//         };

//         let location = Vec3 {
//             x: match cursor.read_i16::<BigEndian>() {
//                 Ok(num) => num as f32 / 6.,
//                 Err(error) => panic!("Problem parsing file: {:?}", error),
//             },
//             y: match cursor.read_i16::<BigEndian>() {
//                 Ok(num) => num as f32 / 6.,
//                 Err(error) => panic!("Problem parsing file: {:?}", error),
//             },
//             z: 0.,
//         };

//         let velocity = match cursor.read_i16::<BigEndian>() {
//             Ok(num) => num,
//             Err(error) => panic!("Problem parsing file: {:?}", error),
//         };

//         let yaw = match cursor.read_i16::<BigEndian>() {
//             Ok(num) => num as f32 / 10200.,
//             Err(error) => panic!("Problem parsing file: {:?}", error),
//         };

//         let distance = match cursor.read_u16::<BigEndian>() {
//             Ok(num) => num,
//             Err(error) => panic!("Problem parsing file: {:?}", error),
//         };

//         lut.push(TurnLutEntry {
//             time,
//             velocity,
//             distance,
//             location,
//             yaw,
//             time_sorted_index: 0,
//         });
//     }

//     lut
// }

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
    pub forward: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub hitbox: Hitbox,
    pub hitbox_offset: Vec3,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub boost: u8,
    pub demolished: bool,
    pub airborne: bool,
    pub jumped: bool,
    pub doublejumped: bool,
    pub max_speed: f32,
    pub max_turn_radius: f32,
}

impl Default for Car {
    fn default() -> Self {
        Self {
            location: Vec3::default(),
            velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            forward: Vec3::default(),
            right: Vec3::default(),
            up: Vec3::default(),
            hitbox: Hitbox::default(),
            hitbox_offset: Vec3::default(),
            pitch: 0.,
            yaw: 0.,
            roll: 0.,
            boost: 0,
            demolished: false,
            airborne: false,
            jumped: false,
            doublejumped: false,
            max_speed: 0.,
            max_turn_radius: 0.,
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

    pub fn calculate_max_values(&mut self) {
        self.max_speed = self.get_max_speed();
        self.max_turn_radius = turn_radius(self.max_speed);
    }

    fn get_max_speed(&mut self) -> f32 {
        let mut b = self.boost as f32;
        let mut v = self.velocity.dot(&self.forward);

        loop {
            // dbg!(v);
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

            if v.signum() == 1. {
                // println!("ta: {}", throttle_acceleration(v));
                v += throttle_acceleration(v) * SIMULATION_DT;
            } else {
                v += -BRAKE_ACC_DT;
            }

            v += BOOST_ACCEL_DT;
            b -= BOOST_CONSUMPTION_DT;
        }
    }
}

pub fn get_vec3(py: Python, py_vec: &PyObject, too_few_vals_err_msg: &str) -> PyResult<Vec3> {
    let mut vec = Vec3::default();

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

pub fn get_vec3_from_dict(py: Python, py_dict: &PyDict, key: &str, name: &str) -> PyResult<Vec3> {
    match py_dict.get_item(py, key) {
        Some(py_arr) => {
            return Ok(get_vec3(py, &py_arr, &format!("Key '{}' in '{}' needs to be a list of exactly 3 numbers", key, name))?);
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, format!("No key called '{}' in '{}'.", key, name)));
        }
    }
}

pub fn get_f32_from_dict(py: Python, py_dict: &PyDict, key: &str, name: &str) -> PyResult<f32> {
    match py_dict.get_item(py, key) {
        Some(py_num) => {
            return Ok(py_num.extract(py)?);
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, format!("No key called '{}' in '{}'.", key, name)));
        }
    }
}

pub fn get_u8_from_dict(py: Python, py_dict: &PyDict, key: &str, name: &str) -> PyResult<u8> {
    match py_dict.get_item(py, key) {
        Some(py_num) => {
            return Ok(py_num.extract(py)?);
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, format!("No key called '{}' in '{}'.", key, name)));
        }
    }
}

pub fn get_bool_from_dict(py: Python, py_dict: &PyDict, key: &str, name: &str) -> PyResult<bool> {
    match py_dict.get_item(py, key) {
        Some(py_bool) => {
            return Ok(py_bool.extract(py)?);
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, format!("No key called '{}' in '{}'.", key, name)));
        }
    }
}

pub fn get_vec_from_vec3(vec: Vec3) -> Vec<f32> {
    vec![vec.x, vec.y, vec.z]
}

pub fn localize_2d(car: &Car, vec: Vec3) -> Vec3 {
    let vec = vec - car.location;

    Vec3 {
        x: vec.dot(&car.forward),
        y: vec.dot(&car.right),
        z: 0.,
    }
}

// pub fn localize(car: &Car, vec: Vec3) -> Vec3 {
//     let vec = vec - car.location;

//     Vec3 {
//         x: vec.dot(&car.forward),
//         y: vec.dot(&car.right),
//         z: vec.dot(&car.up),
//     }
// }

// pub fn globalize(car: &Car, vec: Vec3) -> Vec3 {
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

// pub fn get_turn_time_from_speed(speed: f32) -> f32 {
//     let speed_adj = -speed / VMAX - 1.;
//     -speed_adj.ln() * TAU
// }

// pub fn get_speed_from_time_turning(time: f32) -> f32 {
//     VMAX * (1. - (-(time / TAU)).exp())
// }

// pub fn get_turn_time_from_speed_with_boost(speed: f32) -> f32 {
//     match speed {
//         y if y < 1410. => (y + 26.11) / 1535.948,
//         y if y < 2295. => (y - 829.316) / 621.05,
//         _ => 2.36,
//     }
// }

// pub fn get_speed_from_time_turning_with_boost(time: f32) -> f32 {
//     match time {
//         x if x < 0.017 => 0.,
//         x if x <= 0.935 => 1535.948 * x - 26.111,
//         x if x <= 2.36 => 621.05 * x + 829.316,
//         _ => 2295.,
//     }
// }

pub fn turn_radius(v: f32) -> f32 {
    1. / curvature(v)
}

pub fn curvature(v: f32) -> f32 {
    if 0. <= v && v < 500. {
        return 0.0069 - 5.84e-6 * v;
    }

    if 500. <= v && v < 1000. {
        return 0.00561 - 3.26e-6 * v;
    }

    if 1000. <= v && v < 1500. {
        return 0.0043 - 1.95e-6 * v;
    }

    if 1500. <= v && v < 1750. {
        return 0.003025 - 1.1e-6 * v;
    }

    if 1750. <= v && v < 2500. {
        return 0.0018 - 4e-7 * v;
    }

    0.
}

const VEC3_DOWN: Vec3 = Vec3 {
    x: 0.,
    y: 0.,
    z: -1.,
};

const VEC3_UP: Vec3 = Vec3 {
    x: 0.,
    y: 0.,
    z: 1.,
};

fn clamp_2d(vec: Vec3, start: Vec3, end: Vec3) -> Vec3 {
    let s = vec.normalize();
    let right = s.dot(&end.cross(&VEC3_DOWN)) < 0.;
    let left = s.dot(&start.cross(&VEC3_DOWN)) > 0.;

    let return_original = if end.dot(&start.cross(&VEC3_DOWN)) > 0. {
        right && left
    } else {
        right || left
    };

    if return_original {
        vec
    } else if start.dot(&s) < end.dot(&s) {
        end
    } else {
        start
    }
}

pub fn get_shot_vector_2d(direction: Vec3, ball_location: Vec3, target_left: Vec3, target_right: Vec3) -> Vec3 {
    clamp_2d(direction, (target_left - ball_location).normalize(), (target_right - ball_location).normalize())
}

pub struct PostCorrection {
    pub target_left: Vec3,
    pub target_right: Vec3,
    pub fits: bool,
}

pub fn correct_for_posts(ball_location: Vec3, ball_radius: f32, target_left: Vec3, target_right: Vec3) -> PostCorrection {
    let goal_line_perp = (target_right - target_left).cross(&VEC3_UP);

    let left_adjusted = target_left + (target_left - ball_location).normalize().cross(&VEC3_DOWN) * ball_radius;
    let right_adjusted = target_right + (target_right - ball_location).normalize().cross(&VEC3_UP) * ball_radius;

    let left_corrected = if (left_adjusted - target_left).dot(&goal_line_perp) > 0. {
        target_left
    } else {
        left_adjusted
    };

    let right_corrected = if (right_adjusted - target_right).dot(&goal_line_perp) > 0. {
        target_right
    } else {
        right_adjusted
    };

    let left_to_right = right_corrected - left_corrected;
    let new_goal_line = left_to_right.normalize();
    let new_goal_width = left_to_right.magnitude();

    let new_goal_perp = new_goal_line.cross(&VEC3_UP);
    let goal_center = left_corrected + new_goal_line * new_goal_width * 0.5;
    let ball_to_goal = (goal_center - ball_location).normalize();

    PostCorrection {
        target_left: left_corrected,
        target_right: right_corrected,
        fits: new_goal_width * new_goal_perp.dot(&ball_to_goal).abs() > ball_radius * 2.,
    }
}

// pub fn binary_search_turn_lut_velocity(lut: &TurnLut, velocity: i16) -> usize {
//     let mut left = 0;
//     let mut right = lut.velocity_sorted.len() - 1;

//     while right - left > 0 {
//         let mid = (left + right) / 2;
//         let slice = &lut.velocity_sorted[mid];

//         let cmp = slice.velocity.cmp(&velocity);

//         // The reason why we use if/else control flow rather than match
//         // is because match reorders comparison operations, which is perf sensitive.
//         // This is x86 asm for u8: https://rust.godbolt.org/z/8Y8Pra.
//         if cmp == Ordering::Less {
//             left = mid + 1;
//         } else if cmp == Ordering::Greater {
//             right = mid;
//         } else {
//             return mid;
//         }
//     }

//     left
// }

// pub fn binary_search_turn_lut_time(lut: &TurnLut, time: &f32) -> usize {
//     let mut left = 0;
//     let mut right = lut.time_sorted.len();

//     while right - left > 0 {
//         let mid = (left + right) / 2;
//         let slice = &lut.time_sorted[mid];

//         let cmp = slice.time.partial_cmp(&time).unwrap_or(Ordering::Equal);

//         // The reason why we use if/else control flow rather than match
//         // is because match reorders comparison operations, which is perf sensitive.
//         // This is x86 asm for u8: https://rust.godbolt.org/z/8Y8Pra.
//         if cmp == Ordering::Less {
//             left = mid + 1;
//         } else if cmp == Ordering::Greater {
//             right = mid;
//         } else {
//             return mid;
//         }
//     }

//     left
// }

// pub fn linear_search_turn_lut_target(lut: &TurnLut, required_yaw_change: f32, start_index: usize) -> usize {
//     let mut index = start_index;
//     let mut current_yaw = 0.;

//     let start_yaw = lut.time_sorted[index].yaw;

//     let mut next_index = index + 3;
//     let mut next_yaw = lut.time_sorted[next_index].yaw - start_yaw;

//     // in theory, we should only ever be turning right at most 180 degrees, so this should be fine
//     while (required_yaw_change - next_yaw).abs() <= (required_yaw_change - current_yaw).abs() {
//         index = next_index;
//         current_yaw = next_yaw;

//         next_index = index + 3;
//         next_yaw = lut.time_sorted[next_index].yaw - start_yaw;
//     }

//     index
// }

// fn is_point_in_field(car: &Car, target: Vec3) -> bool {
//     target.y.abs() < 5120. - car.hitbox.length && target.x.abs() < 4093. - car.hitbox.length
// }

// pub fn is_circle_in_field(car_hitbox: &Hitbox, target: Vec3, circle_radius: f32) -> bool {
//     target.y.abs() < 5120. - car_hitbox.length - circle_radius && target.x.abs() < 4093. - car_hitbox.length - circle_radius
// }

pub struct CarFieldRect {
    u_1: f32,
    u_2: f32,
    v_1: f32,
    v_2: f32,
    a_1: f32,
    b_1: f32,
    a_2: f32,
    b_2: f32,
}

// https://math.stackexchange.com/a/1938581/689600
impl CarFieldRect {
    pub fn from(car_hitbox: &Hitbox, car_y: f32) -> Self {
        let margin = car_hitbox.length / 2.;

        let mut length = 5120. - margin;
        let extend = car_y.abs() >= length;
        if extend {
            length = 6000. - margin;
        }

        let width = 4093. - margin;

        let field_1 = Vec3::new(length, width, 0.);
        let field_2 = Vec3::new(-length, width, 0.);
        let field_4 = Vec3::new(length, -width, 0.);

        let u_1 = field_2.x - field_1.x;
        let v_1 = field_2.y - field_1.y;
        let u_2 = field_4.x - field_1.x;
        let v_2 = field_4.y - field_1.y;

        CarFieldRect {
            u_1,
            v_1,
            u_2,
            v_2,
            a_1: field_1.x * u_1 + field_1.y * v_1,
            b_1: field_2.x * u_1 + field_2.y * v_1,
            a_2: field_1.x * u_2 + field_1.y * v_2,
            b_2: field_4.x * u_2 + field_4.y * v_2,
        }
    }

    pub fn is_triangle_in(&self, p: [[f32; 3]; 3]) -> bool {
        self.is_point_in(p[0]) && self.is_point_in(p[1]) && self.is_point_in(p[2])
    }

    fn is_point_in(&self, p: [f32; 3]) -> bool {
        let p_1 = p[0] * self.u_1 + p[1] * self.v_1;
        let p_2 = p[0] * self.u_2 + p[1] * self.v_2;

        self.a_1 < p_1 && p_1 < self.b_1 && self.a_2 < p_2 && p_2 < self.b_2
    }

    // pub fn is_triangle_in(&self, p_1: Vec3, p_2: Vec3, p_3: Vec3) -> bool {
    //     self.is_point_in(p_1) && self.is_point_in(p_2) && self.is_point_in(p_3)
    // }

    // fn is_point_in(&self, p: Vec3) -> bool {
    //     let p_1 = p.x * self.u_1 + p.y * self.v_1;
    //     let p_2 = p.x * self.u_2 + p.y * self.v_2;

    //     self.a_1 < p_1 && p_1 < self.b_1 && self.a_2 < p_2 && p_2 < self.b_2
    // }
}

fn path_section_endpoints(path: &DubinsPath) -> Result<[[f32; 3]; 3], DubinsError> {
    let dists = [path.param[0] * path.rho, (path.param[0] + path.param[1]) * path.rho, (path.param[0] + path.param[1] + path.param[2]) * path.rho];

    Ok([path_sample(path, dists[0])?, path_sample(path, dists[1])?, path_sample(path, dists[2] - f32::EPSILON)?])
}

fn path_section_midpoints(path: &DubinsPath) -> Result<[[f32; 3]; 3], DubinsError> {
    let dists = [path.param[0] * path.rho, (path.param[0] + path.param[1]) * path.rho, (path.param[0] + path.param[1] + path.param[2]) * path.rho];

    Ok([path_sample(path, dists[0] / 2.)?, path_sample(path, dists[1] / 2.)?, path_sample(path, dists[2] / 2.)?])
}

fn path_endpoint_to_vec3(endpoint: [f32; 3]) -> Vec3 {
    Vec3 {
        x: endpoint[0],
        y: endpoint[1],
        z: 0.,
    }
}

fn radius_from_local_point(a: Vec3) -> f32 {
    let b = a.flatten();
    1. / (2. * b.y / a.dot(&b)).abs()
}

pub fn analyze_target(ball: &Box<Ball>, car: &Car, shot_vector: Vec3, get_target: bool, validate: bool) -> Result<([f32; 4], Option<Vec3>, Option<DubinsPath>), DubinsError> {
    let offset_target = ball.location - (shot_vector * ball.collision_radius);
    let car_front_length = car.hitbox_offset.x + car.hitbox.length / 2.;
    let local_offset = localize_2d(car, offset_target);

    let mut end_distance = -car_front_length;
    let angle_to_shot = car.forward.angle_2d(&shot_vector);

    // lock onto the offset target if we're facing it
    if angle_to_shot < 0.1 {
        if local_offset.x > 0. && local_offset.y.abs() < 100. {
            end_distance += car.location.dist_2d(offset_target);

            let final_target = if get_target {
                Some(offset_target)
            } else {
                None
            };

            return Ok(([0., 0., 0., end_distance], final_target, None));
        }
    }

    let offset_distance = 640.;
    let exit_turn_point = offset_target - (shot_vector * offset_distance);

    end_distance += offset_distance;

    // if get_target {
    //     println!("{} | {} | {} | {} ", local_offset.x, car.location.dist_2d(exit_turn_point), radius_from_local_point(exit_turn_point), angle_to_shot);
    // }

    if local_offset.x > 0. && angle_to_shot < 1.3 && car.location.dist_2d(exit_turn_point) < MAX_TURN_RADIUS * 2. && radius_from_local_point(localize_2d(car, exit_turn_point)) < MAX_TURN_RADIUS * 1.05 {
        let final_target = if get_target {
            Some(exit_turn_point)
        } else {
            None
        };
        let turn_distance = angle_to_shot * car.max_turn_radius;

        return Ok(([0., 0., turn_distance, end_distance], final_target, None));
    }

    let q0 = [car.location.x, car.location.y, car.yaw];
    let q1 = [exit_turn_point.x, exit_turn_point.y, shot_vector.y.atan2(shot_vector.x)];

    let path = shortest_path(q0, q1, car.max_turn_radius * 1.02)?;
    let end_parts = path_section_endpoints(&path)?;

    if validate {
        let car_field = CarFieldRect::from(&car.hitbox, car.location.y);
        let midpoints = path_section_midpoints(&path)?;

        if !car_field.is_triangle_in([path.qi, midpoints[0], end_parts[0]]) || !car_field.is_triangle_in([end_parts[1], midpoints[2], end_parts[2]]) {
            return Err(DubinsError::NoPath);
        }
    }

    let distances = [path.param[0] * path.rho, path.param[1] * path.rho, path.param[2] * path.rho];

    let final_target = if get_target {
        let end_part = if distances[0] > 25. {
            0
        } else if distances[1] > 25. {
            1
        } else {
            2
        };

        Some(path_endpoint_to_vec3(end_parts[end_part]))
    } else {
        None
    };

    Ok(([distances[0], distances[1], distances[2], end_distance], final_target, Some(path)))
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

    return END_THROTTLE_ACCEL_M * (x - THROTTLE_ACCEL_DIVISION) + END_THROTTLE_ACCEL_B;
}

pub fn can_reach_target(car: &Car, max_time: f32, distance_remaining: f32, is_forwards: bool) -> Result<f32, ()> {
    let mut d = distance_remaining;
    let mut t_r = max_time;
    let mut b = car.boost as f32;
    let mut v = car.velocity.dot(&car.forward);

    let direction = is_forwards as u8 as f32;

    loop {
        if d <= 0. {
            return Ok(t_r);
        }

        let r = d * direction / t_r;

        if MIN_SPEED > r || r > car.max_speed || t_r < -SIMULATION_DT {
            return Err(());
        }

        let t = r - v;

        if t.abs() < 100. {
            break;
        }

        let acceleration = t / REACTION_TIME;

        let throttle_accel = throttle_acceleration(v);
        let throttle_boost_transition = throttle_accel + 0.5 * BOOST_ACCEL;

        let mut throttle = 0_f32;
        let mut boost = false;

        if acceleration <= BRAKE_COAST_TRANSITION {
            throttle = -1.;
        } else if BRAKE_COAST_TRANSITION < acceleration && acceleration < COASTING_THROTTLE_TRANSITION {
        } else if b >= MIN_BOOST_CONSUMPTION && throttle_boost_transition < acceleration {
            throttle = 1.;
            if t > 0. {
                boost = true;
            }
        }

        if throttle.signum() == v.signum() {
            v += throttle_accel * SIMULATION_DT * throttle;
        } else {
            v += BRAKE_ACC_DT.copysign(-throttle);
        }

        if boost {
            v += BOOST_ACCEL_DT;
            b -= BOOST_CONSUMPTION_DT;
        }

        t_r -= SIMULATION_DT;
        d -= (v * direction) * SIMULATION_DT;
    }

    Ok(t_r)
}

// pub struct TurnInfo {
//     pub car_location: Vec3,
//     pub car_forward: Vec3,
//     pub car_speed: i16,
//     pub car_yaw: f32,
//     pub distance: u16,
//     pub time: f32,
// }

// impl Default for TurnInfo {
//     fn default() -> Self {
//         Self {
//             car_location: Vec3::default(),
//             car_forward: Vec3::default(),
//             car_speed: 0,
//             car_yaw: 0.,
//             distance: 0,
//             time: 0.,
//         }
//     }
// }

// impl TurnInfo {
//     pub fn calc_turn_info(car: &Car, target: &Vec3, turn_accel_lut: &TurnLut, /*turn_accel_boost_lut: &TurnLut, */ turn_decel_lut: &TurnLut) -> Self {
//         let mut target = *target;
//         let mut local_target = localize_2d(car, target);
//         let inverted = local_target.y < 0.;

//         if inverted {
//             local_target.y = -local_target.y;
//             target = globalize(car, local_target)
//         }

//         let required_yaw_change = angle_2d(&car.forward, &(target - car.location).normalize());

//         let mut car_speed = car.velocity.dot(&car.forward).round() as i16;

//         let mut car_location = car.location;
//         let mut car_forward = car.forward;
//         let mut car_yaw = car.yaw;
//         let mut distance = 0;
//         let mut time = 0.;
//         // let mut boost = car.boost;

//         // let mut done = false;

//         // if boost >= 4 {
//         //     let start_velocity_index = binary_search_turn_lut_velocity(turn_accel_boost_lut, car_speed);
//         //     let start_slice = &turn_accel_boost_lut.velocity_sorted[start_velocity_index];

//         //     let final_time_index = linear_search_turn_lut_target(turn_accel_boost_lut, required_yaw_change, start_slice.time_sorted_index);
//         //     let mut final_slice = &turn_accel_boost_lut.time_sorted[final_time_index];

//         //     {
//         //         let max_boost_time = boost as f32 / BOOST_CONSUMPTION;
//         //         let turn_time = final_slice.time - start_slice.time;

//         //         if max_boost_time < turn_time {
//         //             let final_time_index = binary_search_turn_lut_time(turn_accel_boost_lut, &max_boost_time);
//         //             final_slice = &turn_accel_boost_lut.time_sorted[final_time_index];
//         //             boost = 0;
//         //         } else {
//         //             done = true;
//         //             boost -= (turn_time * BOOST_CONSUMPTION) as u8;
//         //         }
//         //     }

//         //     let part_distance = final_slice.distance - start_slice.distance;

//         //     match inverted {
//         //         false => {
//         //             let rotation = final_slice.yaw - start_slice.yaw;
//         //             car_yaw += rotation;
//         //             if car_yaw > PI {
//         //                 car_yaw -= TAU;
//         //             }

//         //             car_forward = rotate_2d(&car_forward, &rotation);

//         //             car_location += (final_slice.location - start_slice.location).scale(part_distance as f32);
//         //         }
//         //         true => {
//         //             let rotation = -1. * (final_slice.yaw - start_slice.yaw);

//         //             car_yaw += rotation;
//         //             if car_yaw > PI {
//         //                 car_yaw -= TAU;
//         //             }

//         //             car_forward = rotate_2d(&car_forward, &rotation);

//         //             let location_dir = (final_slice.location - start_slice.location).normalize();
//         //             let location_rot = -angle_tau_2d(&car.forward, &location_dir);
//         //             car_location += rotate_2d(&location_dir, &location_rot) * part_distance as f32;
//         //         }
//         //     }

//         //     car_speed = final_slice.velocity;
//         //     distance += part_distance;
//         //     time += final_slice.time - start_slice.time;
//         // }

//         // if !done {
//         let turn_lut = if car_speed <= 1234 {
//             turn_accel_lut
//         } else {
//             turn_decel_lut
//         };

//         let start_velocity_index = binary_search_turn_lut_velocity(turn_lut, car_speed);
//         let start_slice = turn_lut.velocity_sorted[start_velocity_index];

//         let final_time_index = linear_search_turn_lut_target(turn_lut, required_yaw_change, start_slice.time_sorted_index);
//         let final_slice = turn_lut.time_sorted[final_time_index];

//         let part_distance = final_slice.distance - start_slice.distance;

//         match inverted {
//             false => {
//                 let rotation = final_slice.yaw - start_slice.yaw;
//                 car_yaw += rotation;
//                 if car_yaw > PI {
//                     car_yaw -= TAU;
//                 }

//                 car_forward = rotate_2d(&car_forward, &rotation);

//                 car_location += (final_slice.location - start_slice.location).scale(part_distance as f32);
//             }
//             true => {
//                 let rotation = -1. * (final_slice.yaw - start_slice.yaw);

//                 car_yaw += rotation;
//                 if car_yaw > PI {
//                     car_yaw -= TAU;
//                 }

//                 car_forward = rotate_2d(&car_forward, &rotation);

//                 let location_dir = (final_slice.location - start_slice.location).normalize();
//                 let location_rot = -angle_tau_2d(&car.forward, &location_dir);
//                 car_location += rotate_2d(&location_dir, &location_rot) * part_distance as f32;
//             }
//         }

//         car_speed = final_slice.velocity;
//         distance += part_distance;
//         time += final_slice.time - start_slice.time;
//         // }

//         Self {
//             car_location,
//             car_forward,
//             car_speed,
//             car_yaw,
//             distance,
//             time,
//         }
//     }
// }
