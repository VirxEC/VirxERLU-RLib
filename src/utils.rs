use std::{
    cmp::Ordering,
    f32::consts::{FRAC_PI_2, PI, TAU},
    io::{Cursor, ErrorKind},
};

use byteorder::{BigEndian, ReadBytesExt};
use cpython::{exc, ObjectProtocol, PyDict, PyErr, PyObject, PyResult, Python};
use rl_ball_sym::{linear_algebra::vector::Vec3, simulation::ball::Ball};

pub const MAX_SPEED: f32 = 2300.;
pub const MAX_SPEED_NO_BOOST: f32 = 1410.;
pub const MIN_SPEED: f32 = -MAX_SPEED_NO_BOOST;
pub const MAX_TURN_RADIUS: f32 = 1. / 0.00076;
pub const TPS: f32 = 120.;
pub const SIMULATION_DT: f32 = 1. / TPS;
pub const BOOST_CONSUMPTION: f32 = 33.3 + 1. / 33.;
pub const BRAKE_ACC: f32 = 3500.;
pub const COAST_ACC: f32 = 525.;
pub const MIN_BOOST_TIME: f32 = 0.1;
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

pub struct TurnLut {
    pub time_sorted: Vec<TurnLutEntry>,
    pub velocity_sorted: Vec<TurnLutEntry>,
}

impl TurnLut {
    pub fn from(mut entries: Vec<TurnLutEntry>) -> Self {
        entries.sort_unstable_by(|a, b| a.time.partial_cmp(&b.time).unwrap_or(Ordering::Equal));

        for (time_index, entry) in entries.iter_mut().enumerate() {
            entry.time_sorted_index = time_index;
        }

        let time_sorted = entries.clone();

        entries.dedup_by_key(|e| e.velocity);
        entries.sort_by(|a, b| a.velocity.partial_cmp(&b.velocity).unwrap_or(Ordering::Equal));
        entries.dedup_by_key(|e| e.velocity);

        Self {
            time_sorted,
            velocity_sorted: entries,
        }
    }
}

#[derive(Clone, Copy)]
pub struct TurnLutEntry {
    pub time: f32,
    pub velocity: i16,
    pub distance: u16,
    pub location: Vec3,
    pub yaw: f32,
    pub time_sorted_index: usize,
}

pub fn read_turn_bin(bin_data: Vec<u8>) -> Vec<TurnLutEntry> {
    let mut cursor = Cursor::new(bin_data);
    let mut lut: Vec<TurnLutEntry> = Vec::new();

    loop {
        let time = match cursor.read_u16::<BigEndian>() {
            Ok(num) => num as f32 / 7222.,
            Err(error) => match error.kind() {
                ErrorKind::UnexpectedEof => break,
                other_error => {
                    panic!("Problem parsing file: {:?}", other_error)
                }
            },
        };

        let location = Vec3 {
            x: match cursor.read_i16::<BigEndian>() {
                Ok(num) => num as f32 / 6.,
                Err(error) => panic!("Problem parsing file: {:?}", error),
            },
            y: match cursor.read_i16::<BigEndian>() {
                Ok(num) => num as f32 / 6.,
                Err(error) => panic!("Problem parsing file: {:?}", error),
            },
            z: 0.,
        };

        let velocity = match cursor.read_i16::<BigEndian>() {
            Ok(num) => num,
            Err(error) => panic!("Problem parsing file: {:?}", error),
        };

        let yaw = match cursor.read_i16::<BigEndian>() {
            Ok(num) => num as f32 / 10200.,
            Err(error) => panic!("Problem parsing file: {:?}", error),
        };

        let distance = match cursor.read_u16::<BigEndian>() {
            Ok(num) => num,
            Err(error) => panic!("Problem parsing file: {:?}", error),
        };

        lut.push(TurnLutEntry {
            time,
            velocity,
            distance,
            location,
            yaw,
            time_sorted_index: 0,
        });
    }

    lut
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

pub fn dot_2d(vec1: &Vec3, vec2: &Vec3) -> f32 {
    vec1.x * vec2.x + vec1.y * vec2.y
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

pub fn angle_2d(vec1: &Vec3, vec2: &Vec3) -> f32 {
    dot_2d(vec1, vec2).clamp(-1., 1.).acos()
}

pub fn angle_tau_2d(vec1: &Vec3, vec2: &Vec3) -> f32 {
    let angle = vec2.y.atan2(vec2.x) - vec1.y.atan2(vec1.x);

    if angle < 0. {
        return angle + 2. * PI;
    }

    angle
}

// pub fn angle_tau_2d_cache(vec1_atan2: &f32, vec2: &Vec3) -> f32 {
//     let angle = vec2.y.atan2(vec2.x) - vec1_atan2;

//     if angle < 0. {
//         return angle + 2. * PI;
//     }

//     angle
// }

// pub fn dist(vec1: &Vec3, vec2: &Vec3) -> f32 {
//     (*vec2 - *vec1).magnitude()
// }

pub fn dist_2d(vec1: &Vec3, vec2: &Vec3) -> f32 {
    incomplete_dist_2d(vec1, vec2).sqrt()
}

pub fn incomplete_dist_2d(vec1: &Vec3, vec2: &Vec3) -> f32 {
    let vec = *vec2 - *vec1;
    dot_2d(&vec, &vec)
}

pub fn rotate_2d(vec: &Vec3, angle: &f32) -> Vec3 {
    Vec3 {
        x: angle.cos() * vec.x - angle.sin() * vec.y,
        y: angle.sin() * vec.x + angle.cos() * vec.y,
        z: vec.z,
    }
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

pub fn globalize(car: &Car, vec: &Vec3) -> Vec3 {
    car.forward * vec.x + car.right * vec.y + car.up * vec.z + car.location
}

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

pub fn binary_search_turn_lut_velocity(lut: &TurnLut, velocity: i16) -> usize {
    let mut left = 0;
    let mut right = lut.velocity_sorted.len() - 1;

    while right - left > 0 {
        let mid = (left + right) / 2;
        let slice = &lut.velocity_sorted[mid];

        let cmp = slice.velocity.cmp(&velocity);

        // The reason why we use if/else control flow rather than match
        // is because match reorders comparison operations, which is perf sensitive.
        // This is x86 asm for u8: https://rust.godbolt.org/z/8Y8Pra.
        if cmp == Ordering::Less {
            left = mid + 1;
        } else if cmp == Ordering::Greater {
            right = mid;
        } else {
            return mid;
        }
    }

    left
}

pub fn binary_search_turn_lut_time(lut: &TurnLut, time: &f32) -> usize {
    let mut left = 0;
    let mut right = lut.time_sorted.len();

    while right - left > 0 {
        let mid = (left + right) / 2;
        let slice = &lut.time_sorted[mid];

        let cmp = slice.time.partial_cmp(&time).unwrap_or(Ordering::Equal);

        // The reason why we use if/else control flow rather than match
        // is because match reorders comparison operations, which is perf sensitive.
        // This is x86 asm for u8: https://rust.godbolt.org/z/8Y8Pra.
        if cmp == Ordering::Less {
            left = mid + 1;
        } else if cmp == Ordering::Greater {
            right = mid;
        } else {
            return mid;
        }
    }

    left
}

pub fn linear_search_turn_lut_target(lut: &TurnLut, required_yaw_change: f32, start_index: usize) -> usize {
    let mut index = start_index;
    let mut current_yaw = 0.;

    let start_yaw = lut.time_sorted[index].yaw;

    let mut next_index = index + 3;
    let mut next_yaw = lut.time_sorted[next_index].yaw - start_yaw;

    // in theory, we should only ever be turning right at most 180 degrees, so this should be fine
    while (required_yaw_change - next_yaw).abs() <= (required_yaw_change - current_yaw).abs() {
        index = next_index;
        current_yaw = next_yaw;

        next_index = index + 3;
        next_yaw = lut.time_sorted[next_index].yaw - start_yaw;
    }

    index
}

// fn is_point_in_field(car: &Car, target: Vec3) -> bool {
//     target.y.abs() < 5120. - car.hitbox.length && target.x.abs() < 4093. - car.hitbox.length
// }

pub fn is_circle_in_field(car: &Car, target: Vec3, circle_radius: f32) -> bool {
    target.y.abs() < 5120. - car.hitbox.length - circle_radius && target.x.abs() < 4093. - car.hitbox.length - circle_radius
}

pub fn analyze_target(ball: Box<Ball>, car: &Car, shot_vector: Vec3, turn_accel_lut: &TurnLut, turn_accel_boost_lut: &TurnLut, turn_decel_lut: &TurnLut, validate: bool) -> (bool, f32, Vec3, bool, TurnInfo) {
    let offset_target = ball.location - (shot_vector * ball.radius);
    let car_front_length = car.hitbox_offset.x + car.hitbox.length / 2.;

    let mut turn_info = TurnInfo::default();
    let mut turn = false;
    let mut distance_remaining = -car_front_length;

    let exit_turn_point = offset_target - (flattened(&shot_vector) * 640.);
    let local_offset = localize_2d(car, offset_target);

    if (exit_turn_point - car.location).dot(&car.forward) < car_front_length && local_offset.x > 0. && local_offset.y.abs() < 150. {
        distance_remaining += dist_2d(&car.location, &offset_target);
        return (true, distance_remaining, offset_target, turn, turn_info);
    }

    distance_remaining += dist_2d(&exit_turn_point, &offset_target);

    let inv_shot_vector = -shot_vector;

    let mut inv_shot_vector_perp = rotate_2d(&inv_shot_vector, &FRAC_PI_2);
    let mut side = false;

    let inv_shot_vector_perp_2 = rotate_2d(&inv_shot_vector, &-FRAC_PI_2);

    if incomplete_dist_2d(&(inv_shot_vector_perp_2 + exit_turn_point), &car.location) < incomplete_dist_2d(&(inv_shot_vector_perp + exit_turn_point), &car.location) {
        inv_shot_vector_perp = inv_shot_vector_perp_2;
        side = true;
    }

    let circle_center = exit_turn_point + inv_shot_vector_perp * MAX_TURN_RADIUS;

    if validate && !is_circle_in_field(car, circle_center, MAX_TURN_RADIUS) {
        // shot isn't in the field, so it's not valid
        // we could just return placeholder info
        // but these technically ARE the default values
        return (false, distance_remaining, offset_target, turn, turn_info);
    }

    let mut cc_to_cl = car.location - circle_center;

    let tangent_angle = (MAX_TURN_RADIUS / cc_to_cl.magnitude()).clamp(-1., 1.).acos();
    cc_to_cl.normalized();

    let tangent_lines = [rotate_2d(&cc_to_cl, &-tangent_angle), rotate_2d(&cc_to_cl, &tangent_angle)];

    let cc_to_etp = (exit_turn_point - circle_center).normalize();
    let enter_turn_line;

    if side {
        enter_turn_line = *tangent_lines.iter().max_by(|a, b| angle_tau_2d(&cc_to_etp, a).partial_cmp(&angle_tau_2d(&cc_to_etp, b)).unwrap_or(Ordering::Equal)).unwrap();
    } else {
        enter_turn_line = *tangent_lines.iter().min_by(|a, b| angle_tau_2d(&cc_to_etp, a).partial_cmp(&angle_tau_2d(&cc_to_etp, b)).unwrap_or(Ordering::Equal)).unwrap();
    }

    let enter_turn_point = enter_turn_line * MAX_TURN_RADIUS + circle_center;

    let turn_angle = angle_tau_2d(&enter_turn_line, &cc_to_etp);
    let turn_distance_remaining = turn_angle * MAX_TURN_RADIUS;

    distance_remaining += turn_distance_remaining;

    if dist_2d(&car.location, &enter_turn_point) > car.hitbox.width / 2. {
        distance_remaining += turn_info.distance as f32 + dist_2d(&turn_info.car_location, &enter_turn_point);

        turn_info = TurnInfo::calc_turn_info(car, &enter_turn_point, turn_accel_lut, turn_accel_boost_lut, turn_decel_lut);
        turn = true;

        return (true, distance_remaining, enter_turn_point, turn, turn_info);
    }

    (true, distance_remaining, exit_turn_point, turn, turn_info)
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

const BOOST_ACCEL: f32 = 991. + 2. / 3.;
const BOOST_ACCEL_DT: f32 = BOOST_ACCEL * SIMULATION_DT;

pub fn can_reach_target(car: &Car, max_time: f32, distance_remaining: f32, is_forwards: bool) -> (bool, f32) {
    let mut d = distance_remaining;
    let mut t_r = max_time;
    let mut b = car.boost as f32;
    let mut v = car.velocity.dot(&car.forward);

    let direction = is_forwards as u8 as f32;

    loop {
        if d <= 0. {
            return (true, t_r);
        }

        let r = d * direction / t_r;

        if MIN_SPEED > r || r > MAX_SPEED || t_r < -SIMULATION_DT {
            return (false, -1.);
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
                boost = true
            }
        }

        if throttle.signum() == v.signum() {
            v += throttle_accel * throttle;
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

    (true, t_r)
}

pub struct TurnInfo {
    pub car_location: Vec3,
    pub car_forward: Vec3,
    pub car_speed: i16,
    pub car_yaw: f32,
    pub distance: u16,
    pub time: f32,
    pub boost: u8,
}

impl Default for TurnInfo {
    fn default() -> Self {
        Self {
            car_location: Vec3::default(),
            car_forward: Vec3::default(),
            car_speed: 0,
            car_yaw: 0.,
            distance: 0,
            time: 0.,
            boost: 0,
        }
    }
}

impl TurnInfo {
    pub fn calc_turn_info(car: &Car, target: &Vec3, turn_accel_lut: &TurnLut, turn_accel_boost_lut: &TurnLut, turn_decel_lut: &TurnLut) -> Self {
        let mut target = *target;
        let mut local_target = localize_2d(car, target);
        let inverted = local_target.y < 0.;

        if inverted {
            local_target.y = -local_target.y;
            target = globalize(car, &local_target)
        }

        let required_yaw_change = angle_2d(&car.forward, &(target - car.location).normalize());

        let mut car_speed = car.velocity.dot(&car.forward).round() as i16;

        let mut car_location = car.location;
        let mut car_forward = car.forward;
        let mut car_yaw = car.yaw;
        let mut distance = 0;
        let mut time = 0.;
        let mut boost = car.boost;

        let mut done = false;

        if boost >= 4 {
            let start_velocity_index = binary_search_turn_lut_velocity(turn_accel_boost_lut, car_speed);
            let start_slice = &turn_accel_boost_lut.velocity_sorted[start_velocity_index];

            let final_time_index = linear_search_turn_lut_target(turn_accel_boost_lut, required_yaw_change, start_slice.time_sorted_index);
            let mut final_slice = &turn_accel_boost_lut.time_sorted[final_time_index];

            {
                let max_boost_time = boost as f32 / BOOST_CONSUMPTION;
                let turn_time = final_slice.time - start_slice.time;

                if max_boost_time < turn_time {
                    let final_time_index = binary_search_turn_lut_time(turn_accel_boost_lut, &max_boost_time);
                    final_slice = &turn_accel_boost_lut.time_sorted[final_time_index];
                    boost = 0;
                } else {
                    done = true;
                    boost -= (turn_time * BOOST_CONSUMPTION) as u8;
                }
            }

            let part_distance = final_slice.distance - start_slice.distance;

            match inverted {
                false => {
                    let rotation = final_slice.yaw - start_slice.yaw;
                    car_yaw += rotation;
                    if car_yaw > PI {
                        car_yaw -= TAU;
                    }

                    car_forward = rotate_2d(&car_forward, &rotation);

                    car_location += (final_slice.location - start_slice.location).scale(part_distance as f32);
                }
                true => {
                    let rotation = -1. * (final_slice.yaw - start_slice.yaw);

                    car_yaw += rotation;
                    if car_yaw > PI {
                        car_yaw -= TAU;
                    }

                    car_forward = rotate_2d(&car_forward, &rotation);

                    let location_dir = (final_slice.location - start_slice.location).normalize();
                    let location_rot = -angle_tau_2d(&car.forward, &location_dir);
                    car_location += rotate_2d(&location_dir, &location_rot) * part_distance as f32;
                }
            }

            car_speed = final_slice.velocity;
            distance += part_distance;
            time += final_slice.time - start_slice.time;
        }

        if !done {
            let turn_lut = if car_speed <= 1234 {
                turn_accel_lut
            } else {
                turn_decel_lut
            };

            let start_velocity_index = binary_search_turn_lut_velocity(turn_lut, car_speed);
            let start_slice = turn_lut.velocity_sorted[start_velocity_index];

            let final_time_index = linear_search_turn_lut_target(turn_lut, required_yaw_change, start_slice.time_sorted_index);
            let final_slice = turn_lut.time_sorted[final_time_index];

            let part_distance = final_slice.distance - start_slice.distance;

            match inverted {
                false => {
                    let rotation = final_slice.yaw - start_slice.yaw;
                    car_yaw += rotation;
                    if car_yaw > PI {
                        car_yaw -= TAU;
                    }

                    car_forward = rotate_2d(&car_forward, &rotation);

                    car_location += (final_slice.location - start_slice.location).scale(part_distance as f32);
                }
                true => {
                    let rotation = -1. * (final_slice.yaw - start_slice.yaw);

                    car_yaw += rotation;
                    if car_yaw > PI {
                        car_yaw -= TAU;
                    }

                    car_forward = rotate_2d(&car_forward, &rotation);

                    let location_dir = (final_slice.location - start_slice.location).normalize();
                    let location_rot = -angle_tau_2d(&car.forward, &location_dir);
                    car_location += rotate_2d(&location_dir, &location_rot) * part_distance as f32;
                }
            }

            car_speed = final_slice.velocity;
            distance += part_distance;
            time += final_slice.time - start_slice.time;
        }

        Self {
            car_location,
            car_forward,
            car_speed,
            car_yaw,
            distance,
            time,
            boost,
        }
    }
}
