use std::{cmp::Ordering, f32::consts::{FRAC_PI_2,FRAC_PI_4, PI, TAU}, io::{Cursor, ErrorKind}};

use byteorder::{BigEndian, ReadBytesExt};
use cpython::{exc, ObjectProtocol, PyDict, PyErr, PyObject, PyResult, Python};
use rl_ball_sym::linear_algebra::vector::Vec3;

// pub static NO_ADJUST_RADIANS_12K: f32 = 0.001; // equates to about 12 units at a distance of 12k units
pub static NO_ADJUST_RADIANS: f32 = 0.025;
pub static MIN_ADJUST_RADIANS: f32 = FRAC_PI_4; // right angle (pi/4*2)

// pub static VMAX: f32 = 1234.;
// pub static TAU: f32 = 0.74704;

pub static MAX_TURN_RADIUS: f32 = 1. / 0.00076;
pub static MAX_SPEED: f32 = 2300.;
pub static BOOST_CONSUMPTION: f32 = 33. + 1. / 3.;

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
    pub velocity: f32,
    pub distance: f32,
    pub location: Vec3,
    pub forward: Vec3,
    pub right: Vec3,
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
            Ok(num) => num as f32 / 13.,
            Err(error) => panic!("Problem parsing file: {:?}", error),
        };

        let forward = Vec3 {
            x: match cursor.read_i16::<BigEndian>() {
                Ok(num) => num as f32 / 32000.,
                Err(error) => panic!("Problem parsing file: {:?}", error),
            },
            y: match cursor.read_i16::<BigEndian>() {
                Ok(num) => num as f32 / 32000.,
                Err(error) => panic!("Problem parsing file: {:?}", error),
            },
            z: 0.,
        };

        let distance = match cursor.read_u16::<BigEndian>() {
            Ok(num) => num as f32 / 3.,
            Err(error) => panic!("Problem parsing file: {:?}", error),
        };

        lut.push(TurnLutEntry {
            time,
            velocity,
            distance,
            location,
            forward,
            right: rotate_2d(&forward, &FRAC_PI_2),
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

pub fn localize(car: &Car, vec: Vec3) -> Vec3  {
    let vec = vec - car.location;

    Vec3 {
        x: vec.dot(&car.forward),
        y: vec.dot(&car.right),
        z: vec.dot(&car.up),
    }
}

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

pub fn binary_search_turn_lut_velocity(lut: &TurnLut, velocity: &f32) -> usize {
    let mut left = 0;
    let mut right = lut.velocity_sorted.len() - 1;

    while right - left > 0 {
        let mid = (left + right) / 2;
        let slice = &lut.velocity_sorted[mid];

        let cmp = slice.velocity.partial_cmp(&velocity).unwrap_or(Ordering::Equal);

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

pub fn linear_search_turn_lut_target(lut: &TurnLut, car_location: &Vec3, car_forward: &Vec3, target_location: &Vec3, start_index: usize) -> usize {
    let mut index = start_index;

    let target;
    {
        let target_local = *target_location - *car_location;
        let target_rotate = angle_tau_2d(&car_forward, &lut.time_sorted[index].forward);
        target = rotate_2d(&target_local, &target_rotate) + lut.time_sorted[index].location;
    }

    let mut angle;
    let mut is_forward;

    {
        let target_local = target - lut.time_sorted[index].location;
        angle = lut.time_sorted[index].right.dot(&target_local).abs();
        is_forward = lut.time_sorted[index].forward.dot(&target_local) > 0.;
    }

    let mut next_index = index + 2;
    let mut next_angle;
    let mut next_is_forward;

    {
        let target_local = target - lut.time_sorted[next_index].location;
        next_angle = lut.time_sorted[next_index].right.dot(&target_local).abs();
        next_is_forward = lut.time_sorted[next_index].forward.dot(&target_local) > 0.;
    }
    
    while !next_is_forward || next_angle <= angle || !is_forward {
        // println!("index: {} | angle: {} | is_forward: {} | next_angle: {} | next_is_forward: {}", index, angle, is_forward, next_angle, next_is_forward);
        
        index = next_index;
        angle = next_angle;
        is_forward = next_is_forward;

        if is_forward {
            next_index += match angle as usize / 100 {
                i if i >= 2 => i,
                _ => 2,
            };
        } else {
            next_index += match angle as usize / 20 {
                i if i >= 2 => i,
                _ => 2,
            };
        }

        let target_local = target - lut.time_sorted[next_index].location;
        next_angle = lut.time_sorted[next_index].right.dot(&target_local).abs();
        next_is_forward = lut.time_sorted[next_index].forward.dot(&target_local) > 0.;
    }

    // println!("index: {} | angle: {} | is_forward: {} | next_angle: {} | next_is_forward: {}", index, angle, is_forward, next_angle, next_is_forward);

    index
}

// fn is_point_in_field(car: &Car, target: Vec3) -> bool {
//     target.y.abs() < 5120. - car.hitbox.length && target.x.abs() < 4093. - car.hitbox.length
// }

pub fn is_circle_in_field(car: &Car, target: Vec3, circle_radius: f32) -> bool {
    target.y.abs() < 5120. - car.hitbox.length - circle_radius && target.x.abs() < 4093. - car.hitbox.length - circle_radius
}

pub struct TurnInfo {
    pub car_location: Vec3,
    pub car_speed: f32,
    pub distance: f32,
    pub time: f32,
    pub boost: u8,
}

impl Default for TurnInfo {
    fn default() -> Self {
        Self {
            car_location: Vec3::default(),
            car_speed: 0.,
            distance: 0.,
            time: 0.,
            boost: 0,
        }
    }
}

impl TurnInfo {
    pub fn calc_turn_info(car: &Car, target: &Vec3, turn_accel_lut: &TurnLut, turn_accel_boost_lut: &TurnLut, turn_decel_lut: &TurnLut) -> Self {
        // println!("");
        let mut target = *target;
        let mut local_target = localize(car, target);
        let inverted = local_target.y < 0.;
        // println!("inverted: {} | target: {:?}", inverted, target);

        if inverted {
            local_target.y = -local_target.y;
            target = globalize(car, &local_target)
        }

        let mut car_speed = car.velocity.dot(&car.forward);

        let mut car_location = car.location;
        let mut car_forward = car.forward;
        let mut distance = 0.;
        let mut time = 0.;
        let mut boost = car.boost;

        let mut done = false;

        if boost >= 4 {
            let start_velocity_index = binary_search_turn_lut_velocity(turn_accel_boost_lut, &car_speed);
            let start_slice = &turn_accel_boost_lut.velocity_sorted[start_velocity_index];

            // println!("Starting values | index: {} | speed: {} | location: {:?} | forward: {:?} | distance: {} | time: {} | boost: {}", start_slice.time_sorted_index, car_speed, car_location, car_forward, distance, time, boost);

            let final_time_index = linear_search_turn_lut_target(turn_accel_boost_lut, &car_location, &car_forward, &target, start_slice.time_sorted_index);
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

            // println!("Starting slice data | index: {} | speed: {} | location: {:?} | forward: {:?} | distance: {} | time: {}", start_slice.time_sorted_index, start_slice.velocity, start_slice.location, start_slice.forward, start_slice.distance, start_slice.time);
            // println!("Final slice data | index: {} | speed: {} | location: {:?} | forward: {:?} | distance: {} | time: {}", final_time_index, final_slice.velocity, final_slice.location, final_slice.forward, final_slice.distance, final_slice.time);

            match inverted {
                false => {
                    car_location += (final_slice.location - start_slice.location).scale(part_distance);
                    car_forward = rotate_2d(&car_forward, &angle_tau_2d(&start_slice.forward, &final_slice.forward));
                }
                true => {
                    let location_dir = final_slice.location - start_slice.location;
                    car_location += rotate_2d(&location_dir, &(TAU - &angle_tau_2d(&car_forward, &location_dir))).scale(part_distance);
                    
                    car_forward = rotate_2d(&car_forward, &(TAU - angle_tau_2d(&start_slice.forward, &final_slice.forward)));
                }
            }
                
            car_speed = final_slice.velocity;
            distance += part_distance;
            time += final_slice.time - start_slice.time;

            // println!("Final values | index: {} | speed: {} | location: {:?} | forward: {:?} | distance: {} | time: {} | boost: {}", final_time_index, car_speed, car_location, car_forward, distance, time, boost);
        }

        if !done {
            let turn_lut = if car_speed <= 1234. {
                turn_accel_lut
            } else {
                turn_decel_lut
            };

            let start_velocity_index = binary_search_turn_lut_velocity(turn_lut, &car_speed);
            let start_slice = turn_lut.velocity_sorted[start_velocity_index];

            let final_time_index = linear_search_turn_lut_target(turn_lut, &car_location, &car_forward, &target, start_slice.time_sorted_index);
            let final_slice = turn_lut.time_sorted[final_time_index];

            let part_distance = final_slice.distance - start_slice.distance;

            match inverted {
                false => {
                    car_location += (final_slice.location - start_slice.location).scale(part_distance);
                }
                true => {
                    let location_dir = final_slice.location - start_slice.location;
                    car_location += rotate_2d(&location_dir, &(TAU - &angle_tau_2d(&car_forward, &location_dir))).scale(part_distance);
                }
            }

            car_speed = final_slice.velocity;
            distance += part_distance;
            time += final_slice.time - start_slice.time;
        }

        Self {
            car_location,
            car_speed,
            distance,
            time,
            boost,
        }
    }
}
