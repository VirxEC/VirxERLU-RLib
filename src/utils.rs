use cpython::{exc, ObjectProtocol, PyDict, PyErr, PyObject, PyResult, Python};
use dubins_paths::{path_sample, shortest_path, DubinsError, DubinsPath};
use rl_ball_sym::simulation::ball::Ball;

use vvec3::Vec3;

pub const MAX_SPEED: f32 = 2300.;
pub const MAX_SPEED_NO_BOOST: f32 = 1410.;
pub const MIN_SPEED: f32 = -MAX_SPEED_NO_BOOST;
// pub const MAX_SPEED_MIN_TURN_RADIUS: f32 = 1. / 0.00088;
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

#[derive(Clone)]
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

#[derive(Clone)]
pub struct Car {
    pub location: Vec3,
    pub velocity: Vec3,
    pub local_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub forward: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub hitbox: Hitbox,
    pub hitbox_offset: Vec3,
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
    /// min turn radius at max_speed
    pub msmtr: f32,
}

impl Default for Car {
    fn default() -> Self {
        Self {
            location: Vec3::default(),
            velocity: Vec3::default(),
            local_velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            forward: Vec3::default(),
            right: Vec3::default(),
            up: Vec3::default(),
            hitbox: Hitbox::default(),
            hitbox_offset: Vec3::default(),
            field: CarFieldRect::default(),
            pitch: 0.,
            yaw: 0.,
            roll: 0.,
            boost: 0,
            demolished: false,
            airborne: false,
            jumped: false,
            doublejumped: false,
            max_speed: 0.,
            msmtr: 0.,
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

        // CP * CY
        // CP * SY
        // SP
        self.forward.x = c_p * c_y;
        self.forward.y = c_p * s_y;
        self.forward.z = s_p;

        // CY * SP * SR - CR * SY
        // SY * SP * SR + CR * CY
        // -CP * SR
        self.right.x = c_y * s_p * s_r - c_r * s_y;
        self.right.y = s_y * s_p * s_r + c_r * c_y;
        self.right.z = -c_p * s_r;

        // -CR * CY * SP - SR * SY
        // -CR * SY * SP + SR * CY
        // CP * CR
        self.up.x = -c_r * c_y * s_p - s_r * s_y;
        self.up.y = -c_r * s_y * s_p + s_r * c_y;
        self.up.z = c_p * c_r;
    }

    pub fn calculate_max_values(&mut self) {
        self.max_speed = self.get_max_speed();
        self.msmtr = turn_radius(self.max_speed);
    }

    pub fn calculate_local_values(&mut self) {
        self.local_velocity = localize(self, self.velocity);
    }

    pub fn calculate_field(&mut self) {
        self.field = CarFieldRect::from(&self.hitbox);
    }

    fn get_max_speed(&mut self) -> f32 {
        let mut b = self.boost as f32;
        let mut v = self.velocity.dot(&self.forward);

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

pub fn get_usize_from_dict(py: Python, py_dict: &PyDict, key: &str, name: &str) -> PyResult<usize> {
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

pub fn localize_2d_location(car: &Car, vec: Vec3) -> Vec3 {
    localize_2d(car, vec - car.location)
}

pub fn localize_2d(car: &Car, vec: Vec3) -> Vec3 {
    Vec3 {
        x: vec.dot(&car.forward),
        y: vec.dot(&car.right),
        z: 0.,
    }
}

// pub fn localize_location(car: &Car, vec: Vec3) -> Vec3 {
//     localize(car, vec - car.location)
// }

pub fn localize(car: &Car, vec: Vec3) -> Vec3 {
    Vec3 {
        x: vec.dot(&car.forward),
        y: vec.dot(&car.right),
        z: vec.dot(&car.up),
    }
}

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

    panic!("Invalid input velocity");
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

#[derive(Clone)]
pub struct CarFieldRect {
    goal_x: f32,
    goal_y: f32,
    field_y: f32,
    field_x: f32,
}

impl Default for CarFieldRect {
    fn default() -> Self {
        Self {
            goal_x: 0.,
            goal_y: 0.,
            field_y: 0.,
            field_x: 0.,
        }
    }
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

    pub fn are_points_in(&self, s: &[[f32; 3]]) -> bool {
        for p in s {
            if !self.is_point_in(p) {
                return false;
            }
        }

        return true;
    }

    pub fn is_point_in(&self, p: &[f32; 3]) -> bool {
        if p[0].abs() > self.goal_x {
            return p[0].abs() < self.field_x && p[1].abs() < self.field_y;
        }

        return p[1].abs() < self.goal_y;
    }
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

pub fn analyze_target(ball: &Box<Ball>, car: &Car, shot_vector: Vec3, time_remaining: f32, get_target: bool, validate: bool) -> Result<([f32; 4], Option<Vec3>, Option<DubinsPath>), DubinsError> {
    let offset_target = ball.location - (shot_vector * ball.radius);
    let car_front_length = (car.hitbox_offset.x + car.hitbox.length) / 2.;

    let mut end_distance = -car_front_length;

    let offset_distance = 640.;
    let exit_turn_point = offset_target - (shot_vector * offset_distance);

    let local_offset = localize_2d_location(car, offset_target);

    if local_offset.x > 0. {
        if local_offset.y.abs() < car.hitbox.width / 2. && car.forward.angle_2d(&shot_vector) < 0.15 {
            end_distance += local_offset.x;

            let final_target = if get_target {
                Some(car.location + shot_vector * local_offset.x)
            } else {
                None
            };
            return Ok(([0., 0., 0., end_distance], final_target, None));
        } else {
            let angle_to_shot = (ball.location - car.location).normalize().angle_2d(&shot_vector);

            if angle_to_shot < 0.75 {
                if car.forward.dot(&(exit_turn_point - car.location)) > 0. {
                    let turn_rad = radius_from_local_point(localize_2d_location(car, exit_turn_point));

                    // TODO
                    // turn radius with direction
                    if turn_radius(car.local_velocity.x.abs()) < turn_rad && turn_rad < car.msmtr * 1.3 {
                        end_distance += offset_distance + turn_rad * angle_to_shot;

                        let final_target = if get_target {
                            Some(exit_turn_point)
                        } else {
                            None
                        };

                        return Ok(([0., 0., 0., end_distance], final_target, None));
                    }
                } else if angle_to_shot < 0.25 && car.forward.angle_2d(&shot_vector) < 0.5 {
                    end_distance += offset_distance + radius_from_local_point(local_offset) * angle_to_shot;

                    let final_target = if get_target {
                        Some(offset_target)
                    } else {
                        None
                    };

                    return Ok(([0., 0., 0., end_distance], final_target, None));
                }
            }
        }
    }

    end_distance += offset_distance;

    if validate && (end_distance + car.location.dist_2d(exit_turn_point)) / time_remaining > car.max_speed {
        return Err(DubinsError::NoPath);
    }

    let q0 = [car.location.x, car.location.y, car.yaw];
    let q1 = [exit_turn_point.x, exit_turn_point.y, shot_vector.y.atan2(shot_vector.x)];

    let path = shortest_path(q0, q1, car.msmtr * 1.05)?;
    let end_parts = path_section_endpoints(&path)?;

    if validate {
        let midpoints = path_section_midpoints(&path)?;
        let all_points = [midpoints[0], end_parts[0], end_parts[1], midpoints[2], end_parts[2]];

        if !car.field.are_points_in(&all_points) {
            return Err(DubinsError::NoPath);
        }
    }

    let distances = [path.param[0] * path.rho, path.param[1] * path.rho, path.param[2] * path.rho];

    if !get_target {
        return Ok(([distances[0], distances[1], distances[2], end_distance], None, Some(path)));
    }

    let final_target = Some(path_endpoint_to_vec3(
        end_parts[{
            if distances[0] + distances[1] < car_front_length {
                2
            } else if distances[0] < car_front_length {
                1
            } else {
                0
            }
        }],
    ));
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
    let mut v = car.local_velocity.x;

    let direction = is_forwards as u8 as f32;

    loop {
        if d <= 0. {
            return Ok(t_r);
        }

        let r = d * direction / t_r;

        if t_r < -SIMULATION_DT || (is_forwards && r > car.max_speed) || (!is_forwards && MIN_SPEED > r) {
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
            v += BRAKE_ACC_DT.copysign(throttle);
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
