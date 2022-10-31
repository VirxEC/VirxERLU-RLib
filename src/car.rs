use crate::{
    constants::*,
    pytypes::{GameCar, Hitbox},
    utils::{flatten, minimum_non_negative, vertex_quadratic_solve_for_x},
    BoostAmount, Mutators,
};
use dubins_paths::DubinsPath;
use glam::Vec3A;

pub fn throttle_acceleration(forward_velocity: f32) -> f32 {
    let x = forward_velocity.abs();

    if x >= MAX_SPEED_NO_BOOST {
        return 0.;
    }

    // use y = mx + b to find the throttle acceleration
    if x < THROTTLE_ACCEL_DIVISION {
        START_THROTTLE_ACCEL_M * x + START_THROTTLE_ACCEL_B
    } else {
        END_THROTTLE_ACCEL_M * (x - THROTTLE_ACCEL_DIVISION) + END_THROTTLE_ACCEL_B
    }
}

pub fn curvature(v: f32) -> f32 {
    let v = v.abs();

    if (0. ..500.).contains(&v) {
        0.0069 - 5.84e-6 * v
    } else if (500. ..1000.).contains(&v) {
        0.00561 - 3.26e-6 * v
    } else if (1000. ..1500.).contains(&v) {
        0.0043 - 1.95e-6 * v
    } else if (1500. ..1750.).contains(&v) {
        0.003_025 - 1.1e-6 * v
    } else if (1750. ..2305.).contains(&v) {
        0.0018 - 4e-7 * v
    } else {
        println!("Invalid input velocity: {v}");
        -1.
    }
}

#[inline]
pub fn turn_radius(v: f32) -> f32 {
    1. / curvature(v)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FieldRect {
    goal_x: f32,
    goal_y: f32,
    field_y: f32,
    field_x: f32,
}

impl FieldRect {
    const CHECK_DISTANCE: f32 = 400.;

    #[inline]
    pub const fn new() -> Self {
        Self {
            goal_x: 0.,
            goal_y: 0.,
            field_y: 0.,
            field_x: 0.,
        }
    }

    pub fn from(car_hitbox: &Hitbox) -> Self {
        let half_car_len = car_hitbox.length / 2.;

        Self {
            goal_x: 893. - car_hitbox.width,
            goal_y: 6000. - car_hitbox.length,
            field_y: 5120. - half_car_len,
            field_x: 4093. - half_car_len,
        }
    }

    pub fn is_path_in(&self, path: &DubinsPath) -> bool {
        let length = path.length();

        let mut dist = Self::CHECK_DISTANCE.min(length / 2.);

        while dist < length {
            if !self.is_point_in(path.sample(dist).pos) {
                return false;
            }

            dist += Self::CHECK_DISTANCE;
        }

        true
    }

    pub fn is_point_in(&self, p: Vec3A) -> bool {
        let p = [p.x.abs(), p.y.abs()];

        if p[0] > self.goal_x {
            p[0] < self.field_x && p[1] < self.field_y
        } else {
            p[1] < self.goal_y
        }
    }
}

#[derive(Clone, Debug, Default)]
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
    pub field: FieldRect,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub boost: u8,
    pub demolished: bool,
    pub airborne: bool,
    pub jumped: bool,
    pub doublejumped: bool,
    pub landing_time: f32,
    pub landing_location: Vec3A,
    pub landing_velocity: Vec3A,
    pub landing_yaw: f32,
    pub landing_forward: Vec3A,
    pub landing_right: Vec3A,
    pub landing_up: Vec3A,
    last_landing_game_time: f32,
    pub last_landing_time: f32,
    pub max_speed: Vec<f32>,
    /// turn radius at calculated max speed
    pub ctrms: Vec<f32>,
    pub max_jump_time: f32,
    pub max_jump_height: f32,
    pub max_double_jump_time: f32,
    pub max_double_jump_height: f32,
    pub init: bool,
}

impl Car {
    #[inline]
    pub const fn new() -> Self {
        Self {
            location: Vec3A::ZERO,
            velocity: Vec3A::ZERO,
            local_velocity: Vec3A::ZERO,
            angular_velocity: Vec3A::ZERO,
            forward: Vec3A::ZERO,
            right: Vec3A::ZERO,
            up: Vec3A::ZERO,
            hitbox: Hitbox::new(),
            hitbox_offset: Vec3A::ZERO,
            field: FieldRect::new(),
            pitch: 0.,
            yaw: 0.,
            roll: 0.,
            boost: 0,
            demolished: false,
            airborne: false,
            jumped: false,
            doublejumped: false,
            landing_time: 0.,
            landing_location: Vec3A::ZERO,
            landing_velocity: Vec3A::ZERO,
            landing_yaw: 0.,
            landing_forward: Vec3A::ZERO,
            landing_right: Vec3A::ZERO,
            landing_up: Vec3A::ZERO,
            last_landing_game_time: 0.,
            last_landing_time: 0.,
            max_speed: Vec::new(),
            ctrms: Vec::new(),
            max_jump_time: 0.,
            max_jump_height: 0.,
            max_double_jump_time: 0.,
            max_double_jump_height: 0.,
            init: false,
        }
    }

    pub fn update(&mut self, py_car: GameCar, game_time: f32) {
        self.location = py_car.physics.location.into();
        self.velocity = py_car.physics.velocity.into();
        self.angular_velocity = py_car.physics.angular_velocity.into();

        self.pitch = py_car.physics.rotation.pitch;
        self.yaw = py_car.physics.rotation.yaw;
        self.roll = py_car.physics.rotation.roll;

        self.hitbox = py_car.hitbox;
        self.hitbox_offset = py_car.hitbox_offset.into();

        self.boost = py_car.boost;
        self.demolished = py_car.is_demolished;
        self.jumped = py_car.jumped;
        self.doublejumped = py_car.double_jumped;

        let airborne = !py_car.has_wheel_contact;

        if self.airborne && !airborne {
            self.last_landing_game_time = game_time;
        }

        self.last_landing_time = self.last_landing_game_time - game_time;
        self.airborne = airborne;

        self.init = false;
    }

    pub fn init(&mut self, gravity: f32, max_ball_slice: usize, mutators: Mutators) {
        if !self.init {
            Self::calculate_orientation_matrix(&mut self.forward, &mut self.right, &mut self.up, self.pitch, self.yaw, self.roll);
            self.calculate_field();
            self.calculate_landing_info(gravity);
            self.calculate_local_values();
            self.calculate_max_values(max_ball_slice, mutators);
            self.calculate_max_jump_height(gravity);
            self.calculate_max_double_jump_height(gravity);

            self.init = true;
        }
    }

    pub fn calculate_max_jump_height(&mut self, gravity: f32) {
        let g = gravity * SIMULATION_DT;

        let mut t = 0.;
        let mut v_z = 0.;
        let mut l_z = 17.;

        while v_z > 0. || t < MAX_HOLD_TIME {
            if t <= f32::EPSILON {
                v_z += JUMP_SPEED;
            }

            if t < MAX_HOLD_TIME {
                v_z += HOLD_BONUS * SIMULATION_DT;
            }

            if t < STICKY_TIMER {
                v_z += STICKY_FORCE * SIMULATION_DT;
            }

            t += SIMULATION_DT;
            v_z += g;
            l_z += v_z * SIMULATION_DT;
        }

        self.max_jump_time = t;
        self.max_jump_height = l_z;
    }

    pub fn calculate_max_double_jump_height(&mut self, gravity: f32) {
        let g = gravity * SIMULATION_DT;

        let mut t = 0.;
        let mut v_z = self.landing_velocity.z;
        let mut l_z = self.landing_location.z;
        let mut double_jumped = false;

        while v_z > 0. || t < MAX_HOLD_TIME {
            if t <= f32::EPSILON {
                v_z += JUMP_SPEED;
            } else if t > MAX_HOLD_TIME + SIMULATION_DT && !double_jumped {
                v_z += JUMP_SPEED;
                double_jumped = true;
            }

            if t < MAX_HOLD_TIME {
                v_z += HOLD_BONUS * SIMULATION_DT;
            }

            if t < STICKY_TIMER {
                v_z += STICKY_FORCE * SIMULATION_DT;
            }

            v_z += g;
            l_z += v_z * SIMULATION_DT;
            t += SIMULATION_DT;
        }

        self.max_double_jump_time = t;
        self.max_double_jump_height = l_z;
    }

    pub fn calculate_landing_info(&mut self, gravity: f32) {
        self.landing_time = 0.;
        self.landing_location = self.location;
        self.landing_velocity = self.velocity;
        self.landing_yaw = self.yaw;
        self.landing_forward = self.forward;
        self.landing_right = self.right;
        self.landing_up = self.up;

        if !self.airborne {
            return;
        }

        // it's impossible to get true 0 gravity in RL
        // best you can get is a very small value
        // so we'll just ignore that edge case

        // this is the vertex of the equation, which also happens to be the apex of the trajectory
        let h = self.velocity.z / -gravity;
        let k = self.velocity.z * self.velocity.z / -gravity;

        let normal_gravity = gravity < 0.;
        let fall_distance = if normal_gravity { self.location.z - 17. } else { 2030. - self.location.z };

        let terminal_velocity = 2300. - flatten(self.velocity).length();
        let time_until_tv = terminal_velocity / -gravity;
        let distance_until_tv = self.velocity.z * time_until_tv + -gravity * (time_until_tv * time_until_tv) / 2.;

        if fall_distance <= distance_until_tv {
            let (time1, time2) = vertex_quadratic_solve_for_x(gravity, h, k, -fall_distance);
            self.landing_time = minimum_non_negative(time1, time2);
            self.landing_velocity.z += gravity * self.landing_time;
            self.landing_location += Vec3A::new(
                self.velocity.x * self.landing_time,
                self.velocity.y * self.landing_time,
                gravity * (self.landing_time - h).powi(2) + k,
            );
        } else {
            self.landing_time = time_until_tv;

            let dt = 1. / 120.;
            self.landing_velocity.z = terminal_velocity;
            self.landing_location += Vec3A::new(
                self.velocity.x * self.landing_time,
                self.velocity.y * self.landing_time,
                gravity * (self.landing_time - h).powi(2) + k,
            );
            let g_dt = Vec3A::new(0., 0., gravity * dt);

            loop {
                let v_t = (self.landing_velocity + g_dt).normalize_or_zero() * 2300.;
                let l_t = self.landing_location + self.landing_velocity * dt;

                let l_z = if normal_gravity { l_t[2] - 17. } else { 2030. - l_t[2] };

                if l_z <= 0. {
                    break;
                }

                self.landing_velocity = v_t;
                self.landing_location = l_t;
                self.landing_time += dt;
            }

            self.landing_location.z = if normal_gravity { 17. } else { 2300. };
        }

        if flatten(self.landing_velocity).length() != 0. {
            self.landing_yaw = self.landing_velocity.y.atan2(self.landing_velocity.x);
        }

        Self::calculate_orientation_matrix(&mut self.landing_forward, &mut self.landing_right, &mut self.landing_up, 0., self.landing_yaw, 0.);
    }

    pub fn calculate_orientation_matrix(forward: &mut Vec3A, right: &mut Vec3A, up: &mut Vec3A, pitch: f32, yaw: f32, roll: f32) {
        let (s_p, c_p) = pitch.sin_cos();
        let (s_y, c_y) = yaw.sin_cos();
        let (s_r, c_r) = roll.sin_cos();

        forward.x = c_p * c_y;
        forward.y = c_p * s_y;
        forward.z = s_p;

        right.x = c_y * s_p * s_r - c_r * s_y;
        right.y = s_y * s_p * s_r + c_r * c_y;
        right.z = -c_p * s_r;

        up.x = -c_r * c_y * s_p - s_r * s_y;
        up.y = -c_r * s_y * s_p + s_r * c_y;
        up.z = c_p * c_r;
    }

    pub fn calculate_max_values(&mut self, max_ball_slice: usize, mutators: Mutators) {
        let mut b = f32::from(self.boost);
        let mut v = self.landing_velocity.dot(self.forward);
        let mut fast_forward = false;

        self.max_speed = Vec::with_capacity(max_ball_slice);
        self.max_speed.push(v);

        self.ctrms = Vec::with_capacity(max_ball_slice);
        self.ctrms.push(turn_radius(v));

        let end_1 = (self.landing_time * 120.).round() as usize;

        for _ in 0..end_1 {
            self.max_speed.push(v);
            self.ctrms.push(turn_radius(v));
        }

        let mut end = end_1;

        if mutators.boost_amount != BoostAmount::NoBoost {
            for _ in end_1..max_ball_slice {
                end += 1;

                if fast_forward {
                    self.max_speed.push(v);
                    self.ctrms.push(turn_radius(v));
                    continue;
                }

                if b < BOOST_CONSUMPTION_DT {
                    break;
                }

                let throttle_accel = throttle_acceleration(v);
                let mut accel = 0.;

                if v.is_sign_positive() {
                    accel += throttle_accel * SIMULATION_DT;
                } else {
                    accel += BRAKE_ACC_DT;
                }

                if b > BOOST_CONSUMPTION_DT {
                    accel += mutators.boost_accel * SIMULATION_DT;
                    if mutators.boost_amount != BoostAmount::Unlimited {
                        b -= BOOST_CONSUMPTION_DT;
                    }
                }

                accel = accel.min(MAX_SPEED - v);

                if accel.abs() < f32::EPSILON {
                    fast_forward = true;
                }

                v += accel;

                self.max_speed.push(v);
                self.ctrms.push(turn_radius(v));
            }

            if end == max_ball_slice {
                return;
            }
        }

        let mut v1 = v;
        let mut v2 = v;

        for _ in end..max_ball_slice {
            if fast_forward {
                self.max_speed.push(v1);
                continue;
            }

            let throttle_accel = throttle_acceleration(v1);
            let mut accel = 0.;

            if (1. - v1.signum()).abs() < f32::EPSILON {
                accel += throttle_accel * SIMULATION_DT;
            } else {
                accel += BRAKE_ACC_DT;
            }

            accel = accel.min(MAX_SPEED - v1);

            if accel.abs() < f32::EPSILON {
                fast_forward = true;
            }

            v1 += accel;

            self.max_speed.push(v1);
        }

        for _ in end..max_ball_slice {
            if fast_forward {
                self.ctrms.push(turn_radius(v2));
                continue;
            }

            let throttle_accel = throttle_acceleration(v2);
            let mut accel = 0.;

            if (1. - v2.signum()).abs() < f32::EPSILON {
                accel += throttle_accel * SIMULATION_DT;
            } else {
                accel += BRAKE_ACC_DT;
            }

            accel = accel.min(MAX_SPEED - v2);

            if accel.abs() < f32::EPSILON {
                fast_forward = true;
            }

            v2 += accel;

            self.ctrms.push(turn_radius(v2));
        }
    }

    pub fn calculate_local_values(&mut self) {
        self.local_velocity = self.localize(self.velocity);
    }

    pub fn calculate_field(&mut self) {
        self.field = FieldRect::from(&self.hitbox);
    }

    #[inline]
    pub fn localize_2d_location(&self, vec: Vec3A) -> Vec3A {
        self.localize_2d(vec - self.landing_location)
    }

    #[inline]
    pub fn localize_2d(&self, vec: Vec3A) -> Vec3A {
        Vec3A::new(vec.dot(self.landing_forward), vec.dot(self.landing_right), 0.)
    }

    // pub fn localize_location(car: &Car, vec: Vec3A) -> Vec3A {
    //     localize(car, vec - car.location)
    // }

    #[inline]
    pub fn localize(&self, vec: Vec3A) -> Vec3A {
        Vec3A::new(vec.dot(self.forward), vec.dot(self.right), vec.dot(self.up))
    }

    // pub fn globalize(car: &Car, vec: Vec3A) -> Vec3A {
    //     car.forward * vec.x + car.right * vec.y + car.up * vec.z + car.location
    // }

    pub fn jump_time_to_height(&self, gravity: f32, height_goal: f32) -> f32 {
        let g = gravity * SIMULATION_DT;

        let mut t = 0.;
        let mut v_z = self.landing_velocity.z;
        let mut l_z = self.landing_location.z;

        while l_z < height_goal && (v_z > 0. || t < MAX_HOLD_TIME) {
            if t <= f32::EPSILON {
                v_z += JUMP_SPEED;
            }

            if t < MAX_HOLD_TIME {
                v_z += HOLD_BONUS * SIMULATION_DT;
            }

            if t < STICKY_TIMER {
                v_z += STICKY_FORCE * SIMULATION_DT;
            }

            t += SIMULATION_DT;
            v_z += g;
            l_z += v_z * SIMULATION_DT;
        }

        t
    }

    pub fn double_jump_time_to_height(&self, gravity: f32, height_goal: f32) -> f32 {
        let g = gravity * SIMULATION_DT;

        let mut t = 0.;
        let mut v_z = self.landing_velocity.z;
        let mut l_z = self.landing_location.z;
        let mut double_jumped = false;

        while l_z < height_goal && (v_z > 0. || t < MAX_HOLD_TIME) {
            if t <= f32::EPSILON {
                v_z += JUMP_SPEED;
            } else if t > MAX_HOLD_TIME + SIMULATION_DT && !double_jumped {
                v_z += JUMP_SPEED;
                double_jumped = true;
            }

            if t < MAX_HOLD_TIME {
                v_z += HOLD_BONUS * SIMULATION_DT;
            }

            if t < STICKY_TIMER {
                v_z += STICKY_FORCE * SIMULATION_DT;
            }

            t += SIMULATION_DT;
            v_z += g;
            l_z += v_z * SIMULATION_DT;
        }

        t
    }
}

#[allow(clippy::field_reassign_with_default)]
#[allow(dead_code)]
pub fn get_one() -> Car {
    let mut car = super::Car::default();

    // set all the values in the car
    car.location = Vec3A::new(-3000., 1500., 20.);
    car.velocity = Vec3A::new(0., 0., 0.);
    car.angular_velocity = Vec3A::new(0., 0., 0.);
    car.pitch = 0.;
    car.yaw = 0.5;
    car.roll = 0.;
    car.hitbox = Hitbox {
        length: 118.,
        width: 84.2,
        height: 36.2,
    };
    car.hitbox_offset = Vec3A::new(13.9, 0., 20.8);
    car.boost = 48;
    car.demolished = false;
    car.airborne = false;
    car.jumped = false;
    car.doublejumped = false;

    car.init(-650., 720, Mutators::new());

    car
}

#[cfg(test)]
mod tests {
    use crate::car::get_one;

    #[test]
    pub fn init_car() {
        get_one();
    }
}
