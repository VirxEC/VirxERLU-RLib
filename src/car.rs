use dubins_paths::DubinsPath;
use glam::Vec3A;
use pyo3::{PyAny, PyResult};

use crate::{
    constants::*,
    utils::{flatten, get_vec3_named, minimum_non_negative, vertex_quadratic_solve_for_x},
};

pub fn throttle_acceleration(forward_velocity: f32) -> f32 {
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

pub fn curvature(v: f32) -> f32 {
    let v = v.abs();

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

    panic!("Invalid input velocity: {}", v);
}

pub fn turn_radius(v: f32) -> f32 {
    1. / curvature(v)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Hitbox {
    pub length: f32,
    pub width: f32,
    pub height: f32,
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

    pub fn is_path_in(&self, path: &DubinsPath) -> bool {
        // instead of this, do a better "is arc in" or "is line in" thing
        for dist in [
            path.segment_length(0) / 2.,
            path.segment_length(0),
            path.segment_length(0) + path.segment_length(1) / 2.,
            path.segment_length(0) + path.segment_length(1),
            path.segment_length(0) + path.segment_length(1) + path.segment_length(2) / 2.,
        ] {
            if dist > 20. && !self.is_point_in(&path.sample(dist)) {
                return false;
            }
        }

        true
    }

    pub fn is_point_in(&self, p: &[f32; 3]) -> bool {
        if p[0].abs() > self.goal_x {
            return p[0].abs() < self.field_x && p[1].abs() < self.field_y;
        }

        p[1].abs() < self.goal_y
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
    pub field: CarFieldRect,
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
    pub max_speed: Vec<f32>,
    /// turn radius at calculated max speed
    pub ctrms: Vec<f32>,
}

impl Car {
    pub fn update(&mut self, py_car: &PyAny, gravity: f32, max_ball_slice: usize) -> PyResult<()> {
        let py_car_physics = py_car.getattr("physics")?;

        self.location = get_vec3_named(py_car_physics.getattr("location")?)?;
        self.velocity = get_vec3_named(py_car_physics.getattr("velocity")?)?;
        self.angular_velocity = get_vec3_named(py_car_physics.getattr("angular_velocity")?)?;

        let py_car_rotator = py_car_physics.getattr("rotation")?;

        self.pitch = py_car_rotator.getattr("pitch")?.extract()?;
        self.yaw = py_car_rotator.getattr("yaw")?.extract()?;
        self.roll = py_car_rotator.getattr("roll")?.extract()?;

        let py_car_hitbox = py_car.getattr("hitbox")?;

        self.hitbox = Hitbox {
            length: py_car_hitbox.getattr("length")?.extract()?,
            width: py_car_hitbox.getattr("width")?.extract()?,
            height: py_car_hitbox.getattr("height")?.extract()?,
        };

        self.hitbox_offset = get_vec3_named(py_car.getattr("hitbox_offset")?)?;

        self.boost = py_car.getattr("boost")?.extract()?;
        self.demolished = py_car.getattr("is_demolished")?.extract()?;
        self.airborne = !py_car.getattr("has_wheel_contact")?.extract()?;
        self.jumped = py_car.getattr("jumped")?.extract()?;
        self.doublejumped = py_car.getattr("double_jumped")?.extract()?;

        self.calculate_orientation_matrix();
        self.calculate_field();
        self.calculate_landing_info(gravity);
        self.calculate_local_values();
        self.calculate_max_values(max_ball_slice);

        Ok(())
    }

    pub fn calculate_landing_info(&mut self, gravity: f32) {
        self.landing_time = 0.;
        self.landing_location = self.location;
        self.landing_velocity = self.velocity;

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
            self.landing_velocity.z += gravity * time_until_tv;
            self.landing_location += Vec3A::new(
                self.velocity.x * self.landing_time,
                self.velocity.y * self.landing_time,
                gravity * (self.landing_time - h).powi(2) + k,
            );
            let g_dt = Vec3A::new(0., 0., gravity * dt);

            loop {
                let v_t = (self.landing_velocity + g_dt).normalize() * 2300.;
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
    }

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

    pub fn calculate_max_values(&mut self, max_ball_slice: usize) {
        let mut b = self.boost as f32;
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

            if 1. == v.signum() {
                accel += throttle_accel * SIMULATION_DT;
            } else {
                accel += BRAKE_ACC_DT;
            }

            if b > BOOST_CONSUMPTION_DT {
                accel += BOOST_ACCEL_DT;
                b -= BOOST_CONSUMPTION_DT;
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

        let mut v1 = v;
        let mut v2 = v;

        for _ in end..max_ball_slice {
            if fast_forward {
                self.max_speed.push(v1);
                continue;
            }

            let throttle_accel = throttle_acceleration(v1);
            let mut accel = 0.;

            if 1 == v1.signum() as usize {
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

            if 1 == v2.signum() as usize {
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
        self.field = CarFieldRect::from(&self.hitbox);
    }

    // pub fn localize_2d_location(&self, vec: Vec3A) -> Vec3A {
    //     self.localize_2d(vec - self.location)
    // }

    // pub fn localize_2d(&self, vec: Vec3A) -> Vec3A {
    //     Vec3A::new(vec.dot(self.forward), vec.dot(self.right), 0.)
    // }

    // pub fn localize_location(car: &Car, vec: Vec3A) -> Vec3A {
    //     localize(car, vec - car.location)
    // }

    pub fn localize(&self, vec: Vec3A) -> Vec3A {
        Vec3A::new(vec.dot(self.forward), vec.dot(self.right), vec.dot(self.up))
    }

    // pub fn globalize(car: &Car, vec: Vec3A) -> Vec3A {
    //     car.forward * vec.x + car.right * vec.y + car.up * vec.z + car.location
    // }
}

#[allow(clippy::field_reassign_with_default)]
#[allow(dead_code)]
pub fn get_a_car() -> Car {
    let mut car = super::Car::default();

    // set all the values in the car
    car.location = Vec3A::new(-3000., 1500., 100.);
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

    car.calculate_orientation_matrix();
    car.calculate_max_values(720);
    car.calculate_local_values();
    car.calculate_field();

    car
}

#[cfg(test)]
mod tests {
    #[test]
    fn get_max_speed() {
        let car = super::get_a_car();

        dbg!(car.max_speed);
        dbg!(car.ctrms);
    }
}
