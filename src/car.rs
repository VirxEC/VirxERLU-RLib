use cpython::{ObjectProtocol, PyObject, PyResult, Python};
use glam::Vec3A;

use crate::{constants::*, utils::get_vec3_named};

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

pub fn turn_radius(v: f32) -> f32 {
    1. / curvature(v)
}

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
    pub fn update(&mut self, py: Python, py_car: PyObject) -> PyResult<()> {
        let py_car_physics = py_car.getattr(py, "physics")?;

        self.location = get_vec3_named(py, py_car_physics.getattr(py, "location")?)?;
        self.velocity = get_vec3_named(py, py_car_physics.getattr(py, "velocity")?)?;
        self.angular_velocity = get_vec3_named(py, py_car_physics.getattr(py, "angular_velocity")?)?;

        let py_car_rotator = py_car_physics.getattr(py, "rotation")?;

        self.pitch = py_car_rotator.getattr(py, "pitch")?.extract(py)?;
        self.yaw = py_car_rotator.getattr(py, "yaw")?.extract(py)?;
        self.roll = py_car_rotator.getattr(py, "roll")?.extract(py)?;

        let py_car_hitbox = py_car.getattr(py, "hitbox")?;

        self.hitbox = Hitbox {
            length: py_car_hitbox.getattr(py, "length")?.extract(py)?,
            width: py_car_hitbox.getattr(py, "width")?.extract(py)?,
            height: py_car_hitbox.getattr(py, "height")?.extract(py)?,
        };

        self.hitbox_offset = get_vec3_named(py, py_car.getattr(py, "hitbox_offset")?)?;

        self.boost = py_car.getattr(py, "boost")?.extract(py)?;
        self.demolished = py_car.getattr(py, "is_demolished")?.extract(py)?;
        self.airborne = !py_car.getattr(py, "has_wheel_contact")?.extract(py)?;
        self.jumped = py_car.getattr(py, "jumped")?.extract(py)?;
        self.doublejumped = py_car.getattr(py, "double_jumped")?.extract(py)?;

        self.calculate_orientation_matrix();
        self.calculate_max_values();
        self.calculate_local_values();
        self.calculate_field();

        Ok(())
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

    pub fn calculate_max_values(&mut self) {
        self.max_speed = self.get_max_speed();
        self.ctrms = turn_radius(self.max_speed);
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
