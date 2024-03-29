use std::f32::consts::PI;

use dubins_paths::{NoPathError, Result as DubinsResult};
use glam::{Vec2, Vec3A};

use crate::{
    car::{Car, State},
    constants::*,
    pytypes::{BasicShotInfo, ShotType},
    BoostAmount, Mutators,
};

#[inline]
fn angle_3d(a: Vec3A, b: Vec3A) -> f32 {
    a.dot(b).acos()
}

#[derive(Debug)]
pub struct AerialTargetInfo {
    pub shot_vector: Vec3A,
    pub jump_type: AerialJumpType,
    pub final_target: Vec3A,
    pub wait_for_land: bool,
}

impl AerialTargetInfo {
    #[inline]
    #[must_use]
    pub const fn get_basic_shot_info(&self, time: f32) -> BasicShotInfo {
        BasicShotInfo::found(time, ShotType::Aerial, self.shot_vector, true, self.wait_for_land)
    }
}

/// Estimation of if a pre-established aerial shot is still possible
#[must_use]
pub fn partial_validate(
    target: Vec3A,
    xf: Vec3A,
    vf: Vec3A,
    boost_amount: BoostAmount,
    boost_accel: f32,
    car_boost: f32,
    time_remaining: f32,
) -> bool {
    let delta_x = target - xf;
    let Some(f) = delta_x.try_normalize() else {
        return true;
    };

    let required_acc = 2. * delta_x.length() / time_remaining.powi(2);
    let ratio = required_acc / boost_accel;
    if ratio.abs() > 1. {
        return false;
    }

    let tau2 = time_remaining - time_remaining * (1. - ratio).sqrt();
    if boost_amount != BoostAmount::Unlimited && (tau2 * BOOST_CONSUMPTION).floor() >= car_boost {
        return false;
    }

    (vf + f * (boost_accel * tau2)).length() <= MAX_SPEED
}

#[derive(Debug)]
struct BasicAerialInfo {
    car_forward: Vec3A,
    car_boost: f32,
    boost_amount: BoostAmount,
    boost_accel: f32,
    target: Vec3A,
    time_remaining: f32,
}

impl BasicAerialInfo {
    /// Estimation of if the aerial is valid
    fn validate(&self, xf: Vec3A, vf: Vec3A, jump_type: AerialJumpType) -> Option<(AerialJumpType, f32)> {
        let delta_x = self.target - xf;
        let f = delta_x.normalize();
        let phi = angle_3d(f, self.car_forward);

        let turn_time = if phi < 0.1 {
            TURN_TIME_POLY_EST[7] * phi as f64 + TURN_TIME_POLY_EST[8]
        } else {
            let x = Vec2::new(delta_x.z.atan2(delta_x.x), delta_x.y.atan2(delta_x.x)).length() as f64;

            if x < 0.2 {
                TURN_TIME_POLY_EST[9] * x.powi(2) + TURN_TIME_POLY_EST[10] * x + TURN_TIME_POLY_EST[11]
            } else {
                TURN_TIME_POLY_EST[0] * x.powi(6)
                    + TURN_TIME_POLY_EST[1] * x.powi(5)
                    + TURN_TIME_POLY_EST[2] * x.powi(4)
                    + TURN_TIME_POLY_EST[3] * x.powi(3)
                    + TURN_TIME_POLY_EST[4] * x.powi(2)
                    + TURN_TIME_POLY_EST[5] * x
                    + TURN_TIME_POLY_EST[6]
            }
        } as f32;

        if turn_time > self.time_remaining {
            return None;
        }

        let required_acc = 2. * delta_x.length() / (self.time_remaining - turn_time).powi(2);
        let ratio = required_acc / self.boost_accel;
        if ratio.abs() >= 0.9 {
            return None;
        }

        // when we stop boosting
        let tau2 = self.time_remaining - (self.time_remaining - turn_time) * (1. - ratio).sqrt();

        let boost_estimate = (tau2 - turn_time).floor() * BOOST_CONSUMPTION;
        if self.boost_amount != BoostAmount::Unlimited && boost_estimate.ceil() >= self.car_boost {
            return None;
        }

        // velocity estimate
        if (vf + f * (self.boost_accel * (tau2 - turn_time))).length() >= MAX_SPEED * 0.9 {
            return None;
        }

        Some((jump_type, boost_estimate))
    }
}

#[repr(i8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AerialJumpType {
    Secondary = -1,
    #[default]
    None = 0,
    Normal,
    Double,
}

pub fn aerial_shot_is_viable(
    car: &Car,
    mutators: Mutators,
    gravity: Vec3A,
    target: Vec3A,
    shot_vector: Vec3A,
    time_remaining: f32,
    check_target_angle: Option<Vec3A>,
) -> DubinsResult<AerialTargetInfo> {
    let is_on_ground = car.car_state == State::Grounded || time_remaining > car.time_to_land;

    if is_on_ground && car.rotmat.z_axis.z >= 0. && time_remaining <= JUMP_MAX_DURATION {
        return Err(NoPathError);
    }

    let land_time = if car.car_state != State::Grounded {
        car.time_to_land
    } else {
        car.last_landing_time
    };
    if is_on_ground && land_time + ON_GROUND_WAIT_TIME > time_remaining {
        return Err(NoPathError);
    }

    let quick_speed_required = car.location.distance(target) / time_remaining;
    if quick_speed_required > MAX_SPEED {
        return Err(NoPathError);
    }

    let max_car_speed = (car.velocity.length() + mutators.boost_accel * time_remaining).min(MAX_SPEED);
    if quick_speed_required > max_car_speed {
        return Err(NoPathError);
    }

    let mut found: Vec<(AerialJumpType, f32, bool)> = Vec::with_capacity(3);

    let boost_accel = mutators.boost_accel + AERIAL_THROTTLE_ACCEL;

    let vf_base = car.velocity + gravity * time_remaining;
    let xf_base = car.velocity * time_remaining + gravity * 0.5 * time_remaining.powi(2);

    let target_angle_check = |car_location: Vec3A| {
        check_target_angle.map_or(true, |ball_location| {
            angle_3d((car_location - ball_location).normalize(), -shot_vector) < PI / 2.
        })
    };

    let ground_time_remaining = time_remaining - car.time_to_land - car.wait_to_jump_time;
    if is_on_ground && ground_time_remaining > 0. && target_angle_check(car.landing_location) {
        const TOTAL_JUMP_ACC: f32 = JUMP_SPEED + JUMP_ACC * JUMP_MAX_DURATION;

        let basic_aerial_info = BasicAerialInfo {
            car_forward: car.landing_rotmat.x_axis,
            car_boost: f32::from(car.boost),
            boost_amount: mutators.boost_amount,
            boost_accel,
            target,
            time_remaining: ground_time_remaining,
            // car,
        };

        if time_remaining > DOUBLE_JUMP_DURATION {
            const TOTAL_JUMP_ACC_2: f32 = JUMP_SPEED + TOTAL_JUMP_ACC;
            const PARITAL_JUMP_LOC: f32 = 2. * JUMP_SPEED + JUMP_ACC * JUMP_MAX_DURATION;
            const JUMP_LOC_P2: f32 =
                -(JUMP_SPEED * JUMP_MAX_DURATION + 0.5 * JUMP_MAX_DURATION * JUMP_MAX_DURATION * JUMP_ACC);

            let vf = vf_base + car.rotmat.z_axis * TOTAL_JUMP_ACC_2;
            let xf = car.landing_location + xf_base + car.rotmat.z_axis * (time_remaining * PARITAL_JUMP_LOC + JUMP_LOC_P2);

            if let Some((jump_type, boost)) = basic_aerial_info.validate(xf, vf, AerialJumpType::Double) {
                found.push((jump_type, boost, true));
            }
        }

        if time_remaining > JUMP_MAX_DURATION {
            const PARITAL_JUMP_LOC: f32 = JUMP_SPEED + JUMP_ACC * JUMP_MAX_DURATION;
            const JUMP_LOC_P2: f32 = -0.5 * JUMP_MAX_DURATION * JUMP_MAX_DURATION * JUMP_ACC;

            let vf = vf_base + car.rotmat.z_axis * TOTAL_JUMP_ACC;
            let xf = car.landing_location + xf_base + car.rotmat.z_axis * (time_remaining * PARITAL_JUMP_LOC + JUMP_LOC_P2);

            if let Some((jump_type, boost)) = basic_aerial_info.validate(xf, vf, AerialJumpType::Normal) {
                found.push((jump_type, boost, true));
            }
        }
    }

    if target_angle_check(car.location) {
        let basic_aerial_info = BasicAerialInfo {
            car_forward: car.rotmat.x_axis,
            car_boost: f32::from(car.boost),
            boost_amount: mutators.boost_amount,
            boost_accel,
            target,
            time_remaining,
            // car,
        };

        if car.car_state != State::DoubleJumped
            && (!is_on_ground
                || (car.car_state != State::Grounded
                    && (car.velocity.z + gravity.z * car.time_to_land)
                        + mutators.boost_accel * car.time_to_land
                        + JUMP_SPEED
                        > 0.))
        {
            let vf = vf_base + car.rotmat.z_axis * JUMP_SPEED;
            let xf = car.location + xf_base + car.rotmat.z_axis * (JUMP_SPEED * time_remaining);

            if let Some((jump_type, boost)) = basic_aerial_info.validate(xf, vf, AerialJumpType::Secondary) {
                found.push((jump_type, boost, false));
            }
        }

        if !is_on_ground
            || car.rotmat.z_axis.z < 0.
            || (car.car_state != State::Grounded
                && (car.velocity.z + gravity.z * car.time_to_land) + mutators.boost_accel * car.time_to_land > 0.)
        {
            if let Some((jump_type, boost)) =
                basic_aerial_info.validate(car.location + xf_base, vf_base, AerialJumpType::None)
            {
                found.push((jump_type, boost, false));
            }
        }
    }

    if found.is_empty() {
        return Err(NoPathError);
    }

    // println!("{found:?}");
    let min_boost_estimate = found
        .into_iter()
        .min_by(|(jump_type, boost_estimate, _), (jump_type_2, boost_estimate_2, _)| {
            boost_estimate
                .partial_cmp(boost_estimate_2)
                .unwrap_or_else(|| panic!("Invalid boost estimate: either {boost_estimate} ({jump_type:?}) or {boost_estimate_2} ({jump_type_2:?})"))
        })
        .ok_or(NoPathError)?;

    Ok(AerialTargetInfo {
        shot_vector,
        jump_type: min_boost_estimate.0,
        final_target: target,
        wait_for_land: min_boost_estimate.2,
    })
}
