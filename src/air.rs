use std::f32::consts::PI;

use crate::{
    car::Car,
    constants::*,
    pytypes::{BasicShotInfo, ShotType},
    BoostAmount, Mutators,
};
use dubins_paths::{NoPathError, Result as DubinsResult};
use glam::Vec3A;

#[inline]
fn angle_3d(a: Vec3A, b: Vec3A) -> f32 {
    a.dot(b).clamp(-1., 1.).acos()
}

#[derive(Debug)]
pub struct AerialTargetInfo {
    pub shot_vector: Vec3A,
    pub jump_type: AerialJumpType,
    pub final_target: Vec3A,
}

impl AerialTargetInfo {
    #[inline]
    pub const fn get_basic_shot_info(&self, time: f32) -> BasicShotInfo {
        BasicShotInfo::found(time, ShotType::Aerial, self.shot_vector, true)
    }
}

/// Estimation of if a pre-established aerial shot is still possible
pub fn partial_validate(target: Vec3A, xf: Vec3A, vf: Vec3A, boost_amount: BoostAmount, boost_accel: f32, car_boost: f32, time_remaining: f32) -> bool {
    let delta_x = target - xf;
    let f = match delta_x.try_normalize() {
        Some(f) => f,
        None => return true,
    };

    let required_acc = delta_x.length() / time_remaining.powi(2);
    let ratio = required_acc / boost_accel;
    if ratio.abs() > 1. {
        return false;
    }

    let tau2 = time_remaining - time_remaining * (1. - ratio).sqrt();
    if boost_amount != BoostAmount::Unlimited && (tau2.floor() * BOOST_CONSUMPTION).ceil() >= car_boost {
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
        let f = delta_x.try_normalize()?;
        let phi = angle_3d(f, self.car_forward);

        let turn_time = 3. * (phi / 9.).sqrt();
        if turn_time > self.time_remaining {
            return None;
        }

        // when we start boosting
        let tau1 = turn_time * (1. - AERIAL_START_BOOST_ANGLE / phi.max(f32::EPSILON)).clamp(0., 1.);

        let required_acc = 2. * delta_x.length() / (self.time_remaining - tau1).powi(2);
        let ratio = required_acc / self.boost_accel;
        if ratio.abs() >= 0.9 {
            return None;
        }

        // when we stop boosting
        let tau2 = self.time_remaining - (self.time_remaining - tau1) * (1. - ratio.clamp(0., 1.)).sqrt();

        let boost_estimate = (tau2 - tau1).floor() * BOOST_CONSUMPTION;
        if self.boost_amount != BoostAmount::Unlimited && boost_estimate.ceil() >= self.car_boost {
            return None;
        }

        // velocity estimate
        if (vf + f * (self.boost_accel * (tau2 - tau1))).length() >= MAX_SPEED * 0.9 {
            return None;
        }

        Some((jump_type, boost_estimate))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AerialJumpType {
    Secondary = -1,
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
    let is_on_ground = !car.airborne || time_remaining > car.landing_time;

    if is_on_ground && car.up.z >= 0. && time_remaining <= JUMP_MAX_DURATION {
        return Err(NoPathError);
    }

    let land_time = if car.landing_time > f32::EPSILON { car.landing_time } else { car.last_landing_time };
    if is_on_ground && land_time + 0.6 > time_remaining {
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

    let mut found: Vec<(AerialJumpType, f32)> = Vec::with_capacity(3);

    let boost_accel = mutators.boost_accel + AERIAL_THROTTLE_ACCEL;

    let vf_base = car.velocity + gravity * time_remaining;
    let xf_base = car.velocity * time_remaining + gravity * 0.5 * time_remaining.powi(2);

    let target_angle_check =
        |car_location: Vec3A| check_target_angle.map_or(true, |ball_location| angle_3d((car_location - ball_location).normalize(), -shot_vector) < PI / 2.);

    let ground_time_remaining = time_remaining - car.landing_time;
    if is_on_ground && ground_time_remaining > 0. && target_angle_check(car.landing_location) {
        const TOTAL_JUMP_ACC: f32 = JUMP_SPEED + JUMP_ACC * JUMP_MAX_DURATION;

        let basic_aerial_info = BasicAerialInfo {
            car_forward: car.landing_forward,
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
            const JUMP_LOC_P2: f32 = -(JUMP_SPEED * JUMP_MAX_DURATION + 0.5 * JUMP_MAX_DURATION * JUMP_MAX_DURATION * JUMP_ACC);

            let vf = vf_base + car.up * TOTAL_JUMP_ACC_2;
            let xf = car.landing_location + xf_base + car.up * (time_remaining * PARITAL_JUMP_LOC + JUMP_LOC_P2);

            if let Some(result) = basic_aerial_info.validate(xf, vf, AerialJumpType::Double) {
                found.push(result);
            }
        }

        if time_remaining > JUMP_MAX_DURATION {
            const PARITAL_JUMP_LOC: f32 = JUMP_SPEED + JUMP_ACC * JUMP_MAX_DURATION;
            const JUMP_LOC_P2: f32 = -0.5 * JUMP_MAX_DURATION * JUMP_MAX_DURATION * JUMP_ACC;

            let vf = vf_base + car.up * TOTAL_JUMP_ACC;
            let xf = car.landing_location + xf_base + car.up * (time_remaining * PARITAL_JUMP_LOC + JUMP_LOC_P2);

            if let Some(result) = basic_aerial_info.validate(xf, vf, AerialJumpType::Normal) {
                found.push(result);
            }
        }
    }

    if target_angle_check(car.location) {
        let basic_aerial_info = BasicAerialInfo {
            car_forward: car.forward,
            car_boost: f32::from(car.boost),
            boost_amount: mutators.boost_amount,
            boost_accel,
            target,
            time_remaining,
            // car,
        };

        if !car.doublejumped
            && (!is_on_ground || (car.airborne && (car.velocity.z + gravity.z * car.landing_time) + mutators.boost_accel * car.landing_time + JUMP_SPEED > 0.))
        {
            let vf = vf_base + car.up * JUMP_SPEED;
            let xf = car.location + xf_base + car.up * (JUMP_SPEED * time_remaining);

            if let Some(result) = basic_aerial_info.validate(xf, vf, AerialJumpType::Secondary) {
                found.push(result);
            }
        }

        if !is_on_ground || car.up.z < 0. || (car.airborne && (car.velocity.z + gravity.z * car.landing_time) + mutators.boost_accel * car.landing_time > 0.) {
            if let Some(result) = basic_aerial_info.validate(car.location + xf_base, vf_base, AerialJumpType::None) {
                found.push(result);
            }
        }
    }

    if found.is_empty() {
        return Err(NoPathError);
    }

    // println!("{found:?}");
    let min_boost_estimate = found
        .into_iter()
        .min_by(|(jump_type, boost_estimate), (jump_type_2, boost_estimate_2)| {
            boost_estimate
                .partial_cmp(boost_estimate_2)
                .unwrap_or_else(|| panic!("Invalid boost estimate: either {boost_estimate} ({jump_type:?}) or {boost_estimate_2} ({jump_type_2:?})"))
        })
        .ok_or(NoPathError)?;

    Ok(AerialTargetInfo {
        shot_vector,
        jump_type: min_boost_estimate.0,
        final_target: target,
    })
}
