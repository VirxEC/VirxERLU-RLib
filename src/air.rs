use crate::{
    car::Car,
    constants::*,
    pytypes::{BasicShotInfo, ShotType},
    BoostAmount, Mutators,
};
use dubins_paths::NoPathError;
use glam::Vec3A;

fn angle_3d(a: Vec3A, b: Vec3A) -> f32 {
    a.normalize_or_zero().dot(b.normalize_or_zero()).clamp(-1.0, 1.0).acos()
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
    fn validate(&self, xf: Vec3A, vf: Vec3A) -> Option<f32> {
        let delta_x = self.target - xf;
        let f = delta_x.normalize();

        let phi = angle_3d(f, self.car_forward);
        // TODO: replace with turn simulation
        let turn_time = 2. * (phi / 9.).sqrt();

        let tau1 = turn_time * (1. - 0.3 / phi).clamp(0., 1.);
        let required_acc = 2. * delta_x.length() / (self.time_remaining - tau1).powi(2);

        let ratio = required_acc / self.boost_accel;
        if ratio.abs() >= 1. {
            return None;
        }

        let tau2 = self.time_remaining - (self.time_remaining - tau1) * (1. - ratio.clamp(0., 1.)).sqrt();

        // velocity estimate

        if (vf + f * (self.boost_accel * (tau2 - tau1))).length() >= MAX_SPEED {
            return None;
        }

        let boost_estimate = (tau2 - tau1).floor() * BOOST_CONSUMPTION;

        if self.boost_amount == BoostAmount::Unlimited || boost_estimate.ceil() < self.car_boost {
            Some(boost_estimate)
        } else {
            None
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum AerialJumpType {
    Secondary = -1,
    None = 0,
    Normal,
    Double,
}

pub fn aerial_shot_is_viable(car: &Car, mutators: Mutators, target: Vec3A, gravity: Vec3A, time_remaining: f32) -> Result<AerialTargetInfo, NoPathError> {
    let is_on_ground = !car.airborne || time_remaining > car.landing_time;

    if is_on_ground && car.up.z >= 0. && car.max_jump_time > time_remaining {
        return Err(NoPathError);
    }

    let car_location = if is_on_ground { car.landing_location } else { car.location };

    let quick_speed_required = car_location.distance(target) / time_remaining;
    if quick_speed_required > MAX_SPEED || quick_speed_required > car.velocity.length() + mutators.boost_accel * time_remaining {
        return Err(NoPathError);
    }

    let mut found: Vec<(AerialJumpType, f32)> = Vec::with_capacity(3);

    let mut add_result = |jump_type, boost_estimate| {
        if let Some(boost_estimate) = boost_estimate {
            found.push((jump_type, boost_estimate));
        }
    };

    let boost_accel = mutators.boost_accel + AERIAL_THROTTLE_ACCEL;

    let vf_base = car.velocity + gravity * time_remaining;
    let xf_base = car.location + car.velocity * time_remaining + gravity * 0.5 * time_remaining.powi(2);

    let basic_aerial_info = BasicAerialInfo {
        car_forward: if is_on_ground { car.landing_forward } else { car.forward },
        car_boost: car.boost as f32,
        boost_amount: mutators.boost_amount,
        boost_accel,
        target,
        time_remaining: if is_on_ground { time_remaining - car.landing_time } else { time_remaining },
    };

    if is_on_ground {
        let total_jump_acc = JUMP_SPEED + JUMP_ACC * JUMP_MAX_DURATION;
        let partial_jump_loc = JUMP_ACC * (time_remaining * JUMP_MAX_DURATION - 0.5 * JUMP_MAX_DURATION.powi(2));

        if time_remaining > JUMP_MAX_DURATION + SIMULATION_DT * 2. {
            let vf = vf_base + car.up * (JUMP_SPEED + total_jump_acc);
            let xf = xf_base + car.up * (JUMP_SPEED * (2. * time_remaining - JUMP_MAX_DURATION) + partial_jump_loc);

            add_result(AerialJumpType::Double, basic_aerial_info.validate(xf, vf));
        }

        if time_remaining > JUMP_MAX_DURATION {
            let vf = vf_base + car.up * total_jump_acc;
            let xf = xf_base + car.up * (JUMP_SPEED * time_remaining + partial_jump_loc);

            add_result(AerialJumpType::Normal, basic_aerial_info.validate(xf, vf));
        }
    }

    if !car.doublejumped && (!is_on_ground || (car.airborne && (car.velocity.z + gravity.z * car.landing_time) + mutators.boost_accel * car.landing_time + JUMP_SPEED > 0.))  {
        let vf = vf_base + car.up * JUMP_SPEED;
        let xf = xf_base + car.up * (JUMP_SPEED * time_remaining);
    
        add_result(AerialJumpType::Secondary, basic_aerial_info.validate(xf, vf));
    }

    if !is_on_ground || car.up.z < 0. || (car.airborne && (car.velocity.z + gravity.z * car.landing_time) + mutators.boost_accel * car.landing_time > 0.) {
        add_result(AerialJumpType::None, basic_aerial_info.validate(xf_base, vf_base));
    }

    let min_boost_estimate = found
        .into_iter()
        .min_by(|(jump_type, boost_estimate), (jump_type_2, boost_estimate_2)| {
            boost_estimate
                .partial_cmp(boost_estimate_2)
                .unwrap_or_else(|| panic!("Invalid boost estimate: either {boost_estimate} ({jump_type:?}) or {boost_estimate_2} ({jump_type_2:?})"))
        })
        .ok_or(NoPathError)?;

    Ok(AerialTargetInfo {
        shot_vector: (target - car_location).normalize_or_zero(),
        jump_type: min_boost_estimate.0,
        final_target: target,
    })
}
