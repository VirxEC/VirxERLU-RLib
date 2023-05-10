use std::f32::{consts::E, INFINITY};

use dubins_paths::{DubinsPath, Intermediate, NoPathError, PathType, PosRot, Result as DubinsResult};
use glam::Vec3A;

use crate::{
    car::{throttle_acceleration, Car, FieldRect},
    constants::*,
    pytypes::{BasicShotInfo, ShotType},
    utils::*,
    BoostAmount, Mutators,
};

/// <https://stackoverflow.com/a/49987361/10930209>
fn get_turn_exit_tangets(target: Vec3A, circle_center: Vec3A, radius: f32) -> (Vec3A, Vec3A) {
    let circle_center_to_target = target - circle_center;
    let b = circle_center_to_target.length();
    let th = (radius / b).acos();
    let d = circle_center_to_target.y.atan2(circle_center_to_target.x);
    let (d1s, d1c) = (d + th).sin_cos();
    let (d2s, d2c) = (d - th).sin_cos();

    (circle_center + radius * Vec3A::new(d1c, d1s, 0.), circle_center + radius * Vec3A::new(d2c, d2s, 0.))
}

/// Get the exit turn point on a circle where the car will face the target
pub fn get_turn_exit_tanget(car: &Car, target: Vec3A, circle_center: Vec3A, rho: f32, mut target_is_forwards: bool, travel_forwards: bool) -> (Vec3A, Vec3A) {
    let (t1, t2) = get_turn_exit_tangets(target, circle_center, rho);
    let (mut t1_local, mut t2_local) = (car.localize_2d_location(t1), car.localize_2d_location(t2));

    if !travel_forwards {
        t1_local.x *= -1.;
        t2_local.x *= -1.;
        target_is_forwards = !target_is_forwards;
    }

    if !target_is_forwards {
        if t1_local.x >= 0. && t1_local.y.abs() < rho {
            return (t2, t1);
        }

        if t2_local.x >= 0. && t2_local.y.abs() < rho {
            return (t1, t2);
        }
    }

    let (t1_angle, t2_angle) = (t1_local.y.atan2(t1_local.x).abs(), t2_local.y.atan2(t2_local.x).abs());

    if t1_angle < t2_angle {
        (t1, t2)
    } else {
        (t2, t1)
    }
}

#[inline]
pub fn angle_2d(vec1: Vec3A, vec2: Vec3A) -> f32 {
    flatten(vec1).normalize_or_zero().dot(flatten(vec2).normalize_or_zero()).clamp(-1., 1.).acos()
}

pub fn shortest_path_in_validate(q0: PosRot, q1: PosRot, rho: f32, car_field: &FieldRect, max_distance: f32) -> DubinsResult<DubinsPath> {
    let mut best_cost = INFINITY;
    let mut best_path = None;

    let intermediate_results = Intermediate::from(q0, q1, rho);

    for (i, param) in PathType::ALL.into_iter().flat_map(|path_type| intermediate_results.word(path_type)).enumerate() {
        let cost = param[0] + param[1] + param[2];
        if cost < best_cost && cost * rho <= max_distance {
            let path = DubinsPath {
                qi: q0,
                rho,
                param,
                type_: PathType::ALL[i],
            };

            if car_field.is_path_in(&path) {
                best_cost = cost;
                best_path = Some(path);
            }
        }
    }

    best_path.ok_or(NoPathError)
}

#[derive(Clone, Copy, Debug)]
pub struct GroundTargetInfo {
    pub distances: [f32; 4],
    pub path: DubinsPath,
    pub shot_type: ShotType,
    pub jump_time: Option<f32>,
    pub is_forwards: bool,
    pub shot_vector: Vec3A,
    pub turn_targets: Option<(Vec3A, Vec3A)>,
    pub wait_for_land: bool,
}

impl GroundTargetInfo {
    pub fn can_reach(&self, car: &Car, max_time: f32, mutators: Mutators) -> Result<f32, ()> {
        let is_curved = PathType::CCC.contains(&self.path.type_);

        let total_d = self.distances.iter().sum::<f32>();

        let middle_range = {
            let start = self.distances[0];
            let end = start + self.distances[1];

            start..end
        };

        let direction = if self.is_forwards { 1. } else { -1. };

        let mut d = total_d;
        let mut t_r = max_time;
        let b_s = f32::from(car.boost.min(12));
        let mut b = f32::from(car.boost) - b_s;
        let mut v = flatten(car.landing_velocity).length() * direction;

        loop {
            if self.distances[3] < f32::EPSILON && d < 1. {
                return Ok(t_r.max(0.));
            }

            if t_r <= 0. {
                return Err(());
            }

            let r = d * direction / t_r;
            let t = r - v;

            let distance_traveled = total_d - d;
            let is_middle_straight = !is_curved && middle_range.contains(&distance_traveled);

            if t.abs() < 20. {
                if d <= self.distances[3] + 1. {
                    break;
                } else if is_middle_straight {
                    // skip ahead to the end of the section
                    let final_d = total_d - self.distances[1] - self.distances[0];
                    let delta_d = final_d - d;

                    if delta_d > 0. {
                        d = final_d;
                        t_r -= delta_d / r;
                        continue;
                    }
                }
            }

            if self.is_forwards {
                let quick_max_speed = if b >= 1. {
                    MAX_SPEED.min(MAX_SPEED_NO_BOOST.max(v) + BOOST_ACCEL * t_r.min(b / BOOST_CONSUMPTION))
                } else {
                    MAX_SPEED_NO_BOOST.max(v)
                };

                if r > quick_max_speed {
                    return Err(());
                }
            } else if MIN_SPEED > r {
                return Err(());
            }

            let throttle_accel = throttle_acceleration(v);
            let (throttle, boost) = {
                let (mut throttle, mut boost) = get_throttle_and_boost(throttle_accel, b, if v < 0. { -t } else { t });

                if t <= 0. {
                    boost = false;
                }

                if v < 0. {
                    throttle *= -1.;
                }

                (throttle, boost)
            };
            let mut accel = 0.;

            if throttle == 0. {
                accel += COAST_ACC * SIMULATION_DT * -v.signum();
            } else if throttle.signum() == v.signum() {
                accel += throttle_accel * SIMULATION_DT * throttle;
            } else {
                accel += BRAKE_ACC_DT.copysign(throttle);
            }

            if mutators.boost_amount != BoostAmount::NoBoost && boost {
                accel += mutators.boost_accel * SIMULATION_DT;
                if mutators.boost_amount != BoostAmount::Unlimited {
                    b -= BOOST_CONSUMPTION_DT;
                }
            }

            if !(is_middle_straight || d < self.distances[3]) {
                accel -= self.path.rho / E * SIMULATION_DT;
                accel = accel.min(2295. - v);
            }

            v += accel;

            t_r -= SIMULATION_DT;
            let d_delta = (v * direction) * SIMULATION_DT;
            d -= d_delta;
        }

        Ok(t_r)
    }

    #[inline]
    pub const fn get_basic_shot_info(&self, time: f32) -> BasicShotInfo {
        BasicShotInfo::found(time, self.shot_type, self.shot_vector, self.is_forwards, self.wait_for_land)
    }
}

fn get_throttle_and_boost(throttle_accel: f32, b: f32, t: f32) -> (f32, bool) {
    let acceleration = t / REACTION_TIME;
    let throttle_boost_transition = throttle_accel + 0.5 * BOOST_ACCEL;

    if acceleration <= BRAKE_COAST_TRANSITION {
        (-1., false)
    } else if BRAKE_COAST_TRANSITION < acceleration && acceleration < COASTING_THROTTLE_TRANSITION {
        (0., false)
    } else if (COASTING_THROTTLE_TRANSITION..throttle_boost_transition).contains(&acceleration) {
        let throttle = if throttle_accel == 0. { 1. } else { (acceleration / throttle_accel).clamp(0.02, 1.) };
        (throttle, false)
    } else if throttle_boost_transition < acceleration {
        (1., b >= MIN_BOOST_CONSUMPTION && t > 0.)
    } else {
        (0., false)
    }
}
