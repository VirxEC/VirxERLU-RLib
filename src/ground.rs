use std::f32::INFINITY;

use dubins_paths::{DubinsError, DubinsIntermediateResults, DubinsPath, DubinsPathType, DubinsResult};
use glam::Vec3A;
use rl_ball_sym::simulation::ball::Ball;

use crate::car::{throttle_acceleration, Car, CarFieldRect};
use crate::constants::*;
use crate::pytypes::{AdvancedShotInfo, ShotType};
use crate::shot::Shot;
use crate::utils::*;

fn angle_2d(vec1: Vec3A, vec2: Vec3A) -> f32 {
    flatten(vec1).dot(flatten(vec2)).clamp(-1., 1.).acos()
}

pub fn path_point_to_vec3(endpoint: [f32; 3]) -> Vec3A {
    Vec3A::new(endpoint[0], endpoint[1], 0.)
}

fn shortest_path_in_validate(q0: [f32; 3], q1: [f32; 3], rho: f32, car_field: &CarFieldRect, max_distance: f32) -> DubinsResult<DubinsPath> {
    let mut best_cost = INFINITY;
    let mut best_path = None;

    let intermediate_results = DubinsIntermediateResults::from(q0, q1, rho);

    for path_type in DubinsPathType::ALL {
        if let Ok(param) = intermediate_results.word(path_type) {
            let cost = param[0] + param[1] + param[2];
            if cost < best_cost && cost * rho <= max_distance {
                let path = DubinsPath {
                    qi: q0,
                    rho,
                    param,
                    type_: path_type,
                };

                if car_field.is_path_in(&path) {
                    best_cost = cost;
                    best_path = Some(path);
                }
            }
        }
    }

    match best_path {
        Some(path) => Ok(path),
        None => Err(DubinsError::NoPath),
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TargetInfo {
    pub distances: [f32; 4],
    pub path: DubinsPath,
    pub target: Option<Vec3A>,
    pub shot_type: usize,
}

impl TargetInfo {
    pub const fn from(distances: [f32; 4], shot_type: usize, path: DubinsPath) -> Self {
        Self {
            distances,
            path,
            shot_type,
            target: None,
        }
    }

    pub const fn from_target(distances: [f32; 4], shot_type: usize, target: Vec3A, path: DubinsPath) -> Self {
        Self {
            distances,
            path,
            shot_type,
            target: Some(target),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AnalyzeOptions {
    pub max_speed: f32,
    pub max_turn_radius: f32,
    pub get_target: bool,
}

pub fn analyze_target(ball: &Ball, car: &Car, shot_vector: Vec3A, time_remaining: f32, options: AnalyzeOptions) -> DubinsResult<TargetInfo> {
    let offset_target = ball.location - (shot_vector * ball.radius);
    let car_front_length = (car.hitbox_offset.x + car.hitbox.length) / 2.;

    let max_distance = time_remaining * options.max_speed + car_front_length;

    if flatten(car.location).distance(flatten(offset_target)) > max_distance {
        return Err(DubinsError::NoPath);
    }

    let shot_type = ShotType::GROUND;
    // will also be used to set offsets for jumps
    let offset_distance = car_front_length + {
        let distance = 320.;
        if (0_f32..distance).contains(&car.forward.dot(ball.location)) && car.right.dot(ball.location) < car.hitbox.width / 2. && angle_2d(car.forward, shot_vector) < 0.02 {
            0. // for pre-aligned ground shots
        } else {
            distance // for non-aligned ground shots
        }
    };

    let end_distance = offset_distance - car_front_length;
    let exit_turn_target = offset_target - (shot_vector * offset_distance);

    if !car.field.is_point_in(&[exit_turn_target.x, exit_turn_target.y, 0.]) {
        return Err(DubinsError::NoPath);
    }

    let target_angle = shot_vector.y.atan2(shot_vector.x);

    let q0 = [car.location.x, car.location.y, car.yaw];
    let q1 = [exit_turn_target.x, exit_turn_target.y, target_angle];

    let path = shortest_path_in_validate(q0, q1, options.max_turn_radius, &car.field, max_distance)?;

    let distances = [path.segment_length(0), path.segment_length(1), path.segment_length(2), end_distance];

    Ok(if options.get_target {
        let distance = car.local_velocity.x.max(500.) * STEER_REACTION_TIME;
        let max_path_distance = path.length();

        let target = if distance > max_path_distance {
            let distance_remaining = distance - max_path_distance;
            let additional_space = shot_vector * distance_remaining;

            path_point_to_vec3(path.sample(max_path_distance)) + additional_space
        } else {
            path_point_to_vec3(path.sample(distance))
        };

        TargetInfo::from_target(distances, shot_type, target, path)
    } else {
        TargetInfo::from(distances, shot_type, path)
    })
}

pub fn can_reach_target(car: &Car, max_time: f32, distances: [f32; 4], path_type: DubinsPathType, turn_rad: f32, is_forwards: bool) -> Result<f32, ()> {
    let is_curved = DubinsPathType::CCC.contains(&path_type);

    let total_d = distances.iter().sum::<f32>();

    let middle_range = {
        let start = distances[0];
        let end = start + distances[1];

        start..end
    };

    let mut d = total_d;
    let mut t_r = max_time;
    let b_s = car.boost.min(12) as f32;
    let mut b = car.boost as f32 - b_s;
    let mut v = car.local_velocity.x;

    let direction = is_forwards as u8 as f32;

    loop {
        if distances[3] < f32::EPSILON && d < 1. {
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
            if d <= distances[3] + 1. {
                break;
            } else if is_middle_straight {
                // skip ahead to the end of the section
                let final_d = total_d - distances[1] - distances[0];
                let delta_d = final_d - d;

                d = final_d;
                t_r -= delta_d / r;

                continue;
            }
        }

        if is_forwards {
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
        let (throttle, boost) = get_throttle_and_boost(throttle_accel, b, t);
        let mut accel = 0.;

        if throttle == 0. {
            accel += COAST_ACC * SIMULATION_DT * -v.signum();
        } else if throttle.signum() == v.signum() {
            accel += throttle_accel * SIMULATION_DT * throttle;
        } else {
            accel += BRAKE_ACC_DT.copysign(throttle);
        }

        if boost {
            accel += BOOST_ACCEL_DT;
            b -= BOOST_CONSUMPTION_DT;
        }

        if !(is_middle_straight || d < distances[3]) {
            accel -= turn_rad / 2. * SIMULATION_DT;
            accel = accel.min(2295. - v);
        }

        v += accel;

        t_r -= SIMULATION_DT;
        let d_delta = (v * direction) * SIMULATION_DT;
        d -= d_delta;
    }

    Ok(t_r)
}

fn get_throttle_and_boost(throttle_accel: f32, b: f32, t: f32) -> (f32, bool) {
    let acceleration = t / REACTION_TIME;
    let throttle_boost_transition = throttle_accel + 0.5 * BOOST_ACCEL;

    if acceleration <= BRAKE_COAST_TRANSITION {
        (-1., false)
    } else if BRAKE_COAST_TRANSITION < acceleration && acceleration < COASTING_THROTTLE_TRANSITION {
        (0., false)
    } else if COASTING_THROTTLE_TRANSITION <= acceleration && acceleration <= throttle_boost_transition {
        let throttle = if throttle_accel == 0. { 1. } else { (acceleration / throttle_accel).clamp(0.02, 1.) };
        (throttle, false)
    } else if b >= MIN_BOOST_CONSUMPTION && throttle_boost_transition < acceleration {
        (1., t > 0.)
    } else {
        (0., false)
    }
}

impl AdvancedShotInfo {
    pub fn get(car: &Car, shot: &Shot) -> Self {
        let (distance_along, index) = shot.get_distance_along_shot_and_index(car.location);

        let distance = car.local_velocity.x.max(500.) * STEER_REACTION_TIME;

        let path_length = shot.distances[0] + shot.distances[1] + shot.distances[2];
        let max_path_distance = path_length - distance_along;
        let distance_to_ball = path_length + shot.distances[3] - distance_along;

        let target = if distance > max_path_distance {
            let distance_remaining = distance - max_path_distance;
            let additional_space = shot.direction * distance_remaining;

            shot.path_endpoint + additional_space
        } else {
            path_point_to_vec3(shot.path.sample(distance_along + distance))
        };

        // get all the samples from the vec after index
        let samples = shot.all_samples.iter().skip(index / Shot::ALL_STEP).cloned().collect();

        Self::from(target, distance_to_ball, samples)
    }
}

// test can_reach_target
#[cfg(test)]
mod tests {
    use dubins_paths::DubinsPathType;

    #[test]
    fn test_can_reach_target() {
        let car = crate::car::get_a_car();

        let max_time = 10.;

        let distances = [1000., 500., 1000., 320.];

        let path_type = DubinsPathType::LRL;
        let rho = 500.;

        let is_forwards = true;

        let result = super::can_reach_target(&car, max_time, distances, path_type, rho, is_forwards);

        match result {
            Ok(t) => println!("{}", t),
            Err(e) => dbg!(e),
        }
    }
}
