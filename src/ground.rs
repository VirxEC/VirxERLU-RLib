use std::f32::INFINITY;

use dubins_paths::{DubinsError, DubinsIntermediateResults, DubinsPath, DubinsPathType, DubinsResult};
use glam::Vec3A;
use rl_ball_sym::simulation::ball::Ball;

use crate::car::{throttle_acceleration, Car, CarFieldRect};
use crate::constants::*;
use crate::pytypes::AdvancedShotInfo;
use crate::shot::Shot;
use crate::utils::*;

fn angle_2d(vec1: Vec3A, vec2: Vec3A) -> f32 {
    flatten(vec1).dot(flatten(vec2)).clamp(-1., 1.).acos()
}

// https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
// fn point_from_line(v: Vec3A, w: Vec3A, p: Vec3A) -> (f32, f32) {
//     // Return minimum distance between line segment vw and point p
//     let l2 = v.distance_squared(w); // i.e. |w-v|^2 -  avoid a sqrt

//     // v == w case
//     if l2 == 0. {
//         return (p.distance(v), 1.);
//     }

//     // Consider the line extending the segment, parameterized as v + t (w - v).
//     // We find projection of point p onto the line.
//     // It falls where t = [(p-v) . (w-v)] / |w-v|^2
//     // We clamp t from [0,1] to handle points outside the segment vw.
//     let t = ((p - v).dot(w - v) / l2).clamp(0., 1.);
//     let projection = lerp(w, v, t); // Projection falls on the segment

//     (p.distance(projection), t)
// }

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

                let mut valid = true;

                // instead of this, do a better "is arc in" or "is line in" thing
                for dist in [path.segment_length(0) / 2., (path.segment_length(0) + path.segment_length(1)) / 2., path.length() / 2.] {
                    if !car_field.is_point_in(&path.sample(dist)) {
                        valid = false;
                        break;
                    }
                }

                if !valid {
                    continue;
                }

                for dist in [path.segment_length(0), path.segment_length(0) + path.segment_length(1), path.length()] {
                    if !car_field.is_point_in(&path.sample(dist)) {
                        valid = false;
                        break;
                    }
                }

                if valid {
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
}

impl TargetInfo {
    pub const fn from(distances: [f32; 4], path: DubinsPath) -> Self {
        Self { distances, path, target: None }
    }

    pub const fn from_target(distances: [f32; 4], target: Vec3A, path: DubinsPath) -> Self {
        Self {
            distances,
            path,
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

    let target_angle = shot_vector.y.atan2(shot_vector.x);
    let max_distance = time_remaining * options.max_speed + car_front_length;

    if flatten(car.location).distance(flatten(offset_target)) > max_distance {
        return Err(DubinsError::NoPath);
    }

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

        TargetInfo::from_target(distances, target, path)
    } else {
        TargetInfo::from(distances, path)
    })
}

pub fn can_reach_target(car: &Car, max_speed: f32, max_time: f32, distance_remaining: f32, is_forwards: bool) -> Result<f32, ()> {
    let mut d = distance_remaining;
    let mut t_r = max_time;
    let mut b = car.boost as f32;
    let mut v = car.local_velocity.x;

    let direction = is_forwards as u8 as f32;

    loop {
        if d <= 0. {
            return Ok(t_r);
        }

        let r = d * direction / t_r.max(0.);

        if t_r < -SIMULATION_DT || (is_forwards && r > max_speed) || (!is_forwards && MIN_SPEED > r) {
            return Err(());
        }

        let t = r - v;

        if t.abs() < 100. {
            break;
        }

        let acceleration = t / REACTION_TIME;

        let throttle_accel = throttle_acceleration(v);
        let throttle_boost_transition = throttle_accel + 0.5 * BOOST_ACCEL;

        let throttle: f32;

        if acceleration <= BRAKE_COAST_TRANSITION {
            throttle = -1.;
        } else if BRAKE_COAST_TRANSITION < acceleration && acceleration < COASTING_THROTTLE_TRANSITION {
            throttle = 0.;
        } else if COASTING_THROTTLE_TRANSITION <= acceleration && acceleration <= throttle_boost_transition {
            throttle = if throttle_accel == 0. { 1. } else { (acceleration / throttle_accel).clamp(0.02, 1.) };
        } else if b >= MIN_BOOST_CONSUMPTION && throttle_boost_transition < acceleration {
            throttle = 1.;

            if t > 0. {
                v += BOOST_ACCEL_DT;
                b -= BOOST_CONSUMPTION_DT;
            }
        } else {
            throttle = 0.;
        }

        if throttle == 0. {
            v += COAST_ACC * SIMULATION_DT;
        } else if throttle.signum() == v.signum() {
            v += throttle_accel * SIMULATION_DT * throttle;
        } else {
            v += BRAKE_ACC_DT.copysign(throttle);
        }

        t_r -= SIMULATION_DT;
        d -= (v * direction) * SIMULATION_DT;
    }

    Ok(t_r)
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
