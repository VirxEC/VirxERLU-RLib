use std::f32::INFINITY;

use dubins_paths::{DubinsError, DubinsPath, DubinsPathType, DubinsIntermediateResults};
use glam::Vec3A;
use rl_ball_sym::simulation::ball::Ball;

use crate::car::{throttle_acceleration, turn_radius, Car, CarFieldRect};
use crate::constants::*;
use crate::utils::*;

fn angle_2d(vec1: Vec3A, vec2: Vec3A) -> f32 {
    flatten(vec1).dot(flatten(vec2)).clamp(-1., 1.).acos()
}

// https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
fn point_from_line(v: Vec3A, w: Vec3A, p: Vec3A) -> (f32, f32) {
    // Return minimum distance between line segment vw and point p
    let l2 = v.distance_squared(w); // i.e. |w-v|^2 -  avoid a sqrt

    // v == w case
    if l2 == 0. {
        return (p.distance(v), 1.);
    }

    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    // We clamp t from [0,1] to handle points outside the segment vw.
    let t = ((p - v).dot(w - v) / l2).clamp(0., 1.);
    let projection = lerp(w, v, t); // Projection falls on the segment

    (p.distance(projection), t)
}

fn path_point_to_vec3(endpoint: [f32; 3]) -> Vec3A {
    Vec3A::new(endpoint[0], endpoint[1], 0.)
}

fn shortest_path_in_validate(q0: [f32; 3], q1: [f32; 3], rho: f32, car_field: &CarFieldRect, max_distance: f32, validate: bool) -> Result<DubinsPath, DubinsError> {
    let mut best_cost = INFINITY;
    let mut best_path = None;

    let intermediate_results = DubinsIntermediateResults::from(q0, q1, rho)?;

    for path_type in DubinsPathType::ALL {
        if let Ok(param) = intermediate_results.word(path_type) {
            let cost = param[0] + param[1] + param[2];
            if cost < best_cost && (!validate || cost * rho <= max_distance) {
                let path = DubinsPath {
                    qi: q0,
                    rho,
                    param,
                    type_: path_type,
                };

                let mut valid = true;

                // instead of this, do a better "is arc in" or "is line in" thing
                for dist in [path.param[0] * path.rho / 2., (path.param[0] + path.param[1]) * path.rho / 2., (path.param[0] + path.param[1] + path.param[2]) * path.rho / 2.] {
                    if !car_field.is_point_in(&path.sample(dist)?) {
                        valid = false;
                        break;
                    }
                }

                if !valid {
                    continue;
                }

                for dist in [path.param[0] * path.rho, (path.param[0] + path.param[1]) * path.rho, (path.param[0] + path.param[1] + path.param[2]) * path.rho] {
                    if !car_field.is_point_in(&path.sample(dist)?) {
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

#[derive(Clone, Copy, Debug, Default)]
pub struct TargetInfo {
    pub distances: [f32; 4],
    pub target: Option<Vec3A>,
    pub path: Option<DubinsPath>,
}

impl TargetInfo {
    pub const fn new_dubin(distances: [f32; 4], path: DubinsPath) -> Self {
        Self {
            distances,
            target: None,
            path: Some(path),
        }
    }
    pub const fn new_dubin_target(distances: [f32; 4], target: Vec3A, path: DubinsPath) -> Self {
        Self {
            distances,
            target: Some(target),
            path: Some(path),
        }
    }

    pub const fn new(distances: [f32; 4]) -> Self {
        Self {
            distances,
            target: None,
            path: None,
        }
    }

    pub const fn new_target(distances: [f32; 4], target: Vec3A) -> Self {
        Self {
            distances,
            target: Some(target),
            path: None,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AnalyzeOptions {
    pub max_speed: f32,
    pub max_turn_radius: f32,
    pub get_target: bool,
    pub validate: bool,
}

pub fn analyze_target(ball: &Ball, car: &Car, shot_vector: Vec3A, time_remaining: f32, options: AnalyzeOptions) -> Result<TargetInfo, DubinsError> {
    let offset_target = ball.location - (shot_vector * ball.radius);
    let car_front_length = (car.hitbox_offset.x + car.hitbox.length) / 2.;

    let mut end_distance = -car_front_length;

    let offset_distance = 640.;
    let exit_turn_target = offset_target - (shot_vector * offset_distance);

    let local_offset = car.localize_2d_location(offset_target);

    let margin = car.hitbox.width / 3.;

    let target_angle = shot_vector.y.atan2(shot_vector.x);

    {
        // all this stuff has the potential ability to drive straight through a wall
        // ignoring what ever the dubin's pathing has told it to not do
        // FIX IT
        // also fix dubin's pathing field check at the same time to be more accurate

        let car_to_shot = angle_2d(car.forward, shot_vector);

        if local_offset.x > 0. && car_to_shot < 0.15 && local_offset.y.abs() < margin / 2. {
            end_distance += local_offset.x;

            let distances = [0., 0., 0., end_distance];

            return Ok(if options.get_target {
                let final_target = car.location + flatten(shot_vector) * local_offset.x;

                TargetInfo::new_target(distances, final_target)
            } else {
                TargetInfo::new(distances)
            });
        }

        // https://www.desmos.com/calculator/e0gk9v6ab0
        let car_to_exit = flatten(exit_turn_target - car.location);
        let turn_direction = car.right.dot(car_to_exit).signum();
        let n = turn_direction * Vec3A::new(-car.forward.y, car.forward.x, 0.);
        let exit_turn_dir = flatten(car_to_shot.sin() * car.forward + (1. - car_to_shot.cos()) * n);

        let turn_radii = [turn_radius(car.local_velocity.x.abs()), options.max_turn_radius];
        let exit_points = [turn_radii[0] * exit_turn_dir, turn_radii[1] * exit_turn_dir];

        let (minimum_distance, t) = point_from_line(exit_points[0], exit_points[1], car_to_exit);
        let turn_rad = lerp(turn_radii[0], turn_radii[1], t);

        // if options.get_target {
        //     dbg!(margin / 2.);
        //     dbg!(minimum_distance);
        //     dbg!(car.forward.dot(car_to_exit) - car.forward.dot(exit_points[1]));
        //     dbg!(car.right.dot(car_to_exit) - car.right.dot(exit_points[1]));
        // }

        if minimum_distance < margin / 2. {
            end_distance += offset_distance;

            let turn_distance = turn_rad * car_to_shot;
            let distances = [0., 0., turn_distance, end_distance];

            return Ok(if options.get_target {
                let distance = car.local_velocity.x.max(500.) * STEER_REACTION_TIME;

                let target = if distance > turn_distance {
                    let distance_remaining = distance - turn_distance;
                    let additional_space = shot_vector * distance_remaining.min(offset_distance - car_front_length);

                    exit_turn_target + additional_space
                } else {
                    let t = turn_distance / distance;
                    let angle = car_to_shot * t;

                    let partial_target_dir = flatten(angle.sin() * car.forward + (1. - angle.cos()) * n);

                    options.max_turn_radius * partial_target_dir + flatten(car.location)
                };

                TargetInfo::new_target(distances, target)
            } else {
                TargetInfo::new(distances)
            });
        } else {
            let right_distance = car.right.dot(car_to_exit) - car.right.dot(exit_points[1]);
            if right_distance.abs() < margin / 2. {
                let forward_distance = car.forward.dot(car_to_exit) - car.forward.dot(exit_points[1]);
                if forward_distance > -margin / 2. {
                    end_distance += offset_distance;
                    let turn_distance = options.max_turn_radius * car_to_shot;
                    let distances = [right_distance, forward_distance, turn_distance, end_distance];

                    return Ok(if options.get_target {
                        let right_adjustment = car.right * right_distance;
                        let forward_travel = car.forward * forward_distance;

                        let target = car.location + right_adjustment + forward_travel;

                        TargetInfo::new_target(distances, target)
                    } else {
                        TargetInfo::new(distances)
                    });
                }
            }
        }
    }

    end_distance += offset_distance;

    let max_distance = time_remaining * options.max_speed;

    if options.validate && (end_distance + flatten(car.location).distance(flatten(exit_turn_target))) > max_distance {
        return Err(DubinsError::NoPath);
    }

    let q0 = [car.location.x, car.location.y, car.yaw];
    let q1 = [exit_turn_target.x, exit_turn_target.y, target_angle];

    let path = shortest_path_in_validate(q0, q1, options.max_turn_radius, &car.field, max_distance, options.validate)?;

    let distances = [path.segment_length(0), path.segment_length(1), path.segment_length(2), end_distance];

    Ok(if options.get_target {
        let distance = car.local_velocity.x.max(500.) * STEER_REACTION_TIME;
        let max_path_distance = path.length();

        let target = if distance > max_path_distance {
            let distance_remaining = distance - max_path_distance;
            let additional_space = shot_vector * distance_remaining.min(offset_distance - car_front_length);

            path_point_to_vec3(path.sample(max_path_distance)?) + additional_space
        } else {
            path_point_to_vec3(path.sample(distance)?)
        };

        TargetInfo::new_dubin_target(distances, target, path)
    } else {
        TargetInfo::new_dubin(distances, path)
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
            throttle = if throttle_accel == 0. {
                1.
            } else {
                (acceleration / throttle_accel).clamp(0.02, 1.)
            };
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
