use dubins_paths::{DubinsError, DubinsResult};
use glam::Vec3A;
use rl_ball_sym::simulation::ball::Ball;

use crate::{
    car::Car,
    ground::{angle_2d, shortest_path_in_validate, TargetInfo},
    pytypes::ShotType,
    utils::flatten,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct Analyzer {
    max_speed: Option<f32>,
    max_turn_radius: Option<f32>,
    gravity: f32,
}

impl Analyzer {
    pub const fn new(max_speed: Option<f32>, max_turn_radius: Option<f32>, gravity: f32) -> Self {
        Self {
            max_speed,
            max_turn_radius,
            gravity,
        }
    }

    fn get_max_speed(&self, car: &Car, slice_num: usize) -> f32 {
        self.max_speed.unwrap_or_else(|| car.max_speed[slice_num])
    }

    fn get_max_turn_radius(&self, car: &Car, slice_num: usize) -> f32 {
        self.max_turn_radius.unwrap_or_else(|| car.ctrms[slice_num])
    }

    pub fn target(&self, ball: &Ball, car: &Car, shot_vector: Vec3A, time_remaining: f32, slice_num: usize) -> DubinsResult<TargetInfo> {
        let offset_target = ball.location - (shot_vector * ball.radius);
        let shot_vector = flatten(shot_vector).try_normalize().unwrap();
        let car_front_length = (car.hitbox_offset.x + car.hitbox.length) / 2.;

        if car.landing_time >= time_remaining {
            return Err(DubinsError::NoPath);
        }

        let time_remaining = time_remaining - car.landing_time;
        let car_location = car.landing_location;
        let max_distance = time_remaining * self.get_max_speed(car, slice_num) + car_front_length;

        if flatten(car_location).distance(flatten(offset_target)) > max_distance {
            return Err(DubinsError::NoPath);
        }

        let base_height = car.hitbox.height / 2. + 17.;
        let shot_type = if offset_target.z < base_height {
            Ok(ShotType::GROUND)
        } else if offset_target.z < base_height + car.max_jump_height {
            Ok(ShotType::JUMP)
        } else {
            Err(DubinsError::NoPath)
        }?;

        let end_distance = match shot_type {
            ShotType::GROUND => {
                let distance = 320.;
                if (0_f32..distance).contains(&car.forward.dot(ball.location))
                    && car.right.dot(ball.location) < car.hitbox.width / 2.
                    && angle_2d(car.forward, shot_vector) < 0.02
                {
                    0. // for pre-aligned ground shots
                } else {
                    distance // for non-aligned ground shots
                }
            }
            ShotType::JUMP => {
                let time = car.jump_time_to_height(self.gravity, offset_target.z - base_height);

                time * self.get_max_speed(car, slice_num) + 128.
            }
            _ => unreachable!(),
        };

        let offset_distance = end_distance - car_front_length;
        let exit_turn_target = flatten(offset_target) - (shot_vector * end_distance);

        if !car.field.is_point_in(&[exit_turn_target.x, exit_turn_target.y, 0.]) || flatten(car_location).distance(exit_turn_target) + end_distance > max_distance {
            return Err(DubinsError::NoPath);
        }

        let target_angle = shot_vector.y.atan2(shot_vector.x);

        let q0 = [car_location.x, car_location.y, car.yaw];
        let q1 = [exit_turn_target.x, exit_turn_target.y, target_angle];

        let path = shortest_path_in_validate(q0, q1, self.get_max_turn_radius(car, slice_num), &car.field, max_distance)?;

        let distances = [path.segment_length(0), path.segment_length(1), path.segment_length(2), offset_distance];

        Ok(TargetInfo::from(distances, shot_type, path))
    }
}
