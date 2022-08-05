use crate::{
    car::Car,
    ground::{angle_2d, shortest_path_in_validate, TargetInfo},
    pytypes::ShotType,
    utils::flatten,
};
use dubins_paths::{self, NoPathError, PosRot};
use glam::Vec3A;
use rl_ball_sym::simulation::ball::Ball;

#[derive(Clone, Copy, Debug, Default)]
pub struct Analyzer {
    max_speed: Option<f32>,
    max_turn_radius: Option<f32>,
    gravity: f32,
    may_ground_shot: bool,
    may_jump_shot: bool,
}

impl Analyzer {
    pub const fn new(max_speed: Option<f32>, max_turn_radius: Option<f32>, gravity: f32, may_ground_shot: bool, may_jump_shot: bool) -> Self {
        Self {
            max_speed,
            max_turn_radius,
            gravity,
            may_ground_shot,
            may_jump_shot,
        }
    }

    fn get_max_speed(&self, car: &Car, slice_num: usize) -> f32 {
        self.max_speed.unwrap_or_else(|| car.max_speed[slice_num])
    }

    fn get_max_turn_radius(&self, car: &Car, slice_num: usize) -> f32 {
        self.max_turn_radius.unwrap_or_else(|| car.ctrms[slice_num])
    }

    pub fn target(&self, ball: &Ball, car: &Car, shot_vector: Vec3A, time_remaining: f32, slice_num: usize) -> dubins_paths::Result<TargetInfo> {
        let offset_target = ball.location - (shot_vector * ball.radius);
        let shot_vector = flatten(shot_vector).try_normalize().unwrap();
        let car_front_length = (car.hitbox_offset.x + car.hitbox.length) / 2.;

        if car.landing_time >= time_remaining {
            return Err(NoPathError);
        }

        let time_remaining = time_remaining - car.landing_time;
        let car_location = car.landing_location;
        let max_distance = time_remaining * self.get_max_speed(car, slice_num) + car_front_length;

        if flatten(car_location).distance(flatten(offset_target)) > max_distance {
            return Err(NoPathError);
        }

        let shot_type = if offset_target.z < car.hitbox.height / 2. + 17. {
            match self.may_ground_shot {
                true => Ok(ShotType::GROUND),
                false => Err(NoPathError),
            }
        } else if offset_target.z < car.max_jump_height {
            match self.may_jump_shot {
                true => Ok(ShotType::JUMP),
                false => Err(NoPathError),
            }
        } else {
            Err(NoPathError)
        }?;

        let jump_time = match shot_type {
            ShotType::GROUND => None,
            ShotType::JUMP => Some(car.jump_time_to_height(self.gravity, offset_target.z - car.hitbox.height / 2.)),
            _ => unreachable!(),
        };

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
                let time = car.jump_time_to_height(self.gravity, offset_target.z - car.hitbox.height / 2.);

                time * self.get_max_speed(car, slice_num) + 128.
            }
            _ => unreachable!(),
        };

        let offset_distance = end_distance - car_front_length;
        let exit_turn_target = flatten(offset_target) - (shot_vector * end_distance);

        if !car.field.is_point_in(&PosRot::from_f32(exit_turn_target.x, exit_turn_target.y, 0.))
            || flatten(car_location).distance(exit_turn_target) + end_distance > max_distance
        {
            return Err(NoPathError);
        }

        let target_angle = shot_vector.y.atan2(shot_vector.x);

        let q0 = PosRot::from_f32(car_location.x, car_location.y, car.landing_yaw);
        let q1 = PosRot::from_f32(exit_turn_target.x, exit_turn_target.y, target_angle);

        let path = shortest_path_in_validate(q0, q1, self.get_max_turn_radius(car, slice_num), &car.field, max_distance)?;

        let distances = [path.segment_length(0), path.segment_length(1), path.segment_length(2), offset_distance];

        Ok(TargetInfo::from(distances, shot_type, path, jump_time))
    }
}
