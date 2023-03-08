use crate::{
    air::{aerial_shot_is_viable, AerialTargetInfo},
    car::Car,
    ground::{angle_2d, get_turn_exit_tanget, shortest_path_in_validate, GroundTargetInfo},
    pytypes::ShotType,
    utils::flatten,
    Mutators,
};
use dubins_paths::{mod2pi, DubinsPath, NoPathError, PathType, PosRot, Result as DubinsResult};
use glam::Vec3A;
use rl_ball_sym::simulation::ball::Ball;
use std::f32::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Shot {
    Ground,
    Jump,
    DoubleJump,
    Aerial,
}

#[derive(Clone, Copy, Debug)]
pub struct Analyzer<'a> {
    max_speed: Option<f32>,
    max_turn_radius: Option<f32>,
    gravity: Vec3A,
    may: [bool; 4],
    pub car: &'a Car,
}

impl<'a> Analyzer<'a> {
    #[inline]
    pub const fn new((max_speed, max_turn_radius): (Option<f32>, Option<f32>), gravity: Vec3A, may: [bool; 4], car: &'a Car) -> Self {
        Self {
            max_speed,
            max_turn_radius,
            gravity,
            may,
            car,
        }
    }

    #[inline]
    fn may_shoot(&self, shot: Shot) -> bool {
        match shot {
            Shot::Ground => self.may[0],
            Shot::Jump => self.may[1],
            Shot::DoubleJump => self.may[2],
            Shot::Aerial => self.may[3],
        }
    }

    #[inline]
    fn get_max_speed(&self, slice_num: usize) -> f32 {
        self.max_speed.unwrap_or_else(|| self.car.max_speed[slice_num])
    }

    #[inline]
    fn get_max_turn_radius(&self, slice_num: usize) -> f32 {
        self.max_turn_radius.unwrap_or_else(|| self.car.ctrms[slice_num])
    }

    /// get the type of shot that will be required to hit the ball
    /// also check if that type of shot has been enabled
    pub fn get_shot_type(&self, target: Vec3A, time_remaining: f32) -> DubinsResult<ShotType> {
        if self.car.airborne && time_remaining < self.car.time_to_land {
            if self.may_shoot(Shot::Aerial) {
                return Ok(ShotType::Aerial);
            }
        } else if target.z < self.car.hitbox.height / 2. + 17. {
            if self.may_shoot(Shot::Ground) {
                return Ok(ShotType::Ground);
            }
        } else if target.z < self.car.max_jump_height {
            if self.may_shoot(Shot::Jump) {
                return Ok(ShotType::Jump);
            }
        } else if target.z < self.car.max_double_jump_height && self.may_shoot(Shot::DoubleJump) {
            return Ok(ShotType::DoubleJump);
        }

        if self.car.airborne || self.car.wait_to_jump_time + self.car.last_landing_time < time_remaining && self.may_shoot(Shot::Aerial) {
            return Ok(ShotType::Aerial);
        }

        Err(NoPathError)
    }

    fn get_jump_info(
        &self,
        ball_location: Vec3A,
        target: Vec3A,
        shot_vector: Vec3A,
        max_speed: f32,
        time_remaining: f32,
        shot_type: ShotType,
    ) -> DubinsResult<(Option<f32>, f32)> {
        Ok(match shot_type {
            ShotType::Ground => {
                let distance = 320.;
                (
                    None,
                    if (0. ..distance).contains(&self.car.forward.dot(ball_location))
                        && self.car.right.dot(ball_location) < self.car.hitbox.width / 2.
                        && angle_2d(self.car.forward, shot_vector) < 0.02
                    {
                        0. // for pre-aligned ground shots
                    } else {
                        distance // for non-aligned ground shots
                    },
                )
            }
            ShotType::Jump => {
                let time = self.car.jump_time_to_height(self.gravity.z, target.z - self.car.hitbox.height / 2.);

                (Some(time), time * max_speed + 128.)
            }
            ShotType::DoubleJump => {
                // if we need to do a double jump but we don't even have time for a normal jump
                if time_remaining < self.car.max_jump_time {
                    return Err(NoPathError);
                }

                let time = self.car.double_jump_time_to_height(self.gravity.z, target.z - self.car.hitbox.height / 2.);

                (Some(time), time * max_speed + 128.)
            }
            ShotType::Aerial => unreachable!(),
        })
    }

    #[inline]
    fn should_travel_forwards(&self, time_remaining: f32, shot_vector: Vec3A) -> bool {
        // it's easier for me to think about what I want the criteria to be for going backwards, so I did that then just took the opposite of it for is_forwards
        let is_backwards = time_remaining < 4. && angle_2d(shot_vector, Vec3A::new(self.car.landing_yaw.cos(), self.car.landing_yaw.sin(), 0.)) > PI * (2. / 3.);
        !is_backwards
    }

    pub fn no_target(&self, ball: &Ball, mut time_remaining: f32, slice_num: usize, shot_type: ShotType) -> DubinsResult<GroundTargetInfo> {
        let car_front_length = (self.car.hitbox_offset.x + self.car.hitbox.length) / 2.;

        let max_speed = self.get_max_speed(slice_num);

        time_remaining -= self.car.time_to_land;
        assert!(time_remaining > 0.);
        let car_location = flatten(self.car.landing_location);
        let max_distance = time_remaining * max_speed + car_front_length + ball.radius();

        // check if a simplified path is longer than the longest distance we can possibly travel
        if car_location.distance(flatten(ball.location)) > max_distance {
            return Err(NoPathError);
        }

        let car_to_ball = (ball.location - self.car.location).normalize_or_zero();

        let (jump_time, end_distance) = if shot_type == ShotType::Ground {
            (None, 0.)
        } else {
            self.get_jump_info(ball.location, ball.location, car_to_ball, max_speed, time_remaining, shot_type)?
        };

        if let Some(jump_time) = jump_time {
            // if we have enough time for just the jump
            if jump_time > time_remaining {
                return Err(NoPathError);
            }
        }

        let rho = self.get_max_turn_radius(slice_num);
        let is_forwards = self.should_travel_forwards(time_remaining, car_to_ball);
        let local_ball = self.car.localize_2d_location(ball.location);
        let target_is_forwards = local_ball.x >= 0.;
        let should_turn_left = local_ball.y < 0.;
        let center_of_turn = car_location + flatten(if should_turn_left { -self.car.landing_right } else { self.car.landing_right } * rho);

        let (turn_target, turn_target_2) = get_turn_exit_tanget(self.car, flatten(ball.location), center_of_turn, rho, target_is_forwards, is_forwards);

        // check if the exit point is in the field
        if !self.car.field.is_point_in(turn_target) {
            return Err(NoPathError);
        }

        // compute the distance of each path, validating that it is within our current maximum travel distance (returning an error if neither are)

        let turn_final_distance = turn_target.distance(ball.location) - ball.radius() - car_front_length;
        let offset_distance = end_distance - car_front_length - ball.radius();

        if turn_final_distance < offset_distance || turn_final_distance + turn_target.distance(car_location) > max_distance {
            return Err(NoPathError);
        }

        let shot_vector = (flatten(ball.location) - turn_target).normalize_or_zero();
        let shot_vector_angle = shot_vector.y.atan2(shot_vector.x);
        let forward_angle = if is_forwards {
            self.car.landing_forward.y.atan2(self.car.landing_forward.x)
        } else {
            let forward = self.car.landing_forward * Vec3A::NEG_ONE;
            forward.y.atan2(forward.x)
        };

        let direction_turn_left = (is_forwards && should_turn_left) || (!is_forwards && !should_turn_left);

        // find the angle between the car location and each turn exit point relative to the exit turn point centers
        let turn_angle = mod2pi(if direction_turn_left {
            forward_angle - shot_vector_angle
        } else {
            shot_vector_angle - forward_angle
        });

        let turn_arc_distance = turn_angle * rho;

        if turn_final_distance + turn_arc_distance > max_distance {
            return Err(NoPathError);
        }

        let enter_yaw = if is_forwards { self.car.landing_yaw } else { mod2pi(self.car.landing_yaw + PI) };

        // construct a path so we can easily follow our defined turn arc
        let path = DubinsPath {
            qi: PosRot::new(car_location, enter_yaw),
            rho,
            type_: if direction_turn_left { PathType::RSL } else { PathType::LSR },
            param: [turn_angle, 0., 0.],
        };

        let distances = [turn_arc_distance, 0., 0., turn_final_distance];

        Ok(GroundTargetInfo {
            distances,
            shot_type,
            path,
            jump_time,
            is_forwards,
            shot_vector,
            turn_targets: Some((turn_target, turn_target_2)),
            wait_for_land: self.car.airborne,
        })
    }

    pub fn target(&self, ball: &Ball, shot_vector: Vec3A, mut time_remaining: f32, slice_num: usize, shot_type: ShotType) -> DubinsResult<GroundTargetInfo> {
        let offset_target = ball.location - (shot_vector * ball.radius());
        let car_front_length = (self.car.hitbox_offset.x + self.car.hitbox.length) / 2.;

        let max_speed = self.get_max_speed(slice_num);

        time_remaining -= self.car.time_to_land;
        assert!(time_remaining > 0.);
        let car_location = self.car.landing_location;
        let max_distance = time_remaining * max_speed + car_front_length;

        // check if a simplified path is longer than the longest distance we can possibly travel
        if flatten(car_location).distance(flatten(offset_target)) > max_distance {
            return Err(NoPathError);
        }
        let (jump_time, end_distance) = self.get_jump_info(ball.location, offset_target, shot_vector, max_speed, time_remaining, shot_type)?;

        if let Some(jump_time) = jump_time {
            // if we have enough time for just the jump
            if jump_time > time_remaining {
                return Err(NoPathError);
            }
        }

        let exit_turn_target = flatten(offset_target) - (flatten(shot_vector).normalize_or_zero() * end_distance);

        // check if the exit point is in the field, and make sure a simplified version of the path isn't longer than the longest distance we can travel
        if !self.car.field.is_point_in(flatten(exit_turn_target)) || flatten(car_location).distance(exit_turn_target) + end_distance > max_distance {
            return Err(NoPathError);
        }

        // calculate and return the dubin's path

        let target_angle = shot_vector.y.atan2(shot_vector.x);
        let mut starting_yaw = self.car.landing_yaw;

        let is_forwards = self.should_travel_forwards(time_remaining, shot_vector);

        if !is_forwards {
            starting_yaw += PI;
        }

        let q0 = PosRot::new(flatten(car_location), starting_yaw);
        let q1 = PosRot::new(flatten(exit_turn_target), target_angle);

        let path = shortest_path_in_validate(q0, q1, self.get_max_turn_radius(slice_num), &self.car.field, max_distance)?;

        let offset_distance = end_distance - car_front_length;
        let distances = [path.segment_length(0), path.segment_length(1), path.segment_length(2), offset_distance];

        Ok(GroundTargetInfo {
            distances,
            shot_type,
            path,
            jump_time,
            is_forwards,
            shot_vector,
            turn_targets: None,
            wait_for_land: self.car.airborne,
        })
    }

    #[inline]
    pub fn aerial_shot(
        &self,
        mutators: Mutators,
        target: Vec3A,
        shot_vector: Vec3A,
        time_remaining: f32,
        check_target_angle: Option<Vec3A>,
    ) -> DubinsResult<AerialTargetInfo> {
        aerial_shot_is_viable(self.car, mutators, self.gravity, target, shot_vector, time_remaining, check_target_angle)
    }
}
