#![forbid(unsafe_code)]

mod air;
mod analyzer;
mod car;
mod constants;
mod ground;
mod pytypes;
mod shot;
mod utils;

use analyzer::*;
use car::{turn_radius, Car};
use combo_vec::{rearr, ReArr};
use constants::*;
use glam::Vec3A;
use pyo3::prelude::*;
use pytypes::*;
use rl_ball_sym::simulation::{
    ball::{Ball, BallPrediction},
    game::Game,
};
use shot::{AirBasedShot, GroundBasedShot, Options, Shot, Target};
use std::sync::RwLock;
use utils::*;

static CARS: RwLock<ReArr<Car, 8>> = RwLock::new(rearr![]);
static BALL_STRUCT: RwLock<BallPrediction> = RwLock::new(BallPrediction::new());
static GRAVITY: RwLock<Vec3A> = RwLock::new(Vec3A::ZERO);
static GAME_TIME: RwLock<f32> = RwLock::new(0.);
static GAME: RwLock<Option<Game>> = RwLock::new(None);
static BALL: RwLock<Option<Ball>> = RwLock::new(None);
static MUTATORS: RwLock<Mutators> = RwLock::new(Mutators::new());
static TARGETS: RwLock<ReArr<Option<Target>, 16>> = RwLock::new(rearr![]);

macro_rules! pynamedmodule {
    (doc: $doc:literal, name: $name:tt, funcs: [$($func_name:path),*], classes: [$($class_name:ident),*]) => {
        #[doc = $doc]
        #[pymodule]
        fn $name(_py: Python, m: &PyModule) -> PyResult<()> {
            $(m.add_function(wrap_pyfunction!($func_name, m)?)?);*;
            $(m.add_class::<$class_name>()?);*;
            Ok(())
        }
    };
}

pynamedmodule!(
    doc: "VirxERLU-RLib is written in Rust with Python bindings to make analyzing the ball prediction struct much faster.",
    name: virx_erlu_rlib,
    funcs: [load_soccer, load_soccar, load_dropshot, load_hoops, load_soccer_throwback, load_soccar_throwback,
    tick, get_slice, get_slice_index, get_num_ball_slices, set_mutator_settings,
    new_target, new_any_target, confirm_target, remove_target, print_targets, get_targets_length,
    get_shot_with_target, get_data_for_shot_with_target],
    classes: [TargetOptions, ShotType]
);

#[pyfunction]
fn load_soccar() {
    let (game, ball) = rl_ball_sym::compressed::load_soccar();

    *GAME.write().unwrap() = Some(game);
    *BALL.write().unwrap() = Some(ball);
}

#[pyfunction]
fn load_soccer() {
    load_soccar();
}

#[pyfunction]
fn load_dropshot() {
    let (game, ball) = rl_ball_sym::compressed::load_dropshot();

    *GAME.write().unwrap() = Some(game);
    *BALL.write().unwrap() = Some(ball);
}

#[pyfunction]
fn load_hoops() {
    let (game, ball) = rl_ball_sym::compressed::load_hoops();

    *GAME.write().unwrap() = Some(game);
    *BALL.write().unwrap() = Some(ball);
}

#[pyfunction]
fn load_soccar_throwback() {
    let (game, ball) = rl_ball_sym::compressed::load_soccar_throwback();

    *GAME.write().unwrap() = Some(game);
    *BALL.write().unwrap() = Some(ball);
}

#[pyfunction]
fn load_soccer_throwback() {
    load_soccar_throwback();
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoostAmount {
    Default,
    Unlimited,
    SlowRecharge,
    FastRecharge,
    NoBoost,
}

impl BoostAmount {
    #[inline]
    fn from(item: u8) -> BoostAmount {
        match item {
            1 => BoostAmount::Unlimited,
            2 => BoostAmount::SlowRecharge,
            3 => BoostAmount::FastRecharge,
            4 => BoostAmount::NoBoost,
            _ => BoostAmount::Default,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Mutators {
    boost_amount: BoostAmount,
    boost_accel: f32,
}

impl Mutators {
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Mutators {
            boost_amount: BoostAmount::Default,
            boost_accel: BOOST_ACCEL,
        }
    }

    #[inline]
    pub fn from(mutators: &PyAny) -> PyResult<Self> {
        Ok(Mutators {
            boost_amount: BoostAmount::from(mutators.call_method("BoostOption", (), None)?.extract()?),
            boost_accel: match mutators.call_method("BoostStrengthOption", (), None)?.extract()? {
                1 => BOOST_ACCEL * 1.5,
                2 => BOOST_ACCEL * 2.,
                3 => BOOST_ACCEL * 10.,
                _ => BOOST_ACCEL,
            },
        })
    }
}

#[pyfunction]
fn set_mutator_settings(py: Python, mutators: PyObject) -> PyResult<()> {
    *MUTATORS.write().unwrap() = Mutators::from(mutators.as_ref(py))?;

    Ok(())
}

#[pyfunction]
fn tick(py: Python, packet: PyObject, prediction_time: Option<f32>) -> PyResult<()> {
    TARGETS.write().unwrap().iter_mut().for_each(|target | {
        if matches!(target, Some(t) if !t.is_confirmed()) {
            *target = None;
        }
    });

    let mut game_guard = GAME.write().unwrap();
    let game = game_guard.as_mut().ok_or_else(|| PyErr::new::<NoGamePyErr, _>(NO_GAME_ERR))?;

    let mut ball = BALL.read().unwrap().ok_or_else(|| PyErr::new::<NoBallPyErr, _>(NO_BALL_ERR))?;

    let py_packet = packet.as_ref(py);
    let packet = py_packet.extract::<GamePacket>()?;

    // Get general game information
    *GAME_TIME.write().unwrap() = packet.game_info.seconds_elapsed;
    game.gravity.z = packet.game_info.world_gravity_z;
    *GRAVITY.write().unwrap() = game.gravity;

    // Get information about the ball
    ball.update(
        packet.game_info.seconds_elapsed,
        packet.game_ball.physics.location.into(),
        packet.game_ball.physics.velocity.into(),
        packet.game_ball.physics.angular_velocity.into(),
    );

    let radius = packet.game_ball.collision_shape.get_radius();

    // check if the new radius is different
    // if is is, set it
    if (ball.radius() - radius).abs() > 0.1 {
        ball.set_radius(radius, radius + 1.9);
    }

    // Predict future information about the ball
    *BALL_STRUCT.write().unwrap() = ball.get_ball_prediction_struct_for_time(game, prediction_time.unwrap_or(6.));

    // Get information about the cars on the field
    let mut cars = CARS.write().unwrap();

    if cars.len() != packet.num_cars {
        const NEW_CAR: Car = Car::new();
        cars.resize(packet.num_cars, NEW_CAR);
    }

    let py_game_cars = py_packet.getattr("game_cars")?;
    for (i, car) in cars.iter_mut().enumerate() {
        car.update(py_game_cars.get_item(i)?.extract()?, packet.game_info.seconds_elapsed);
    }

    Ok(())
}

#[pyfunction]
fn get_slice(slice_time: f32) -> BallSlice {
    let slice_num = ((slice_time - *GAME_TIME.read().unwrap()) * TPS).round() as usize;
    get_slice_index(slice_num)
}

#[pyfunction]
fn get_slice_index(slice_num: usize) -> BallSlice {
    let ball_struct = BALL_STRUCT.read().unwrap();
    let ball = ball_struct[slice_num.clamp(0, ball_struct.len() - 1)];

    BallSlice::from(ball)
}

#[pyfunction]
fn get_num_ball_slices() -> usize {
    BALL_STRUCT.read().unwrap().len()
}

#[pyfunction]
fn new_target(left_target: Vec<f32>, right_target: Vec<f32>, car_index: usize, options: Option<TargetOptions>) -> PyResult<usize> {
    let num_slices = BALL_STRUCT.read().unwrap().len();

    if num_slices == 0 {
        return Err(PyErr::new::<NoSlicesPyErr, _>(NO_SLICES_ERR));
    }

    let target_left = get_vec3_from_vec(&left_target, "target_left")?;
    let target_right = get_vec3_from_vec(&right_target, "target_right")?;
    let options = Options::from(options, num_slices);

    {
        let mut cars = CARS.write().unwrap();
        let car = cars.get_mut(car_index).ok_or_else(|| PyErr::new::<NoCarPyErr, _>(NO_CAR_ERR))?;
        car.init(GRAVITY.read().unwrap().z, num_slices, *MUTATORS.read().unwrap());
    }

    let target = Some(Target::new(target_left, target_right, car_index, options));
    let mut targets = TARGETS.write().unwrap();

    let target_position = targets.iter().position(Option::is_none);
    let target_index = if let Some(i) = target_position {
        targets[i] = target;
        i
    } else {
        targets.push(target);
        targets.len() - 1
    };

    Ok(target_index)
}

#[pyfunction]
fn new_any_target(car_index: usize, options: Option<TargetOptions>) -> PyResult<usize> {
    let num_slices = BALL_STRUCT.read().unwrap().len();

    if num_slices == 0 {
        return Err(PyErr::new::<NoSlicesPyErr, _>(NO_SLICES_ERR));
    }

    let options = Options::from(options, num_slices);

    {
        let mut cars = CARS.write().unwrap();
        let car = cars.get_mut(car_index).ok_or_else(|| PyErr::new::<NoCarPyErr, _>(NO_CAR_ERR))?;
        car.init(GRAVITY.read().unwrap().z, num_slices, *MUTATORS.read().unwrap());
    }

    let target = Some(Target::new_any(car_index, options));
    let mut targets = TARGETS.write().unwrap();

    let target_position = targets.iter().position(Option::is_none);
    let target_index = if let Some(i) = target_position {
        targets[i] = target;
        i
    } else {
        targets.push(target);
        targets.len() - 1
    };

    Ok(target_index)
}

#[pyfunction]
fn confirm_target(target_index: usize) -> PyResult<()> {
    let mut targets = TARGETS.write().unwrap();
    let target = targets
        .get_mut(target_index)
        .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?
        .as_mut()
        .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?;

    if target.shot.is_none() {
        return Err(PyErr::new::<NoShotPyErr, _>(NO_SHOT_ERR));
    }

    target.confirm();
    Ok(())
}

#[pyfunction]
fn remove_target(target_index: usize) {
    let mut targets = TARGETS.write().unwrap();
    if targets.get(target_index).is_none() {
        return;
    }

    targets[target_index] = None;
}

#[pyfunction]
fn print_targets() {
    let targets = TARGETS.read().unwrap();
    let mut out = Vec::with_capacity(targets.len());

    for target in targets.iter() {
        match target {
            Some(t) => match &t.shot {
                Some(s) => out.push(format!("{}", s.time())),
                None => out.push(String::from("No shot")),
            },
            None => {
                out.push(String::from("None"));
            }
        }
    }

    println!("[{}]", out.join(", "));
}

#[pyfunction]
fn get_targets_length() -> usize {
    TARGETS.read().unwrap().len()
}

#[pyfunction]
fn get_shot_with_target(
    target_index: usize,
    temporary: Option<bool>,
    may_ground_shot: Option<bool>,
    may_jump_shot: Option<bool>,
    may_double_jump_shot: Option<bool>,
    may_aerial_shot: Option<bool>,
    only: Option<bool>,
) -> PyResult<BasicShotInfo> {
    let only = only.unwrap_or(false);
    let may_ground_shot = may_ground_shot.unwrap_or(!only);
    let may_jump_shot = may_jump_shot.unwrap_or(!only);
    let may_double_jump_shot = may_double_jump_shot.unwrap_or(!only);
    let may_aerial_shot = may_aerial_shot.unwrap_or(!only);
    let temporary = temporary.unwrap_or(false);

    if !may_ground_shot && !may_jump_shot && !may_double_jump_shot && !may_aerial_shot {
        return Err(PyErr::new::<NoShotSelectedPyErr, _>(NO_SHOT_SELECTED_ERR));
    }

    let mutators = *MUTATORS.read().unwrap();
    let gravity = *GRAVITY.read().unwrap();
    let game_time = *GAME_TIME.read().unwrap();
    let ball_prediction = BALL_STRUCT.read().unwrap();

    let mut found_shot = None;
    let mut basic_shot_info = None;

    {
        let targets_gaurd = TARGETS.read().unwrap();
        let target = targets_gaurd
            .get(target_index)
            .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?
            .as_ref()
            .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?;

        let cars = CARS.read().unwrap();
        let car = cars.get(target.car_index).ok_or_else(|| PyErr::new::<NoCarPyErr, _>(NO_CAR_ERR))?;

        if ball_prediction.is_empty() || car.demolished || car.landing_time >= ball_prediction.last().map(|slice| slice.time).unwrap_or_default() {
            return Ok(BasicShotInfo::not_found());
        }

        let analyzer = {
            let (max_speed, max_turn_radius) = if target.options.use_absolute_max_values {
                (Some(MAX_SPEED), Some(turn_radius(MAX_SPEED)))
            } else {
                (None, None)
            };

            Analyzer::new(
                (max_speed, max_turn_radius),
                gravity,
                may_ground_shot,
                may_jump_shot,
                may_double_jump_shot,
                may_aerial_shot,
                car,
            )
        };

        for (i, ball) in ball_prediction[target.options.min_slice..target.options.max_slice].iter().enumerate() {
            if ball.location.y.abs() > 5120. + ball.collision_radius() {
                break;
            }

            let max_time_remaining = ball.time - game_time;

            if let Some(target_location) = &target.location {
                let post_info = PostCorrection::from(ball.location, ball.collision_radius(), target_location.left, target_location.right);

                if !post_info.fits {
                    continue;
                }

                let shot_vector = post_info.get_shot_vector_target(car.landing_location, ball.location);

                let shot_type = match analyzer.get_shot_type(ball.location, max_time_remaining) {
                    Ok(st) => st,
                    Err(_) => continue,
                };

                if shot_type == ShotType::Aerial {
                    let ball_edge = ball.location - flatten(shot_vector) * ball.radius();
                    let target_location = ball_edge - Vec3A::new(0., 0., shot_vector.z) * (car.hitbox_offset.x + car.hitbox.length) / 2.;

                    let target_info = match analyzer.aerial_shot(mutators, target_location, shot_vector, max_time_remaining, Some(ball.location)) {
                        Ok(ti) => ti,
                        Err(_) => continue,
                    };

                    if found_shot.is_none() {
                        basic_shot_info = Some(target_info.get_basic_shot_info(ball.time));

                        found_shot = Some(if temporary { AirBasedShot::default() } else { AirBasedShot::from(ball, &target_info) }.into());

                        if !target.options.all {
                            break;
                        }
                    }

                    continue;
                }

                let target_info = match analyzer.target(ball, shot_vector, max_time_remaining, i, shot_type) {
                    Ok(ti) => ti,
                    Err(_) => continue,
                };

                if target_info.can_reach(car, max_time_remaining, mutators).is_err() {
                    continue;
                }

                if found_shot.is_none() {
                    basic_shot_info = Some(target_info.get_basic_shot_info(ball.time));

                    found_shot = Some(
                        if temporary {
                            GroundBasedShot::default()
                        } else {
                            GroundBasedShot::from(ball, &target_info)
                        }
                        .into(),
                    );

                    if !target.options.all {
                        break;
                    }
                }
            } else {
                let shot_type = match analyzer.get_shot_type(ball.location, max_time_remaining) {
                    Ok(st) => st,
                    Err(_) => continue,
                };

                if shot_type == ShotType::Aerial {
                    let ball_edge = ball.location - flatten(ball.location - car.location).normalize_or_zero() * ball.radius();
                    let shot_vector = (ball_edge - car.location).normalize_or_zero();
                    let target_location = ball_edge - shot_vector * (car.hitbox_offset.x + car.hitbox.length) / 2.;

                    let target_info = match analyzer.aerial_shot(mutators, target_location, shot_vector, max_time_remaining, None) {
                        Ok(ti) => ti,
                        Err(_) => continue,
                    };

                    if found_shot.is_none() {
                        basic_shot_info = Some(target_info.get_basic_shot_info(ball.time));

                        found_shot = Some(if temporary { AirBasedShot::default() } else { AirBasedShot::from(ball, &target_info) }.into());

                        if !target.options.all {
                            break;
                        }
                    }

                    continue;
                }

                let target_info = match analyzer.no_target(ball, max_time_remaining, i, shot_type) {
                    Ok(ti) => ti,
                    Err(_) => continue,
                };

                if target_info.can_reach(car, max_time_remaining, mutators).is_err() {
                    continue;
                }

                if found_shot.is_none() {
                    basic_shot_info = Some(target_info.get_basic_shot_info(ball.time));

                    found_shot = Some(
                        if temporary {
                            GroundBasedShot::default()
                        } else {
                            GroundBasedShot::from(ball, &target_info)
                        }
                        .into(),
                    );

                    if !target.options.all {
                        break;
                    }
                }
            };
        }
    }

    if !temporary {
        TARGETS
            .write()
            .unwrap()
            .get_mut(target_index)
            .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?
            .as_mut()
            .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?
            .shot = found_shot;
    }

    Ok(match basic_shot_info {
        Some(bsi) => bsi,
        None => BasicShotInfo::not_found(),
    })
}

#[pyfunction]
fn get_data_for_shot_with_target(target_index: usize) -> PyResult<AdvancedShotInfo> {
    let targets_gaurd = TARGETS.read().unwrap();
    let target = targets_gaurd
        .get(target_index)
        .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?
        .as_ref()
        .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?;
    let shot = target.shot.as_ref().ok_or_else(|| PyErr::new::<NoShotPyErr, _>(NO_SHOT_ERR))?;

    let time_remaining = shot.time() - *GAME_TIME.read().unwrap();

    if time_remaining < 0. {
        return Err(PyErr::new::<NoTimeRemainingPyErr, _>(NO_TIME_REMAINING_ERR));
    }

    let cars_guard = CARS.read().unwrap();
    let car = cars_guard.get(target.car_index).ok_or_else(|| PyErr::new::<NoCarPyErr, _>(NO_CAR_ERR))?;

    let ball_struct = BALL_STRUCT.read().unwrap();
    let slice_num = ((time_remaining * TPS).round() as usize).clamp(1, ball_struct.len()) - 1;
    let ball = ball_struct[slice_num];

    if ball.location.distance(shot.ball_location()) > car.hitbox.width {
        return Err(PyErr::new::<BallChangedPyErr, _>(BALL_CHANGED_ERR));
    }

    match shot {
        Shot::GroundBased(shot_details) => {
            let shot_info = AdvancedShotInfo::get_from_ground(car, shot_details).ok_or_else(|| PyErr::new::<StrayedFromPathPyErr, _>(STRAYED_FROM_PATH_ERR))?;

            if car.max_speed[slice_num] * (time_remaining + 0.1) >= shot_info.get_distance_remaining() {
                Ok(shot_info)
            } else {
                Err(PyErr::new::<BadAccelerationPyErr, _>(BAD_ACCELERATION_ERR))
            }
        }
        Shot::AirBased(shot_details) => {
            let shot_info = AdvancedShotInfo::get_from_air(car, shot_details);
            // Ok(shot_info)

            let gravity = *GRAVITY.read().unwrap();
            let vf_base = car.velocity + gravity * time_remaining;
            let xf_base = car.velocity * time_remaining + gravity * 0.5 * time_remaining.powi(2);

            let mutators = MUTATORS.read().unwrap();

            if air::partial_validate(
                shot_details.final_target,
                car.location + xf_base,
                vf_base,
                mutators.boost_amount,
                mutators.boost_accel,
                f32::from(car.boost),
                shot_details.time - *GAME_TIME.read().unwrap(),
            ) {
                Ok(shot_info)
            } else {
                Err(PyErr::new::<BadAccelerationPyErr, _>(BAD_ACCELERATION_ERR))
            }
        }
    }
}
