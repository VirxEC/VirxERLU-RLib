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

/// VirxERLU-RLib is written in Rust with Python bindings to make analyzing the ball prediction struct much faster.
#[pymodule]
fn virx_erlu_rlib(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(load_soccer, m)?)?;
    m.add_function(wrap_pyfunction!(load_soccar, m)?)?;
    m.add_function(wrap_pyfunction!(load_dropshot, m)?)?;
    m.add_function(wrap_pyfunction!(load_hoops, m)?)?;
    m.add_function(wrap_pyfunction!(load_soccer_throwback, m)?)?;
    m.add_function(wrap_pyfunction!(load_soccar_throwback, m)?)?;
    m.add_function(wrap_pyfunction!(tick, m)?)?;
    m.add_function(wrap_pyfunction!(get_slice, m)?)?;
    m.add_function(wrap_pyfunction!(get_slice_index, m)?)?;
    m.add_function(wrap_pyfunction!(get_num_ball_slices, m)?)?;
    m.add_function(wrap_pyfunction!(new_target, m)?)?;
    m.add_function(wrap_pyfunction!(new_any_target, m)?)?;
    m.add_function(wrap_pyfunction!(confirm_target, m)?)?;
    m.add_function(wrap_pyfunction!(remove_target, m)?)?;
    m.add_function(wrap_pyfunction!(print_targets, m)?)?;
    m.add_function(wrap_pyfunction!(get_targets_length, m)?)?;
    m.add_function(wrap_pyfunction!(get_shot_with_target, m)?)?;
    m.add_function(wrap_pyfunction!(get_data_for_shot_with_target, m)?)?;
    m.add_function(wrap_pyfunction!(set_mutator_settings, m)?)?;
    m.add_class::<TargetOptions>()?;
    m.add_class::<ShotType>()?;
    Ok(())
}

#[pyfunction]
fn load_soccar() {
    let (game, ball) = rl_ball_sym::load_soccar();

    *GAME.write().unwrap() = Some(game);
    *BALL.write().unwrap() = Some(ball);
}

#[pyfunction]
fn load_soccer() {
    load_soccar();
}

#[pyfunction]
fn load_dropshot() {
    let (game, ball) = rl_ball_sym::load_dropshot();

    *GAME.write().unwrap() = Some(game);
    *BALL.write().unwrap() = Some(ball);
}

#[pyfunction]
fn load_hoops() {
    let (game, ball) = rl_ball_sym::load_hoops();

    *GAME.write().unwrap() = Some(game);
    *BALL.write().unwrap() = Some(ball);
}

#[pyfunction]
fn load_soccar_throwback() {
    let (game, ball) = rl_ball_sym::load_soccar_throwback();

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
    fn from(item: u8) -> BoostAmount {
        match item {
            0 => BoostAmount::Default,
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
    pub const fn new() -> Self {
        Mutators {
            boost_amount: BoostAmount::Default,
            boost_accel: BOOST_ACCEL,
        }
    }

    pub fn from(mutators: &PyAny) -> PyResult<Self> {
        Ok(Mutators {
            boost_amount: BoostAmount::from(mutators.call_method("BoostOption", (), None)?.extract()?),
            boost_accel: match mutators.call_method("BoostStrengthOption", (), None)?.extract()? {
                0 => BOOST_ACCEL,
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
    {
        let mut targets = TARGETS.write().unwrap();

        // simulate max jump height
        // simulate max double jump height
        // add option for max path time

        for target in targets.iter_mut() {
            if let Some(t) = target {
                if !t.is_confirmed() {
                    *target = None;
                }
            }
        }
    }

    /*
    Example GameTickPacket<
        game_cars: [
            PlayerInfo<
                physics: Physics<
                    location: Vector3<x: 0.0, y: -4608.0, z: 17.010000228881836>,
                    rotation: Rotator<pitch: -0.009587380103766918, yaw: 1.5707963705062866, roll: 0.0>,
                    velocity: Vector3<x: 0.0, y: 0.0, z: 0.210999995470047>,
                    angular_velocity: Vector3<x: -0.0006099999882280827, y: 0.0, z: 0.0>
                >,
                score_info: ScoreInfo<score: 0, goals: 0, own_goals: 0, assists: 0, saves: 0, shots: 0, demolitions: 0>,
                is_demolished: False,
                has_wheel_contact: True,
                is_super_sonic: False,
                is_bot: True,
                jumped: False,
                double_jumped: False,
                name: 'DownToEarth',
                team: 0,
                boost: 34,
                hitbox: BoxShape<length: 118.00737762451172, width: 84.19940948486328, height: 36.15907287597656>,
                hitbox_offset: Vector3<x: 13.875659942626953, y: 0.0, z: 20.754987716674805>,
                spawn_id: 1793714700
            >
        ],
        num_cars: 1,
        game_boosts: <rlbot.utils.structures.game_data_struct.BoostPadState_Array_50 object at 0x000002C910DE8EC8>,
        num_boost: 34,
        game_ball: BallInfo<
            physics: Physics<location: Vector3<x: 0.0, y: 0.0, z: 92.73999786376953>, rotation: Rotator<pitch: 0.0, yaw: 0.0, roll: 0.0>, velocity: Vector3<x: 0.0, y: 0.0, z: 0.0>, angular_velocity: Vector3<x: 0.0, y: 0.0, z: 0.0>>,
            latest_touch: Touch<player_name: '', time_seconds: 0.0, hit_location: Vector3<x: 0.0, y: 0.0, z: 0.0>, hit_normal: Vector3<x: 0.0, y: 0.0, z: 0.0>, team: 0, player_index: 0>,
            drop_shot_info: DropShotInfo<absorbed_force: 0.0, damage_index: 0, force_accum_recent: 0.0>,
            collision_shape: CollisionShape<type: 1, box: BoxShape<length: 0.0, width: 0.0, height: 0.0>,
            sphere: SphereShape<diameter: 182.49998474121094>, cylinder: CylinderShape<diameter: 0.0, height: 0.0>>
        >,
        game_info: GameInfo<seconds_elapsed: 718.4749755859375, game_time_remaining: -707.4849243164062, is_overtime: False, is_unlimited_time: True, is_round_active: False, is_kickoff_pause: False, is_match_ended: False, world_gravity_z: -650.0, game_speed: 0.0, frame_num: 86217>,
        dropshot_tiles: <rlbot.utils.structures.game_data_struct.TileInfo_Array_200 object at 0x000002C910DE8EC8>,
        num_tiles: 0,
        teams: <rlbot.utils.structures.game_data_struct.TeamInfo_Array_2 object at 0x000002C910DE8EC8>,
        num_teams: 2
    >
    */

    let mut game_guard = GAME.write().unwrap();
    let game = game_guard.as_mut().ok_or_else(|| PyErr::new::<NoGamePyErr, _>(NO_GAME_ERR))?;

    let mut ball = BALL.read().unwrap().ok_or_else(|| PyErr::new::<NoBallPyErr, _>(NO_BALL_ERR))?;

    let packet = packet.as_ref(py);

    {
        // Get the general game information
        let py_game_info = packet.getattr("game_info")?;

        *GAME_TIME.write().unwrap() = py_game_info.getattr("seconds_elapsed")?.extract::<f32>()?;
        game.gravity.z = py_game_info.getattr("world_gravity_z")?.extract()?;
        *GRAVITY.write().unwrap() = game.gravity;

        // Get information about the ball

        let py_ball = packet.getattr("game_ball")?;
        let py_ball_physics = py_ball.getattr("physics")?;

        ball.update(
            *GAME_TIME.read().unwrap(),
            get_vec3_named(py_ball_physics.getattr("location")?)?,
            get_vec3_named(py_ball_physics.getattr("velocity")?)?,
            get_vec3_named(py_ball_physics.getattr("angular_velocity")?)?,
        );

        let py_ball_shape = py_ball.getattr("collision_shape")?;

        let radius = py_ball_shape.getattr("sphere")?.getattr("diameter")?.extract::<f32>()? / 2.;
        ball.set_radius(radius, radius + 1.9);
    }

    // Predict future information about the ball
    *BALL_STRUCT.write().unwrap() = ball.get_ball_prediction_struct_for_time(game, &prediction_time.unwrap_or(6.));

    // Get information about the cars on the field
    {
        let mut cars = CARS.write().unwrap();

        let num_cars = packet.getattr("num_cars")?.extract::<usize>()?;
        let py_game_cars = packet.getattr("game_cars")?;

        if cars.len() != num_cars {
            const NEW_CAR: Car = Car::new();
            cars.resize(num_cars, NEW_CAR);
        }

        let game_time = *GAME_TIME.read().unwrap();

        for (i, car) in cars.iter_mut().enumerate() {
            car.update(py_game_cars.get_item(i)?, game_time)?;
        }
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
    let ball = ball_struct[slice_num.clamp(1, ball_struct.len()) - 1];

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

    let target_left = get_vec3_from_vec(left_target, "target_left")?;
    let target_right = get_vec3_from_vec(right_target, "target_right")?;
    let options = Options::from(options, num_slices);

    {
        let mut cars = CARS.write().unwrap();
        let car = cars.get_mut(car_index).ok_or_else(|| PyErr::new::<NoCarPyErr, _>(NO_CAR_ERR))?;
        car.init(GRAVITY.read().unwrap().z, num_slices, &*MUTATORS.read().unwrap());
    }

    let target = Some(Target::new(target_left, target_right, car_index, options));
    let mut targets = TARGETS.write().unwrap();

    let target_position = targets.iter().position(|x| x.is_none());
    let target_index = match target_position {
        Some(i) => {
            targets[i] = target;
            i
        }
        None => {
            targets.push(target);
            targets.len() - 1
        }
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
        car.init(GRAVITY.read().unwrap().z, num_slices, &*MUTATORS.read().unwrap());
    }

    let target = Some(Target::new_any(car_index, options));
    let mut targets = TARGETS.write().unwrap();

    let target_position = targets.iter().position(|x| x.is_none());
    let target_index = match target_position {
        Some(i) => {
            targets[i] = target;
            i
        }
        None => {
            targets.push(target);
            targets.len() - 1
        }
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

        if ball_prediction.len() == 0 || car.demolished || car.landing_time >= ball_prediction[ball_prediction.len() - 1].time {
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

                    let target_info = match analyzer.aerial_shot(mutators, target_location, shot_vector, max_time_remaining) {
                        Ok(ti) => ti,
                        Err(_) => continue,
                    };

                    if found_shot.is_none() {
                        basic_shot_info = Some(target_info.get_basic_shot_info(ball.time));

                        found_shot = Some(if temporary { AirBasedShot::default() } else { AirBasedShot::from(ball, target_info) }.into());

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

                if target_info.can_reach(car, max_time_remaining, &mutators).is_err() {
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

                    let target_info = match analyzer.aerial_shot(mutators, target_location, shot_vector, max_time_remaining) {
                        Ok(ti) => ti,
                        Err(_) => continue,
                    };

                    if found_shot.is_none() {
                        basic_shot_info = Some(target_info.get_basic_shot_info(ball.time));

                        found_shot = Some(if temporary { AirBasedShot::default() } else { AirBasedShot::from(ball, target_info) }.into());

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

                if target_info.can_reach(car, max_time_remaining, &mutators).is_err() {
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
            let shot_info = AdvancedShotInfo::get_from_air(car, shot_details).ok_or_else(|| PyErr::new::<StrayedFromPathPyErr, _>(STRAYED_FROM_PATH_ERR))?;

            Ok(shot_info)
        }
    }
}
