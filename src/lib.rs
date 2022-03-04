mod car;
mod constants;
mod ground;
mod pytypes;
mod shot;
mod utils;

use car::{turn_radius, Car};
use constants::*;
use ground::*;
use lazy_static::{initialize, lazy_static};
use pyo3::{prelude::*, PyErr};
use pytypes::*;
use rl_ball_sym::simulation::{
    ball::{Ball, BallPrediction},
    game::Game,
};
use shot::{Options, Shot, Target};
use std::sync::Mutex;
use utils::*;

lazy_static! {
    static ref GAME_TIME: Mutex<f32> = Mutex::new(0.);
}

lazy_static! {
    static ref GAME: Mutex<Option<Game>> = Mutex::new(None);
}

lazy_static! {
    static ref CARS: Mutex<[Car; 64]> = Mutex::new([Car::default(); 64]);
}

lazy_static! {
    static ref BALL_STRUCT: Mutex<BallPrediction> = Mutex::new(BallPrediction::default());
}

lazy_static! {
    static ref TARGETS: Mutex<Vec<Option<Target>>> = Mutex::new(Vec::new());
}

/// VirxERLU-RLib is written in Rust with Python bindings to make analyzing the ball prediction struct much faster.
#[pymodule]
fn virx_erlu_rlib(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(load_soccar, m)?)?;
    m.add_function(wrap_pyfunction!(load_dropshot, m)?)?;
    m.add_function(wrap_pyfunction!(load_hoops, m)?)?;
    m.add_function(wrap_pyfunction!(load_soccar_throwback, m)?)?;
    m.add_function(wrap_pyfunction!(tick, m)?)?;
    m.add_function(wrap_pyfunction!(get_slice, m)?)?;
    m.add_function(wrap_pyfunction!(new_target, m)?)?;
    m.add_function(wrap_pyfunction!(confirm_target, m)?)?;
    m.add_function(wrap_pyfunction!(remove_target, m)?)?;
    m.add_function(wrap_pyfunction!(print_targets, m)?)?;
    m.add_function(wrap_pyfunction!(get_targets_length, m)?)?;
    m.add_function(wrap_pyfunction!(get_shot_with_target, m)?)?;
    m.add_function(wrap_pyfunction!(get_data_for_shot_with_target, m)?)?;
    m.add_class::<TargetOptions>()?;
    m.add_class::<ShotType>()?;
    Ok(())
}

fn init() {
    initialize(&GAME_TIME);
    initialize(&TARGETS);
    initialize(&BALL_STRUCT);
    initialize(&CARS);
}

#[pyfunction]
fn load_soccar() {
    init();

    let mut game_guard = GAME.lock().unwrap();
    *game_guard = Some(rl_ball_sym::load_soccar());
}

#[pyfunction]
fn load_dropshot() {
    init();

    let mut game_guard = GAME.lock().unwrap();
    *game_guard = Some(rl_ball_sym::load_dropshot());
}

#[pyfunction]
fn load_hoops() {
    init();

    let mut game_guard = GAME.lock().unwrap();
    *game_guard = Some(rl_ball_sym::load_hoops());
}

#[pyfunction]
fn load_soccar_throwback() {
    init();

    let mut game_guard = GAME.lock().unwrap();
    *game_guard = Some(rl_ball_sym::load_soccar_throwback());
}

#[pyfunction]
fn tick(py: Python, packet: PyObject, prediction_time: Option<f32>) -> PyResult<()> {
    let mut game_guard = GAME.lock().unwrap();
    let game = game_guard.as_mut().ok_or_else(|| PyErr::new::<NoGamePyErr, _>(NO_GAME_ERR))?;

    let mut cars = CARS.lock().unwrap();
    let mut targets = TARGETS.lock().unwrap();

    // simulate max jump height
    // simulate max double jump height
    // add option for max path time

    targets.retain(|target| match target {
        Some(t) => t.is_confirmed(),
        None => true,
    });

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
    let packet = packet.as_ref(py);

    let py_game_info = packet.getattr("game_info")?;

    let mut time = GAME_TIME.lock().unwrap();
    *time = py_game_info.getattr("seconds_elapsed")?.extract::<f32>()?;

    game.gravity.z = py_game_info.getattr("world_gravity_z")?.extract()?;

    let py_ball = packet.getattr("game_ball")?;
    let py_ball_physics = py_ball.getattr("physics")?;

    game.ball.update(
        *time,
        get_vec3_named(py_ball_physics.getattr("location")?)?,
        get_vec3_named(py_ball_physics.getattr("velocity")?)?,
        get_vec3_named(py_ball_physics.getattr("angular_velocity")?)?,
    );

    let py_ball_shape = py_ball.getattr("collision_shape")?;

    game.ball.radius = py_ball_shape.getattr("sphere")?.getattr("diameter")?.extract::<f32>()? / 2.;
    game.ball.collision_radius = game.ball.radius + 1.9;
    game.ball.calculate_moi();

    let prediction_time = prediction_time.unwrap_or(6.);
    let ball_struct = Ball::get_ball_prediction_struct_for_time(game, &prediction_time);

    let mut ball_struct_guard = BALL_STRUCT.lock().unwrap();
    *ball_struct_guard = ball_struct;

    let num_cars = packet.getattr("num_cars")?.extract::<usize>()?;
    let py_game_cars = packet.getattr("game_cars")?;

    for (i, car) in cars.iter_mut().enumerate().take(num_cars) {
        car.update(py_game_cars.get_item(i)?)?;
    }

    Ok(())
}

#[pyfunction]
fn get_slice(slice_time: f32) -> BallSlice {
    let game_time = GAME_TIME.lock().unwrap();
    let ball_struct = BALL_STRUCT.lock().unwrap();

    let slice_num = ((slice_time - *game_time) * 120.).round() as usize;
    let ball = ball_struct.slices[slice_num.clamp(1, ball_struct.num_slices) - 1];

    BallSlice::from(&ball)
}

#[pyfunction]
fn new_target(left_target: Vec<f32>, right_target: Vec<f32>, car_index: usize, options: Option<TargetOptions>) -> PyResult<usize> {
    let num_slices = BALL_STRUCT.lock().unwrap().num_slices;

    if num_slices == 0 {
        return Err(PyErr::new::<NoSlicesPyErr, _>(NO_SLICES_ERR));
    }

    let target_left = get_vec3_from_vec(left_target, "target_left")?;
    let target_right = get_vec3_from_vec(right_target, "target_right")?;
    let options = Options::from(options, num_slices);

    let target = Some(Target::new(target_left, target_right, car_index, options));
    let mut targets = TARGETS.lock().unwrap();

    let target_index = match targets.iter().position(|x| x.is_none()) {
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
    let mut targets = TARGETS.lock().unwrap();
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
    let mut targets = TARGETS.lock().unwrap();
    if targets.get(target_index).is_none() {
        return;
    }

    targets[target_index] = None;
}

#[pyfunction]
fn print_targets() {
    let targets = TARGETS.lock().unwrap();
    let mut out = Vec::with_capacity(targets.len());

    for target in targets.iter() {
        match target {
            Some(t) => match &t.shot {
                Some(s) => out.push(format!("{}", s.time)),
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
    TARGETS.lock().unwrap().len()
}

#[pyfunction]
fn get_shot_with_target(target_index: usize, temporary: Option<bool>, may_ground_shot: Option<bool>, only: Option<bool>) -> PyResult<BasicShotInfo> {
    let only = only.unwrap_or(false);
    let may_ground_shot = may_ground_shot.unwrap_or(!only);

    if !may_ground_shot {
        return Err(PyErr::new::<NoShotSelectedPyErr, _>(NO_SHOT_SELECTED_ERR));
    }

    let _gravity = {
        let game_guard = GAME.lock().unwrap();
        let game = game_guard.as_ref().ok_or_else(|| PyErr::new::<NoGamePyErr, _>(NO_GAME_ERR))?;

        game.gravity
    };

    let game_time = GAME_TIME.lock().unwrap();
    let ball_prediction = BALL_STRUCT.lock().unwrap();

    let mut targets_gaurd = TARGETS.lock().unwrap();
    let target = targets_gaurd
        .get_mut(target_index)
        .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?
        .as_mut()
        .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?;

    let mut cars = CARS.lock().unwrap();
    let car = cars.get_mut(target.car_index).ok_or_else(|| PyErr::new::<NoCarPyErr, _>(NO_CAR_ERR))?;

    let mut found_shot = None;
    let mut basic_shot_info = None;

    if ball_prediction.num_slices == 0 || car.demolished || car.airborne {
        return Ok(BasicShotInfo::not_found());
    }

    let max_speed = if target.options.use_absolute_max_values { MAX_SPEED } else { car.max_speed };

    let max_turn_radius = if target.options.use_absolute_max_values { turn_radius(MAX_SPEED) } else { car.ctrms };

    let analyze_options = AnalyzeOptions {
        max_speed,
        max_turn_radius,
        get_target: false,
    };

    let temporary = temporary.unwrap_or(false);

    for ball in &ball_prediction.slices[target.options.min_slice..target.options.max_slice] {
        if ball.location.y.abs() > 5120. + ball.collision_radius {
            break;
        }

        let post_info = correct_for_posts(ball.location, ball.collision_radius, target.target_left, target.target_right);

        if !post_info.fits {
            continue;
        }

        let shot_vector = get_shot_vector_target(car.location, ball.location, post_info.target_left, post_info.target_right);
        let max_time_remaining = ball.time - *game_time;
        let result = match analyze_target(ball, car, shot_vector, max_time_remaining, analyze_options) {
            Ok(r) => r,
            Err(_) => continue,
        };

        let is_forwards = true;

        // will be used to calculate if there's enough time left to jump after accelerating
        let _time_remaining = match can_reach_target(car, max_time_remaining, result.distances, result.path.type_, result.path.rho, is_forwards) {
            Ok(t_r) => t_r,
            Err(_) => continue,
        };

        if found_shot.is_none() {
            basic_shot_info = Some(BasicShotInfo::found(ball.time, result.shot_type));

            if !temporary {
                found_shot = Some(Shot::from(ball, result.path, result.distances, shot_vector));
            }

            if !target.options.all {
                break;
            }
        }
    }

    if !temporary {
        target.shot = found_shot;
    }

    Ok(match basic_shot_info {
        Some(bsi) => bsi,
        None => BasicShotInfo::not_found(),
    })
}

#[pyfunction]
fn get_data_for_shot_with_target(target_index: usize) -> PyResult<AdvancedShotInfo> {
    let _gravity = {
        let game_guard = GAME.lock().unwrap();
        let game = game_guard.as_ref().ok_or_else(|| PyErr::new::<NoGamePyErr, _>(NO_GAME_ERR))?;

        game.gravity
    };

    let targets_gaurd = TARGETS.lock().unwrap();
    let target = targets_gaurd
        .get(target_index)
        .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?
        .as_ref()
        .ok_or_else(|| PyErr::new::<NoTargetPyErr, _>(NO_TARGET_ERR))?;
    let shot = target.shot.as_ref().ok_or_else(|| PyErr::new::<NoShotPyErr, _>(NO_SHOT_ERR))?;

    let time_remaining = {
        let game_time = GAME_TIME.lock().unwrap();
        shot.time - *game_time
    };

    if time_remaining < 0. {
        return Err(PyErr::new::<NoTimeRemainingPyErr, _>(NO_TIME_REMAINING_ERR));
    }

    let cars_guard = CARS.lock().unwrap();
    let car = cars_guard.get(target.car_index).ok_or_else(|| PyErr::new::<NoCarPyErr, _>(NO_CAR_ERR))?;

    let ball_struct = BALL_STRUCT.lock().unwrap();
    let ball = {
        let slice_num = (time_remaining * 120.).round() as usize;

        &ball_struct.slices[slice_num.clamp(1, ball_struct.num_slices) - 1]
    };

    if ball.location.distance(shot.ball_location) > car.hitbox.width {
        Err(PyErr::new::<BallChangedPyErr, _>(BALL_CHANGED_ERR))
    } else {
        let shot_info = AdvancedShotInfo::get(car, shot);

        if car.max_speed * (time_remaining + 0.2) >= shot_info.get_distance_remaining() {
            Ok(shot_info)
        } else {
            Err(PyErr::new::<BadAccelerationPyErr, _>(BAD_ACCELERATION_ERR))
        }
    }
}
