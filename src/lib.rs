extern crate cpython;
extern crate rl_ball_sym;

mod car;
mod constants;
mod ground;
mod shot;
mod utils;

use car::{turn_radius, Car, Hitbox};
use constants::*;
use cpython::{exc, py_fn, py_module_initializer, ObjectProtocol, PyDict, PyErr, PyFloat, PyInt, PyObject, PyResult, PyTuple, Python, PythonObject};
use glam::Vec3A;
use ground::*;
use rl_ball_sym::simulation::{
    ball::{Ball, BallPrediction},
    game::Game,
};
use shot::{Options, Shot, Target};
use utils::*;

static mut GAME_TIME: f32 = 0.;

static mut GAME: Option<Game> = None;
const NO_GAME_ERR: &str = "GAME is unset. Call a function like load_soccar first.";

static mut CARS: Option<Vec<Car>> = None;
const NO_CARS_ERR: &str = "CARS is unset. Call a function like load_soccar first.";

static mut BALL_STRUCT: Option<BallPrediction> = None;
const NO_BALL_STRUCT_ERR: &str = "BALL_STRUCT is unset. Call the function tick and pass in game information first.";

static mut TARGETS: Vec<Option<Target>> = Vec::new();
const NO_TARGET_ERR: &str = "Target no longer exists";

py_module_initializer!(virxrlru, |py, m| {
    m.add(py, "__doc__", "VirxERLU-RLib is written in Rust with Python bindings to make analyzing the ball prediction struct much faster.")?;
    #[allow(clippy::manual_strip)]
    m.add(py, "load_soccar", py_fn!(py, load_soccar()))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "load_dropshot", py_fn!(py, load_dropshot()))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "load_hoops", py_fn!(py, load_hoops()))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "load_soccar_throwback", py_fn!(py, load_soccar_throwback()))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "set_gravity", py_fn!(py, set_gravity(gravity: PyDict)))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "tick", py_fn!(py, tick(packet: PyObject)))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "tick_dict", py_fn!(py, tick_dict(time: PyFloat, ball: PyDict, car: PyDict)))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "get_slice", py_fn!(py, get_slice(time: PyFloat)))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "new_target", py_fn!(py, new_target(target_left: PyTuple, target_right: PyTuple, car_index: PyInt, options: PyDict)))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "confirm_target", py_fn!(py, confirm_target(target: PyInt)))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "remove_target", py_fn!(py, remove_target(target: PyInt)))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "print_targets", py_fn!(py, print_targets()))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "get_data_for_shot_with_target", py_fn!(py, get_data_for_shot_with_target(target: PyInt)))?;
    #[allow(clippy::manual_strip)]
    m.add(py, "get_shot_with_target", py_fn!(py, get_shot_with_target(target: PyInt)))?;
    Ok(())
});

fn load_soccar(py: Python) -> PyResult<PyObject> {
    unsafe {
        GAME = Some(rl_ball_sym::load_soccar());
        CARS = Some(vec![Car::default(); 64]);
        BALL_STRUCT = Some(BallPrediction::default());
    }

    Ok(py.None())
}

fn load_dropshot(py: Python) -> PyResult<PyObject> {
    unsafe {
        GAME = Some(rl_ball_sym::load_dropshot());
        CARS = Some(vec![Car::default(); 64]);
        BALL_STRUCT = Some(BallPrediction::default());
    }

    Ok(py.None())
}

fn load_hoops(py: Python) -> PyResult<PyObject> {
    unsafe {
        GAME = Some(rl_ball_sym::load_hoops());
        CARS = Some(vec![Car::default(); 64]);
        BALL_STRUCT = Some(BallPrediction::default());
    }

    Ok(py.None())
}

fn load_soccar_throwback(py: Python) -> PyResult<PyObject> {
    unsafe {
        GAME = Some(rl_ball_sym::load_soccar_throwback());
    }

    Ok(py.None())
}

fn set_gravity(py: Python, py_gravity: PyDict) -> PyResult<PyObject> {
    let mut game: &mut Game;

    unsafe {
        game = match GAME.as_mut() {
            Some(game) => game,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_GAME_ERR));
            }
        };
    }

    game.gravity = get_vec3(py, py_gravity.as_object(), "Key 'gravity' needs to be a list of exactly 3 numbers")?;

    Ok(py.None())
}

fn tick(py: Python, py_packet: PyObject) -> PyResult<PyObject> {
    let game: &mut Game;
    let cars: &mut Vec<Car>;
    let targets: &mut Vec<Option<Target>>;

    // simulate max jump height
    // simulate max double jump height
    // add option for max path time

    unsafe {
        game = match GAME.as_mut() {
            Some(game) => game,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_GAME_ERR));
            }
        };

        cars = match &mut CARS {
            Some(cars) => cars,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_CARS_ERR));
            }
        };

        targets = &mut TARGETS;
    }

    targets.retain(|target| match target {
        Some(t) => t.is_confirmed(),
        None => false,
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
    let py_game_info = py_packet.getattr(py, "game_info")?;

    let time = py_game_info.getattr(py, "seconds_elapsed")?.extract(py)?;

    unsafe {
        GAME_TIME = time;
    }

    game.gravity.z = py_game_info.getattr(py, "world_gravity_z")?.extract(py)?;

    let py_ball = py_packet.getattr(py, "game_ball")?;
    let py_ball_physics = py_ball.getattr(py, "physics")?;

    game.ball.update(
        time,
        get_vec3_named(py, py_ball_physics.getattr(py, "location")?)?,
        get_vec3_named(py, py_ball_physics.getattr(py, "velocity")?)?,
        get_vec3_named(py, py_ball_physics.getattr(py, "angular_velocity")?)?,
    );

    let py_ball_shape = py_ball.getattr(py, "collision_shape")?;

    game.ball.radius = py_ball_shape.getattr(py, "sphere")?.getattr(py, "diameter")?.extract::<f32>(py)? / 2.;
    game.ball.collision_radius = game.ball.radius + 0.9;
    game.ball.calculate_moi();

    let num_cars = py_packet.getattr(py, "num_cars")?.extract::<usize>(py)?;
    let py_game_cars = py_packet.getattr(py, "game_cars")?;

    for (i, car) in cars.iter_mut().enumerate().take(num_cars) {
        car.update(py, py_game_cars.get_item(py, i)?)?;
    }

    unsafe {
        BALL_STRUCT = Some(Ball::get_ball_prediction_struct_for_time(game, &6.));
    }

    Ok(py.None())
}

fn tick_dict(py: Python, py_time: PyFloat, py_ball: PyDict, py_car: PyDict) -> PyResult<PyObject> {
    let mut game: &mut Game;
    let cars: &mut Vec<Car>;
    let targets: &mut Vec<Option<Target>>;

    unsafe {
        game = match GAME.as_mut() {
            Some(game) => game,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_GAME_ERR));
            }
        };

        cars = match &mut CARS {
            Some(cars) => cars,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_CARS_ERR));
            }
        };

        targets = &mut TARGETS;
    }

    targets.retain(|target| match target {
        Some(t) => t.is_confirmed(),
        None => false,
    });

    game.ball.time = py_time.value(py) as f32;

    unsafe {
        GAME_TIME = game.ball.time;
    }

    game.ball.location = get_vec3_from_dict(py, &py_ball, "location", "ball")?;
    game.ball.velocity = get_vec3_from_dict(py, &py_ball, "velocity", "ball")?;
    game.ball.angular_velocity = get_vec3_from_dict(py, &py_ball, "angular_velocity", "ball")?;

    if let Some(radius) = py_ball.get_item(py, "radius") {
        game.ball.radius = radius.extract(py)?;
        game.ball.calculate_moi();
    }

    if let Some(collision_radius) = py_ball.get_item(py, "collision_radius") {
        game.ball.collision_radius = collision_radius.extract(py)?;
    }

    let index = get_usize_from_dict(py, &py_car, "index", "car")?;

    cars[index].location = get_vec3_from_dict(py, &py_car, "location", "car")?;
    cars[index].velocity = get_vec3_from_dict(py, &py_car, "velocity", "car")?;
    cars[index].angular_velocity = get_vec3_from_dict(py, &py_car, "angular_velocity", "car")?;
    cars[index].hitbox = Hitbox::from_vec3(get_vec3_from_dict(py, &py_car, "hitbox", "car")?);
    cars[index].hitbox_offset = get_vec3_from_dict(py, &py_car, "hitbox_offset", "car")?;
    cars[index].pitch = get_f32_from_dict(py, &py_car, "pitch", "car")?;
    cars[index].yaw = get_f32_from_dict(py, &py_car, "yaw", "car")?;
    cars[index].roll = get_f32_from_dict(py, &py_car, "roll", "car")?;
    cars[index].boost = get_u8_from_dict(py, &py_car, "boost", "car")?;
    cars[index].demolished = get_bool_from_dict(py, &py_car, "demolished", "car")?;
    cars[index].airborne = get_bool_from_dict(py, &py_car, "airborne", "car")?;
    cars[index].jumped = get_bool_from_dict(py, &py_car, "jumped", "car")?;
    cars[index].doublejumped = get_bool_from_dict(py, &py_car, "doublejumped", "car")?;
    cars[index].calculate_orientation_matrix();
    cars[index].calculate_max_values();
    cars[index].calculate_local_values();
    cars[index].calculate_field();

    unsafe {
        BALL_STRUCT = Some(Ball::get_ball_prediction_struct_for_time(game, &6.));
    }

    Ok(py.None())
}

fn get_slice(py: Python, py_slice_time: PyFloat) -> PyResult<PyObject> {
    let game_time: &f32;
    let ball_struct: &BallPrediction;

    unsafe {
        game_time = &GAME_TIME;

        ball_struct = match BALL_STRUCT.as_ref() {
            Some(ball_struct) => ball_struct,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_BALL_STRUCT_ERR));
            }
        };
    }

    let py_slice = PyDict::new(py);
    let slice_num = ((py_slice_time.value(py) as f32 - game_time) * 120.).round() as usize;

    let slice = ball_struct.slices[slice_num.clamp(1, ball_struct.num_slices) - 1];

    py_slice.set_item(py, "time", slice.time)?;
    py_slice.set_item(py, "location", get_vec_from_vec3(slice.location))?;
    py_slice.set_item(py, "velocity", get_vec_from_vec3(slice.velocity))?;
    py_slice.set_item(py, "angular_velocity", get_vec_from_vec3(slice.angular_velocity))?;

    Ok(py_slice.into_object())
}

fn new_target(py: Python, py_target_left: PyTuple, py_target_right: PyTuple, py_car_index: PyInt, py_options: PyDict) -> PyResult<usize> {
    let num_slices: usize;

    unsafe {
        num_slices = match BALL_STRUCT.as_ref() {
            Some(ball_struct) => ball_struct.num_slices,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_BALL_STRUCT_ERR));
            }
        };
    }

    let car_index = py_car_index.into_object().extract::<usize>(py).unwrap();
    let target_left = get_vec3(py, py_target_left.as_object(), "Key 'target_left' needs to be a list of exactly 3 numbers")?;
    let target_right = get_vec3(py, py_target_right.as_object(), "Key 'target_right' needs to be a list of exactly 3 numbers")?;
    let options = Options::from(py, py_options, num_slices)?;

    let target = Some(Target::new(target_left, target_right, car_index, options));
    let target_index;

    unsafe {
        target_index = TARGETS.len();
        TARGETS.push(target);
    }

    Ok(target_index)
}

fn confirm_target(py: Python, py_target: PyInt) -> PyResult<PyObject> {
    let target;
    let target_index = py_target.into_object().extract::<usize>(py).unwrap();

    unsafe {
        target = match TARGETS[target_index].as_mut() {
            Some(t) => t,
            None => {
                return Err(PyErr::new::<exc::IndexError, _>(py, NO_TARGET_ERR));
            }
        }
    }

    target.confirm();

    Ok(py.None())
}

fn remove_target(py: Python, py_target: PyInt) -> PyResult<PyObject> {
    let target_index = py_target.into_object().extract::<usize>(py).unwrap();

    unsafe {
        TARGETS[target_index] = None;
    }

    Ok(py.None())
}

fn print_targets(py: Python) -> PyResult<PyObject> {
    unsafe {
        dbg!(&TARGETS);
    }

    Ok(py.None())
}

fn get_data_for_shot_with_target(py: Python, py_target: PyInt) -> PyResult<PyObject> {
    let game_time: &f32;
    let _gravity: &Vec3A;
    let car: &Car;
    let ball: &Ball;
    let target: &Target;

    unsafe {
        game_time = &GAME_TIME;
    }

    let target_index = py_target.into_object().extract::<usize>(py).unwrap();

    unsafe {
        match GAME.as_ref() {
            Some(game) => {
                _gravity = &game.gravity;
            }
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_GAME_ERR));
            }
        }

        target = match TARGETS[target_index].as_ref() {
            Some(t) => t,
            None => {
                return Err(PyErr::new::<exc::IndexError, _>(py, NO_TARGET_ERR));
            }
        };

        car = match &mut CARS {
            Some(cars) => cars.get_mut(target.car_index).unwrap(),
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_CARS_ERR));
            }
        };
    }

    let shot = target.shot.as_ref().unwrap();
    let slice_num = ((shot.time - game_time) * 120.).round() as usize;

    unsafe {
        ball = match BALL_STRUCT.as_ref() {
            Some(ball_struct) => &ball_struct.slices[slice_num.clamp(1, ball_struct.num_slices) - 1],
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_BALL_STRUCT_ERR));
            }
        };
    }

    let result = PyDict::new(py);

    let distance_remaining: f32 = shot.distances.iter().sum();
    result.set_item(py, "distance_remaining", distance_remaining)?;

    let car_to_ball = ball.location - car.location;
    let post_info = correct_for_posts(ball.location, ball.collision_radius, target.target_left, target.target_right);
    let shot_vector = get_shot_vector_2d(flatten(car_to_ball).normalize_or_zero(), flatten(ball.location), flatten(post_info.target_left), flatten(post_info.target_right));

    let target = match get_target(car, shot, shot_vector) {
        Ok(t) => t,
        Err(e) => {
            return Err(PyErr::new::<exc::Exception, _>(py, format!("{:?} - Couldn't calculate final target", e)));
        }
    };

    result.set_item(py, "final_target", get_vec_from_vec3(target))?;

    let mut path_samples = Vec::with_capacity(shot.sections.len());

    for section in &shot.sections {
        path_samples.push(vec![section.x, section.y])
    }

    result.set_item(py, "path_samples", path_samples)?;

    Ok(result.into_object())
}

fn get_shot_with_target(py: Python, py_target: PyInt) -> PyResult<PyObject> {
    let game_time: &f32;
    let _gravity: &Vec3A;
    let radius: &f32;
    let car: &Car;
    let ball_prediction: &BallPrediction;
    let target: &mut Target;

    unsafe {
        game_time = &GAME_TIME;
    }

    let target_index = py_target.into_object().extract::<usize>(py).unwrap();

    unsafe {
        match GAME.as_ref() {
            Some(game) => {
                _gravity = &game.gravity;
                radius = &game.ball.radius;
            }
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_GAME_ERR));
            }
        }

        ball_prediction = match BALL_STRUCT.as_ref() {
            Some(ball_struct) => ball_struct,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_BALL_STRUCT_ERR));
            }
        };

        target = match TARGETS[target_index].as_mut() {
            Some(t) => t,
            None => {
                return Err(PyErr::new::<exc::IndexError, _>(py, NO_TARGET_ERR));
            }
        };

        car = match &mut CARS {
            Some(cars) => cars.get_mut(target.car_index).unwrap(),
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_CARS_ERR));
            }
        };
    }

    let dist_from_side = radius + car.hitbox.height;

    let mut found_shot = None;
    let mut found_time = None;

    if ball_prediction.num_slices == 0 || car.demolished || car.airborne {
        let result = PyDict::new(py);
        result.set_item(py, "found", false)?;
        return Ok(result.into_object());
    }

    let max_speed = if target.options.use_absolute_max_values {
        MAX_SPEED
    } else {
        car.max_speed
    };

    let max_turn_radius = if target.options.use_absolute_max_values {
        turn_radius(MAX_SPEED)
    } else {
        car.ctrms
    };

    let analyze_options = AnalyzeOptions {
        max_speed,
        max_turn_radius,
        get_target: false,
        validate: true,
    };

    for ball in &ball_prediction.slices[target.options.min_slice..target.options.max_slice] {
        if ball.location.y.abs() > 5120. + ball.collision_radius {
            break;
        }

        if ball.location.z > dist_from_side {
            continue;
        }

        let car_to_ball = ball.location - car.location;

        let post_info = correct_for_posts(ball.location, ball.collision_radius, target.target_left, target.target_right);

        if !post_info.fits {
            continue;
        }

        let shot_vector = get_shot_vector_2d(flatten(car_to_ball).normalize_or_zero(), flatten(ball.location), flatten(post_info.target_left), flatten(post_info.target_right));
        let max_time_remaining = ball.time - game_time;
        let result = match analyze_target(ball, car, shot_vector, max_time_remaining, analyze_options) {
            Ok(r) => r,
            Err(_) => continue,
        };

        let distance_remaining = result.distances.iter().sum();
        let is_forwards = true;

        // will be used to calculate if there's enough time left to jump after accelerating
        let _time_remaining = match can_reach_target(car, max_speed, max_time_remaining, distance_remaining, is_forwards) {
            Ok(t_r) => t_r,
            Err(_) => continue,
        };

        if found_shot.is_none() {
            found_shot = Some(Shot::from(ball.time, result.path, result.distances));
            found_time = Some(ball.time);

            if !target.options.all {
                break;
            }
        }
    }

    target.shot = found_shot;

    let result = PyDict::new(py);

    match found_time {
        Some(time) => {
            result.set_item(py, "found", true)?;
            result.set_item(py, "time", time)?;
        }
        None => {
            result.set_item(py, "found", false)?;
        }
    }

    Ok(result.into_object())
}
