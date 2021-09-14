extern crate cpython;
extern crate rl_ball_sym;

mod utils;

use cpython::{exc, py_fn, py_module_initializer, PyBool, PyDict, PyErr, PyFloat, PyList, PyObject, PyResult, Python, PythonObject};
use rl_ball_sym::{
    linear_algebra::vector::Vec3,
    simulation::{
        ball::{Ball, BallPrediction},
        game::Game,
    },
};
use utils::*;

static mut GAME_TIME: f32 = 0.;

static mut GAME: Option<Game> = None;
static NO_GAME_ERR: &str = "GAME is unset. Call a function like load_soccar first.";

static mut CAR: Option<Car> = None;
static NO_CAR_ERR: &str = "CAR is unset. Call a function like load_soccar first.";

static mut BALL_STRUCT: Option<BallPrediction> = None;
static NO_BALL_STRUCT_ERR: &str = "BALL_STRUCT is unset. Call the function tick and pass in game information first.";

static mut TURN_ACCEL_LUT: Option<TurnLut> = None;
static NO_TURN_ACCEL_LUT_ERR: &str = "TURN_ACCEL_LUT is unset. Call a function like load_soccar first.";

static mut TURN_ACCEL_BOOST_LUT: Option<TurnLut> = None;
static NO_TURN_ACCEL_BOOST_LUT_ERR: &str = "TURN_ACCEL_BOOST_LUT is unset. Call a function like load_soccar first.";

static mut TURN_DECEL_LUT: Option<TurnLut> = None;
static NO_TURN_DECEL_LUT_ERR: &str = "TURN_DECEL_LUT is unset. Call a function like load_soccar first.";

py_module_initializer!(virxrlru, |py, m| {
    m.add(py, "__doc__", "VirxERLU-RLib is written in Rust with Python bindings to make analyzing the ball prediction struct much faster.")?;
    m.add(py, "load_soccar", py_fn!(py, load_soccar()))?;
    m.add(py, "load_dropshot", py_fn!(py, load_dropshot()))?;
    m.add(py, "load_hoops", py_fn!(py, load_hoops()))?;
    m.add(py, "set_gravity", py_fn!(py, set_gravity(gravity: PyDict)))?;
    m.add(py, "tick", py_fn!(py, tick(time: PyFloat, ball: PyDict, car: PyDict)))?;
    m.add(py, "get_slice", py_fn!(py, get_slice(time: PyFloat)))?;
    m.add(py, "calc_dr_and_ft", py_fn!(py, calc_dr_and_ft(target: PyList, time: PyFloat)))?;
    m.add(py, "calculate_intercept", py_fn!(py, calculate_intercept(target: PyList, all: PyBool)))?;
    Ok(())
});

fn load_luts() {
    let turn_accel_lut = TurnLut::from(read_turn_bin(include_bytes!("../turn_data/turn_data_accel.bin").to_vec()));
    let turn_accel_boost_lut = TurnLut::from(read_turn_bin(include_bytes!("../turn_data/turn_data_accel_boost.bin").to_vec()));
    let turn_decel_lut = TurnLut::from(read_turn_bin(include_bytes!("../turn_data/turn_data_decel.bin").to_vec()));

    unsafe {
        TURN_ACCEL_LUT = Some(turn_accel_lut);
        TURN_ACCEL_BOOST_LUT = Some(turn_accel_boost_lut);
        TURN_DECEL_LUT = Some(turn_decel_lut);
    }
}

fn load_soccar(py: Python) -> PyResult<PyObject> {
    unsafe {
        GAME = Some(rl_ball_sym::load_soccar());
        CAR = Some(Car::default());
        BALL_STRUCT = Some(BallPrediction::default());
    }

    load_luts();

    Ok(py.None())
}

fn load_dropshot(py: Python) -> PyResult<PyObject> {
    unsafe {
        GAME = Some(rl_ball_sym::load_dropshot());
        CAR = Some(Car::default());
        BALL_STRUCT = Some(BallPrediction::default());
    }

    load_luts();

    Ok(py.None())
}

fn load_hoops(py: Python) -> PyResult<PyObject> {
    unsafe {
        GAME = Some(rl_ball_sym::load_hoops());
        CAR = Some(Car::default());
        BALL_STRUCT = Some(BallPrediction::default());
    }

    load_luts();

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

fn tick(py: Python, py_time: PyFloat, py_ball: PyDict, py_car: PyDict) -> PyResult<PyObject> {
    let mut game: &mut Game;
    let mut car: &mut Car;

    unsafe {
        game = match GAME.as_mut() {
            Some(game) => game,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_GAME_ERR));
            }
        };

        car = match CAR.as_mut() {
            Some(car) => car,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_CAR_ERR));
            }
        };
    }

    // adjust this to accept the entire game tick packet
    // this will require a minor version bump
    // make shot also take the car index
    // maybe multithreading will actually become useful
    // simulate max jump height
    // simulate max double jump height
    // add options to individual shots like min/max time
    // add option to tick for max path time
    // add option to change the TPS of the simulation

    game.ball.time = py_time.value(py) as f32;

    unsafe {
        GAME_TIME = game.ball.time.clone();
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

    car.location = get_vec3_from_dict(py, &py_car, "location", "car")?;
    car.velocity = get_vec3_from_dict(py, &py_car, "velocity", "car")?;
    car.angular_velocity = get_vec3_from_dict(py, &py_car, "angular_velocity", "car")?;
    car.hitbox = Hitbox::from_vec3(get_vec3_from_dict(py, &py_car, "hitbox", "car")?);
    car.hitbox_offset = get_vec3_from_dict(py, &py_car, "hitbox_offset", "car")?;
    car.pitch = get_f32_from_dict(py, &py_car, "pitch", "car")?;
    car.yaw = get_f32_from_dict(py, &py_car, "yaw", "car")?;
    car.roll = get_f32_from_dict(py, &py_car, "roll", "car")?;
    car.boost = get_u8_from_dict(py, &py_car, "boost", "car")?;
    car.demolished = get_bool_from_dict(py, &py_car, "demolished", "car")?;
    car.airborne = get_bool_from_dict(py, &py_car, "airborne", "car")?;
    car.jumped = get_bool_from_dict(py, &py_car, "jumped", "car")?;
    car.doublejumped = get_bool_from_dict(py, &py_car, "doublejumped", "car")?;
    car.calculate_orientation_matrix();

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

    let slice = ball_struct.slices[slice_num.clamp(1, ball_struct.num_slices) - 1].clone();

    py_slice.set_item(py, "time", slice.time)?;
    py_slice.set_item(py, "location", get_vec_from_vec3(slice.location))?;
    py_slice.set_item(py, "velocity", get_vec_from_vec3(slice.velocity))?;
    py_slice.set_item(py, "angular_velocity", get_vec_from_vec3(slice.angular_velocity))?;

    Ok(py_slice.into_object())
}

fn calc_dr_and_ft(py: Python, py_target: PyList, py_slice_time: PyFloat) -> PyResult<PyObject> {
    let game_time: &f32;
    let _gravity: &Vec3;
    let car: &Car;
    let ball: Box<Ball>;
    let turn_accel_lut: &TurnLut;
    let turn_accel_boost_lut: &TurnLut;
    let turn_decel_lut: &TurnLut;

    unsafe {
        game_time = &GAME_TIME;
    }

    let slice_num = ((py_slice_time.value(py) as f32 - game_time) * 120.).round() as usize;

    unsafe {
        match GAME.as_ref() {
            Some(game) => {
                _gravity = &game.gravity;
            }
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_GAME_ERR));
            }
        }

        car = match CAR.as_ref() {
            Some(car) => car,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_CAR_ERR));
            }
        };

        ball = match BALL_STRUCT.as_ref() {
            Some(ball_struct) => ball_struct.slices[slice_num.clamp(1, ball_struct.num_slices) - 1].clone(),
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_BALL_STRUCT_ERR));
            }
        };

        turn_accel_lut = match TURN_ACCEL_LUT.as_ref() {
            Some(lut) => lut,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_TURN_ACCEL_LUT_ERR));
            }
        };

        turn_accel_boost_lut = match TURN_ACCEL_BOOST_LUT.as_ref() {
            Some(lut) => lut,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_TURN_ACCEL_BOOST_LUT_ERR));
            }
        };

        turn_decel_lut = match TURN_DECEL_LUT.as_ref() {
            Some(lut) => lut,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_TURN_DECEL_LUT_ERR));
            }
        };
    }

    let target = get_vec3(py, py_target.as_object(), "Key 'target' needs to be a list of exactly 3 numbers")?;

    let shot_vector = (target - ball.location).normalize();

    let (_, distance_remaining, final_target, _, _) = analyze_target(ball, car, shot_vector, turn_accel_lut, turn_accel_boost_lut, turn_decel_lut, false);

    let result = PyDict::new(py);

    result.set_item(py, "distance_remaining", distance_remaining)?;
    result.set_item(py, "final_target", get_vec_from_vec3(final_target))?;

    Ok(result.into_object())
}

fn calculate_intercept(py: Python, py_target: PyList, py_all: PyBool) -> PyResult<PyObject> {
    let game_time: &f32;
    let _gravity: &Vec3;
    let radius: &f32;
    let car: &Car;
    let ball_slices: Vec<Box<Ball>>;
    let turn_accel_lut: &TurnLut;
    let turn_accel_boost_lut: &TurnLut;
    let turn_decel_lut: &TurnLut;

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

        car = match CAR.as_ref() {
            Some(car) => car,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_CAR_ERR));
            }
        };

        ball_slices = match BALL_STRUCT.as_ref() {
            Some(ball_struct) => ball_struct.slices.clone(),
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_BALL_STRUCT_ERR));
            }
        };

        turn_accel_lut = match TURN_ACCEL_LUT.as_ref() {
            Some(lut) => lut,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_TURN_ACCEL_LUT_ERR));
            }
        };

        turn_accel_boost_lut = match TURN_ACCEL_BOOST_LUT.as_ref() {
            Some(lut) => lut,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_TURN_ACCEL_BOOST_LUT_ERR));
            }
        };

        turn_decel_lut = match TURN_DECEL_LUT.as_ref() {
            Some(lut) => lut,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_TURN_DECEL_LUT_ERR));
            }
        };

        game_time = &GAME_TIME;
    }

    let target = get_vec3(py, py_target.as_object(), "Key 'target' needs to be a list of exactly 3 numbers")?;

    let all = py_all.is_true();

    let dist_from_side = radius + car.hitbox.height + 17.;

    let mut found_ball = None;

    if ball_slices.len() == 0 || car.demolished {
        let result = PyDict::new(py);
        result.set_item(py, "found", false)?;
        return Ok(result.into_object());
    }

    for ball in ball_slices {
        if ball.location.y.abs() > 5120. + ball.collision_radius {
            break;
        }

        if ball.location.z > dist_from_side {
            continue;
        }

        let shot_vector = (target - ball.location).normalize();

        let (valid, distance_remaining, _, turn, turn_info) = analyze_target(ball.clone(), car, shot_vector, turn_accel_lut, turn_accel_boost_lut, turn_decel_lut, true);

        if !valid {
            continue;
        }

        let mut time_remaining = ball.time - game_time;

        if turn {
            time_remaining -= turn_info.time;

            if time_remaining <= 0. {
                continue;
            }
        }

        // let (found, _) = can_reach_target(car, time_remaining, distance_remaining, true);
        let found = distance_remaining / time_remaining < MAX_SPEED;

        if all {
            // for worst-case benchmarking only
            // returns accurate numbers, but analyzes all slices
            if found && found_ball.is_none() {
                found_ball = Some(ball.clone());
            }
        } else {
            if found {
                found_ball = Some(ball.clone());
                break;
            }
        }
    }

    let result = PyDict::new(py);

    match found_ball {
        Some(box_slice) => {
            result.set_item(py, "found", true)?;
            result.set_item(py, "time", box_slice.time)?;
        }
        None => {
            result.set_item(py, "found", false)?;
        }
    }

    Ok(result.into_object())
}
