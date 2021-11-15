extern crate cpython;
extern crate rl_ball_sym;

mod utils;

use cpython::{exc, py_fn, py_module_initializer, PyBool, PyDict, PyErr, PyFloat, PyInt, PyObject, PyResult, PyTuple, Python, PythonObject};
use dubins_paths::path_sample_many;
use rl_ball_sym::simulation::{
    ball::{Ball, BallPrediction},
    game::Game,
};
use utils::*;
use vvec3::Vec3;

static mut GAME_TIME: f32 = 0.;

static mut GAME: Option<Game> = None;
const NO_GAME_ERR: &str = "GAME is unset. Call a function like load_soccar first.";

static mut CARS: Option<Vec<Car>> = None;
const NO_CARS_ERR: &str = "CAR is unset. Call a function like load_soccar first.";

static mut BALL_STRUCT: Option<BallPrediction> = None;
const NO_BALL_STRUCT_ERR: &str = "BALL_STRUCT is unset. Call the function tick and pass in game information first.";

py_module_initializer!(virxrlru, |py, m| {
    m.add(py, "__doc__", "VirxERLU-RLib is written in Rust with Python bindings to make analyzing the ball prediction struct much faster.")?;
    m.add(py, "load_soccar", py_fn!(py, load_soccar()))?;
    m.add(py, "load_dropshot", py_fn!(py, load_dropshot()))?;
    m.add(py, "load_hoops", py_fn!(py, load_hoops()))?;
    m.add(py, "set_gravity", py_fn!(py, set_gravity(gravity: PyDict)))?;
    m.add(py, "tick", py_fn!(py, tick(time: PyFloat, ball: PyDict, car: PyDict)))?;
    m.add(py, "get_slice", py_fn!(py, get_slice(time: PyFloat)))?;
    m.add(py, "get_data_for_shot_with_target", py_fn!(py, get_data_for_shot_with_target(target_left: PyTuple, target_right: PyTuple, time: PyFloat, py_index: PyInt)))?;
    m.add(py, "get_shot_with_target", py_fn!(py, get_shot_with_target(target_left: PyTuple, target_right: PyTuple, py_index: PyInt, all: PyBool)))?;
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
    let cars: &mut Vec<Car>;

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

    let slice = ball_struct.slices[slice_num.clamp(1, ball_struct.num_slices) - 1].clone();

    py_slice.set_item(py, "time", slice.time)?;
    py_slice.set_item(py, "location", get_vec_from_vec3(slice.location))?;
    py_slice.set_item(py, "velocity", get_vec_from_vec3(slice.velocity))?;
    py_slice.set_item(py, "angular_velocity", get_vec_from_vec3(slice.angular_velocity))?;

    Ok(py_slice.into_object())
}

fn get_data_for_shot_with_target(py: Python, py_target_left: PyTuple, py_target_right: PyTuple, py_slice_time: PyFloat, py_index: PyInt) -> PyResult<PyObject> {
    let game_time: &f32;
    let _gravity: &Vec3;
    let car: &Car;
    let ball: &Ball;

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

        car = match &mut CARS {
            Some(cars) => cars.get_mut(py_index.into_object().extract::<usize>(py).unwrap()).unwrap(),
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_CARS_ERR));
            }
        };

        ball = match BALL_STRUCT.as_ref() {
            Some(ball_struct) => &ball_struct.slices[slice_num.clamp(1, ball_struct.num_slices) - 1],
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_BALL_STRUCT_ERR));
            }
        };
    }

    let target_left = get_vec3(py, py_target_left.as_object(), "Key 'target_left' needs to be a list of exactly 3 numbers")?;
    let target_right = get_vec3(py, py_target_right.as_object(), "Key 'target_right' needs to be a list of exactly 3 numbers")?;

    let car_to_ball = ball.location - car.location;
    let post_info = correct_for_posts(ball.location, ball.collision_radius, target_left, target_right);
    let shot_vector = get_shot_vector_2d(car_to_ball.flatten().normalize(), ball.location.flatten(), post_info.target_left.flatten(), post_info.target_right.flatten());

    let (distance_parts, final_target, face_shot_vector, path) = match analyze_target(ball, car, shot_vector, 0., true, false) {
        Ok(result) => (result.0, result.1.unwrap(), result.2, result.3),
        Err(duberr) => {
            return Err(PyErr::new::<exc::Exception, _>(py, format!("{:?} - Couldn't calculate path", duberr)));
        }
    };

    let result = PyDict::new(py);

    // dbg!(distance_parts);

    let distance_remaining: f32 = distance_parts.iter().sum();

    result.set_item(py, "distance_remaining", distance_remaining)?;
    result.set_item(py, "final_target", get_vec_from_vec3(final_target))?;
    result.set_item(py, "shot_vector", get_vec_from_vec3(shot_vector))?;
    result.set_item(py, "face_shot_vector", face_shot_vector)?;

    match path {
        Some(path_) => {
            let path_samples = match path_sample_many(&path_, 50.) {
                Ok(raw_samples) => {
                    let mut samples = Vec::with_capacity(raw_samples.len());
                    for sample in raw_samples {
                        samples.push(vec![sample[0], sample[1]]);
                    }
                    samples
                }
                Err(duberr) => {
                    return Err(PyErr::new::<exc::Exception, _>(py, format!("{:?} - Couldn't calculate samples", duberr)));
                }
            };
            result.set_item(py, "path_samples", path_samples)?;
        }
        None => {
            result.set_item(py, "path_samples", vec![get_vec_from_vec3(car.location), get_vec_from_vec3(ball.location)])?;
        }
    }

    Ok(result.into_object())
}

fn get_shot_with_target(py: Python, py_target_left: PyTuple, py_target_right: PyTuple, py_index: PyInt, py_all: PyBool) -> PyResult<PyObject> {
    let game_time: &f32;
    let _gravity: &Vec3;
    let radius: &f32;
    let car: &Car;
    let ball_prediction: &BallPrediction;

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

        car = match &mut CARS {
            Some(cars) => cars.get_mut(py_index.into_object().extract::<usize>(py).unwrap()).unwrap(),
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_CARS_ERR));
            }
        };

        ball_prediction = match BALL_STRUCT.as_ref() {
            Some(ball_struct) => ball_struct,
            None => {
                return Err(PyErr::new::<exc::NameError, _>(py, NO_BALL_STRUCT_ERR));
            }
        };

        game_time = &GAME_TIME;
    }

    let target_left = get_vec3(py, py_target_left.as_object(), "Key 'target_left' needs to be a list of exactly 3 numbers")?;
    let target_right = get_vec3(py, py_target_right.as_object(), "Key 'target_right' needs to be a list of exactly 3 numbers")?;

    let all = py_all.is_true();

    let dist_from_side = radius + car.hitbox.height;

    let mut found_ball = None;

    if ball_prediction.num_slices == 0 || car.demolished || car.airborne {
        let result = PyDict::new(py);
        result.set_item(py, "found", false)?;
        return Ok(result.into_object());
    }

    for ball in &ball_prediction.slices {
        if ball.location.y.abs() > 5120. + ball.collision_radius {
            break;
        }

        if ball.location.z > dist_from_side {
            continue;
        }

        let car_to_ball = ball.location - car.location;

        let post_info = correct_for_posts(ball.location, ball.collision_radius, target_left, target_right);

        if !post_info.fits {
            continue;
        }

        let shot_vector = get_shot_vector_2d(car_to_ball.flatten().normalize(), ball.location.flatten(), post_info.target_left.flatten(), post_info.target_right.flatten());
        // dbg!(shot_vector);

        let time_remaining = ball.time - game_time;

        let distance_parts = match analyze_target(ball, car, shot_vector, time_remaining, false, true) {
            Ok(result) => result.0,
            Err(_) => continue,
        };

        // will be used to calculate if there's enough time left to jump after accelerating
        let _time_remaining = match can_reach_target(car, time_remaining, distance_parts.iter().sum(), true) {
            Ok(t_r) => t_r,
            Err(_) => continue,
        };

        if !all {
            found_ball = Some(ball.clone());
            break;
        } else if found_ball.is_none() {
            // for worst-case benchmarking only
            // returns accurate numbers, but analyzes all slices
            found_ball = Some(ball.clone());
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
