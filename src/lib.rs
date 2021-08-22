extern crate cpython;
extern crate rl_ball_sym;

mod utils;

use crate::utils::{get_vec3, Car};
use cpython::{exc, py_fn, py_module_initializer, PyBool, PyDict, PyErr, PyFloat, PyList, PyObject, PyResult, Python, PythonObject};
use rl_ball_sym::simulation::{
    ball::{Ball, BallPrediction},
    game::Game,
};
// use std::{thread, time};
use utils::{Hitbox, Slice, angle, generate_hierarchy_from_vec, get_distance_remaining, incomplete_dist_2d};

static mut GAME: Option<Game> = None;
static NO_GAME_ERR: &str = "GAME is unset. Call a function like load_soccar first.";

static mut CAR: Option<Car> = None;
static NO_CAR_ERR: &str = "CAR is unset. Call a function like load_soccar first.";

static mut BALL_STRUCT: Option<BallPrediction> = None;
static NO_BALL_STRUCT_ERR: &str = "BALL_STRUCT is unset. Call the function tick and pass in game information first.";

static mut USE_BINARY_TREE: bool = false;

py_module_initializer!(virxrlru, |py, m| {
    m.add(py, "__doc__", "VirxERLU-RLib is written in Rust with Python bindings to make analyzing the ball prediction struct much faster.")?;
    m.add(py, "load_soccar", py_fn!(py, load_soccar()))?;
    m.add(py, "load_dropshot", py_fn!(py, load_dropshot()))?;
    m.add(py, "load_hoops", py_fn!(py, load_hoops()))?;
    m.add(py, "set_gravity", py_fn!(py, set_gravity(gravity: PyDict)))?;
    m.add(py, "tick", py_fn!(py, tick(time: PyFloat, ball: PyDict, car: PyDict)))?;
    m.add(py, "use_binary_search", py_fn!(py, use_binary_tree(use_: PyBool)))?;
    m.add(py, "calculate_intercept", py_fn!(py, calculate_intercept(target: PyList)))?;
    Ok(())
});

fn load_soccar(py: Python) -> PyResult<PyObject> {
    unsafe {
        GAME = Some(rl_ball_sym::load_soccar());
        CAR = Some(Car::default());
        BALL_STRUCT = Some(BallPrediction::default());
    }

    Ok(py.None())
}

fn load_dropshot(py: Python) -> PyResult<PyObject> {
    unsafe {
        GAME = Some(rl_ball_sym::load_dropshot());
        CAR = Some(Car::default());
        BALL_STRUCT = Some(BallPrediction::default());
    }

    Ok(py.None())
}

fn load_hoops(py: Python) -> PyResult<PyObject> {
    unsafe {
        GAME = Some(rl_ball_sym::load_hoops());
        CAR = Some(Car::default());
        BALL_STRUCT = Some(BallPrediction::default());
    }

    Ok(py.None())
}

fn set_gravity(py: Python, py_gravity: PyDict) -> PyResult<PyObject> {
    let mut game: &mut Game;

    unsafe {
        if GAME.is_none() {
            return Err(PyErr::new::<exc::NameError, _>(py, NO_GAME_ERR));
        }

        game = GAME.as_mut().unwrap();
    }

    game.gravity = get_vec3(py, py_gravity.as_object())?;

    Ok(py.None())
}

fn tick(py: Python, py_time: PyFloat, py_ball: PyDict, py_car: PyDict) -> PyResult<PyObject> {
    let mut game: &mut Game;
    let mut car: &mut Car;

    unsafe {
        if GAME.is_none() {
            return Err(PyErr::new::<exc::NameError, _>(py, NO_GAME_ERR));
        }

        game = GAME.as_mut().unwrap();

        if CAR.is_none() {
            return Err(PyErr::new::<exc::NameError, _>(py, NO_CAR_ERR));
        }

        car = CAR.as_mut().unwrap();
    }

    game.ball.time = py_time.value(py) as f32;

    match py_ball.get_item(py, "location") {
        Some(location) => {
            game.ball.location = get_vec3(py, &location)?;
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, "No attribute called 'location' in 'ball'."));
        }
    }

    match py_ball.get_item(py, "velocity") {
        Some(velocity) => {
            game.ball.velocity = get_vec3(py, &velocity)?;
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, "No attribute called 'velocity' in 'ball'."));
        }
    }

    match py_ball.get_item(py, "angular_velocity") {
        Some(angular_velocity) => {
            game.ball.angular_velocity = get_vec3(py, &angular_velocity)?;
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, "No attribute called 'angular_velocity' in 'ball'."));
        }
    }

    if let Some(radius) = py_ball.get_item(py, "radius") {
        game.ball.radius = radius.extract(py)?;
        game.ball.calculate_moi();
    }

    if let Some(collision_radius) = py_ball.get_item(py, "collision_radius") {
        game.ball.collision_radius = collision_radius.extract(py)?;
    }

    match py_car.get_item(py, "location") {
        Some(location) => {
            car.location = get_vec3(py, &location)?;
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, "No attribute called 'location' in 'car'."));
        }
    }

    match py_car.get_item(py, "velocity") {
        Some(velocity) => {
            car.velocity = get_vec3(py, &velocity)?;
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, "No attribute called 'velocity' in 'car'."));
        }
    }

    match py_car.get_item(py, "angular_velocity") {
        Some(angular_velocity) => {
            car.angular_velocity = get_vec3(py, &angular_velocity)?;
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, "No attribute called 'angular_velocity' in 'car'."));
        }
    }

    match py_car.get_item(py, "hitbox") {
        Some(hitbox) => {
            car.hitbox = Hitbox::from_vec3(get_vec3(py, &hitbox)?);
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, "No attribute called 'hitbox' in 'car'."));
        }
    }

    match py_car.get_item(py, "pitch") {
        Some(pitch) => {
            car.pitch = pitch.extract(py)?;
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, "No attribute called 'pitch' in 'car'."));
        }
    }

    match py_car.get_item(py, "yaw") {
        Some(yaw) => {
            car.yaw = yaw.extract(py)?;
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, "No attribute called 'yaw' in 'car'."));
        }
    }

    match py_car.get_item(py, "roll") {
        Some(roll) => {
            car.roll = roll.extract(py)?;
        }
        None => {
            return Err(PyErr::new::<exc::AttributeError, _>(py, "No attribute called 'roll' in 'car'."));
        }
    }

    car.calculate_orientation_matrix();

    unsafe {
        BALL_STRUCT = Some(Ball::get_ball_prediction_struct_for_time(game, &8.));
    }

    Ok(py.None())
}

fn use_binary_tree(py: Python, py_bool: PyBool) -> PyResult<PyObject> {
    unsafe {
        USE_BINARY_TREE = py_bool.is_true();
    }

    Ok(py.None())
}

fn calculate_intercept(py: Python, py_target: PyList) -> PyResult<PyObject> {
    let use_binary_tree: &bool;
    let game: &Game;
    let car: &Car;
    let mut ball_slices: Vec<Box<Ball>>;

    unsafe {
        if GAME.is_none() {
            return Err(PyErr::new::<exc::NameError, _>(py, NO_GAME_ERR));
        }

        game = GAME.as_ref().unwrap();

        if CAR.is_none() {
            return Err(PyErr::new::<exc::NameError, _>(py, NO_CAR_ERR));
        }

        car = CAR.as_ref().unwrap();

        if BALL_STRUCT.is_none() {
            return Err(PyErr::new::<exc::NameError, _>(py, NO_BALL_STRUCT_ERR));
        }

        ball_slices = BALL_STRUCT.as_ref().unwrap().slices.clone();

        use_binary_tree = &USE_BINARY_TREE;
    }

    let target = get_vec3(py, py_target.as_object())?;

    let dist_from_side = game.ball.collision_radius + car.hitbox.height + 17.;

    let mut found_ball = None;
    let mut found_slice = None;

    if ball_slices.len() != 0 {
        if *use_binary_tree {
            let mut passed_goal = false;

            // filter out any impossibilities
            ball_slices = ball_slices
                .into_iter()
                .filter(|ball| {
                    if !passed_goal {
                        passed_goal = ball.location.y.abs() > 5120. + ball.collision_radius;
                    }

                    if passed_goal || ball.location.z > dist_from_side {
                        return false;
                    }

                    let shot_vector = (target - ball.location).normalize();
                    let car_to_ball = (ball.location - car.location).normalize();

                    return angle(&shot_vector, &car_to_ball) < 1.8;
                })
                .collect();

            // convert vector of boxed balls to a vector of slices
            let mut sorted_slices: Vec<Box<Slice>> = ball_slices
                .iter()
                .map(|ball| {
                    Box::new(Slice {
                        ball: ball.clone(),
                        distance: incomplete_dist_2d(&ball.location, &car.location),
                        up: None,
                        down: None,
                    })
                })
                .collect();

            // sort the slices by distance
            sorted_slices.sort_unstable_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());

            // construct the binary tree
            let mut slice = generate_hierarchy_from_vec(sorted_slices).unwrap();

            // traverse the binary tree
            loop {
                let ball = slice.ball.clone();

                let shot_vector = (target - ball.location).normalize();

                let found = get_distance_remaining(ball.clone(), car, &shot_vector) / ball.time < 1600.;

                // thread::sleep(time::Duration::from_nanos(1));

                if slice.down.is_none() && slice.up.is_none() {
                    break;
                } else if found {
                    found_slice = Some(slice.clone());
                    match slice.down {
                        Some(down) => slice = down,
                        None => break,
                    }
                }
            }
        } else {
            for ball in ball_slices {
                if ball.location.y.abs() > 5120. + ball.collision_radius {
                    break;
                }

                let shot_vector = (target - ball.location).normalize();
                let car_to_ball = (ball.location - car.location).normalize();

                if angle(&shot_vector, &car_to_ball) > 1.8 {
                    continue;
                }

                let found = get_distance_remaining(ball.clone(), car, &shot_vector) / ball.time < 1600.;
                // thread::sleep(time::Duration::from_nanos(1));

                if found {
                    found_ball = Some(ball.clone());
                }
            }
        }
    }

    let result = PyDict::new(py);

    if *use_binary_tree {
        match found_slice {
            Some(box_slice) => {
                result.set_item(py, "found", true)?;
                result.set_item(py, "time", box_slice.ball.time)?;
            }
            None => {
                result.set_item(py, "found", false)?;
            }
        }
    } else {
        match found_ball {
            Some(box_slice) => {
                result.set_item(py, "found", true)?;
                result.set_item(py, "time", box_slice.time)?;
            }
            None => {
                result.set_item(py, "found", false)?;
            }
        }
    }

    return Ok(result.into_object());
}
