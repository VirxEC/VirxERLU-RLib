extern crate cpython;
extern crate rl_ball_sym;

mod utils;

use std::{cmp::Ordering, f32::consts::FRAC_PI_2};

use cpython::{exc, py_fn, py_module_initializer, PyDict, PyErr, PyFloat, PyList, PyObject, PyResult, Python, PythonObject};
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
    m.add(py, "calculate_intercept", py_fn!(py, calculate_intercept(target: PyList)))?;
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
        BALL_STRUCT = Some(Ball::get_ball_prediction_struct_for_time(game, &8.));
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

    let mut offset_target = ball.location - (shot_vector * ball.radius);

    let angle_to_target = angle_2d(&shot_vector, &car.forward);

    let final_target: Vec3;
    let mut distance_remaining = -car.hitbox.length / 2.;

    // let mut debug_stuff = Vec::new();

    // debug_stuff.push(get_vec_from_vec3(car.location));
    // debug_stuff.push(get_vec_from_vec3(offset_target));

    if angle_to_target < MIN_ADJUST_RADIANS + NO_ADJUST_RADIANS || offset_target.dot(&car.right).abs() <= 10. {
        if offset_target.dot(&car.right).abs() > 10. || offset_target.dot(&car.forward) < 0. {
            //angle_to_target > NO_ADJUST_RADIANS_12K {
            let car_to_ball = (ball.location - car.location).normalize();
            let side_of_shot = shot_vector
                .cross(&Vec3 {
                    x: 0.,
                    y: 0.,
                    z: 1.,
                })
                .dot(&car_to_ball)
                .signum();
            let car_to_offset_target = offset_target - car.location;
            let car_to_offset_perp = car_to_offset_target.cross(&Vec3 {
                x: 0.,
                y: 0.,
                z: side_of_shot,
            });

            offset_target += car_to_offset_perp.scale(car.hitbox.width / 2.);
        }

        if angle_tau_2d(&car.forward, &(offset_target - car.location)) > NO_ADJUST_RADIANS {
            let turn_info = TurnInfo::calc_turn_info(car, &offset_target, turn_accel_lut, turn_accel_boost_lut, turn_decel_lut);
            // debug_stuff.push(get_vec_from_vec3(turn_info.car_location));
            distance_remaining += turn_info.distance as f32 + dist_2d(&turn_info.car_location, &offset_target);
        } else {
            distance_remaining += dist_2d(&car.location, &offset_target);
        }

        final_target = offset_target;
    } else {
        let inv_shot_vector;
        let exit_turn_point = {
            let og_inv_shot_vector = -shot_vector;

            let rotation = FRAC_PI_2 - MIN_ADJUST_RADIANS;
            let og_inv_shot_vector_adjusts = [rotate_2d(&og_inv_shot_vector, &rotation), rotate_2d(&og_inv_shot_vector, &-rotation)];

            let inv_shot_vectors = [rotate_2d(&-og_inv_shot_vector_adjusts[0], &FRAC_PI_2), rotate_2d(&-og_inv_shot_vector_adjusts[1], &-FRAC_PI_2)];

            let offset_targets = [offset_target + og_inv_shot_vector_adjusts[0] * car.hitbox.width / 2., offset_target + og_inv_shot_vector_adjusts[1] * car.hitbox.width / 2.];

            let exit_turn_points = [offset_targets[0] + (flattened(&inv_shot_vectors[0]) * 640.), offset_targets[1] + (flattened(&inv_shot_vectors[1]) * 640.)];

            let index;

            // yes, this is ugly
            // but it works
            // deal with it
            if offset_targets[0].dot(&car.right).abs() < 10. {
                index = 0;
            } else if offset_targets[1].dot(&car.right).abs() < 10. {
                index = 1;
            } else {
                index = if incomplete_dist_2d(&exit_turn_points[0], &car.location) < incomplete_dist_2d(&exit_turn_points[1], &car.location) {
                    0
                } else {
                    1
                };
            }

            inv_shot_vector = inv_shot_vectors[index];
            offset_target = offset_targets[index];
            exit_turn_points[index]
        };

        // debug_stuff.push(get_vec_from_vec3(exit_turn_point));

        let mut inv_shot_vector_perp = rotate_2d(&inv_shot_vector, &FRAC_PI_2);
        let mut side = false;

        let inv_shot_vector_perp_2 = rotate_2d(&inv_shot_vector, &-FRAC_PI_2);

        if incomplete_dist_2d(&(inv_shot_vector_perp_2 + exit_turn_point), &car.location) < incomplete_dist_2d(&(inv_shot_vector_perp + exit_turn_point), &car.location) {
            inv_shot_vector_perp = inv_shot_vector_perp_2;
            side = true;
        }

        let circle_center = exit_turn_point + inv_shot_vector_perp * MAX_TURN_RADIUS;

        // debug_stuff.push(get_vec_from_vec3(circle_center));

        let cc_to_cl = car.location - circle_center;

        let tangent_angle = (MAX_TURN_RADIUS / cc_to_cl.magnitude()).clamp(-1., 1.).acos();
        let cc_to_cl = cc_to_cl.normalize();

        let tangent_lines = [rotate_2d(&cc_to_cl, &-tangent_angle), rotate_2d(&cc_to_cl, &tangent_angle)];

        let cc_to_etp = (exit_turn_point - circle_center).normalize();
        let enter_turn_line;

        if side {
            enter_turn_line = tangent_lines.iter().max_by(|a, b| angle_tau_2d(&cc_to_etp, a).partial_cmp(&angle_tau_2d(&cc_to_etp, b)).unwrap_or(Ordering::Equal)).unwrap();
        } else {
            enter_turn_line = tangent_lines.iter().min_by(|a, b| angle_tau_2d(&cc_to_etp, a).partial_cmp(&angle_tau_2d(&cc_to_etp, b)).unwrap_or(Ordering::Equal)).unwrap();
        }

        let enter_turn_point = *enter_turn_line * MAX_TURN_RADIUS + circle_center;

        // debug_stuff.push(get_vec_from_vec3(enter_turn_point));

        let turn_angle = angle_tau_2d(&enter_turn_line, &cc_to_etp);
        let turn_distance_remaining = turn_angle * MAX_TURN_RADIUS;

        distance_remaining += turn_distance_remaining + dist_2d(&exit_turn_point, &offset_target);

        if dist_2d(&car.location, &enter_turn_point) > car.hitbox.width / 2. {
            let turn_info = TurnInfo::calc_turn_info(car, &enter_turn_point, turn_accel_lut, turn_accel_boost_lut, turn_decel_lut);
            // debug_stuff.push(get_vec_from_vec3(turn_info.car_location));
            distance_remaining += turn_info.distance as f32 + dist_2d(&turn_info.car_location, &enter_turn_point);
            final_target = enter_turn_point;
        } else {
            final_target = exit_turn_point;
        }
    }

    // debug_stuff.push(get_vec_from_vec3(offset_target));

    let result = PyDict::new(py);

    result.set_item(py, "distance_remaining", distance_remaining)?;
    result.set_item(py, "final_target", get_vec_from_vec3(final_target))?;
    // result.set_item(py, "debug_stuff", debug_stuff)?;

    Ok(result.into_object())
}

fn calculate_intercept(py: Python, py_target: PyList) -> PyResult<PyObject> {
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

        let mut offset_target = ball.location - (shot_vector * ball.radius);

        let angle_to_target = angle_2d(&(offset_target - car.location).normalize(), &car.forward);

        let mut distance_remaining = -car.hitbox.length / 2.;

        let mut turn_info = TurnInfo::default();
        let mut turn = false;

        if angle_to_target < MIN_ADJUST_RADIANS + NO_ADJUST_RADIANS || offset_target.dot(&car.right).abs() <= 10. {
            if offset_target.dot(&car.right).abs() > 10. || offset_target.dot(&car.forward) < 0. {
                //angle_to_target > NO_ADJUST_RADIANS_12K {
                let car_to_ball = (ball.location - car.location).normalize();
                let side_of_shot = shot_vector
                    .cross(&Vec3 {
                        x: 0.,
                        y: 0.,
                        z: 1.,
                    })
                    .dot(&car_to_ball)
                    .signum();
                let car_to_offset_target = offset_target - car.location;
                let car_to_offset_perp = car_to_offset_target.cross(&Vec3 {
                    x: 0.,
                    y: 0.,
                    z: side_of_shot,
                });

                offset_target += car_to_offset_perp.scale(car.hitbox.width / 2.);
            }

            if angle_tau_2d(&car.forward, &(offset_target - car.location)) > NO_ADJUST_RADIANS {
                turn_info = TurnInfo::calc_turn_info(car, &offset_target, turn_accel_lut, turn_accel_boost_lut, turn_decel_lut);
                distance_remaining += dist_2d(&turn_info.car_location, &offset_target);
                turn = true;
            } else {
                distance_remaining += dist_2d(&car.location, &offset_target);
            }
        } else {
            let inv_shot_vector;
            let exit_turn_point = {
                let og_inv_shot_vector = -shot_vector;

                let rotation = FRAC_PI_2 - MIN_ADJUST_RADIANS;
                let og_inv_shot_vector_adjusts = [rotate_2d(&og_inv_shot_vector, &rotation), rotate_2d(&og_inv_shot_vector, &-rotation)];

                let inv_shot_vectors = [rotate_2d(&-og_inv_shot_vector_adjusts[0], &FRAC_PI_2), rotate_2d(&-og_inv_shot_vector_adjusts[1], &-FRAC_PI_2)];

                let offset_targets = [offset_target + og_inv_shot_vector_adjusts[0] * car.hitbox.width / 2., offset_target + og_inv_shot_vector_adjusts[1] * car.hitbox.width / 2.];

                let exit_turn_points = [offset_targets[0] + (flattened(&inv_shot_vectors[0]) * 640.), offset_targets[1] + (flattened(&inv_shot_vectors[1]) * 640.)];

                let index;

                // yes, this is ugly
                // but it works
                // deal with it
                if offset_targets[0].dot(&car.right).abs() < 10. {
                    index = 0;
                } else if offset_targets[1].dot(&car.right).abs() < 10. {
                    index = 1;
                } else {
                    index = if incomplete_dist_2d(&exit_turn_points[0], &car.location) < incomplete_dist_2d(&exit_turn_points[1], &car.location) {
                        0
                    } else {
                        1
                    };
                }

                inv_shot_vector = inv_shot_vectors[index];
                offset_target = offset_targets[index];
                exit_turn_points[index]
            };

            let mut inv_shot_vector_perp = rotate_2d(&inv_shot_vector, &FRAC_PI_2);
            let mut side = false;

            let inv_shot_vector_perp_2 = rotate_2d(&inv_shot_vector, &-FRAC_PI_2);

            if incomplete_dist_2d(&(inv_shot_vector_perp_2 + exit_turn_point), &car.location) < incomplete_dist_2d(&(inv_shot_vector_perp + exit_turn_point), &car.location) {
                inv_shot_vector_perp = inv_shot_vector_perp_2;
                side = true;
            }

            let circle_center = exit_turn_point + inv_shot_vector_perp * MAX_TURN_RADIUS;

            if !is_circle_in_field(car, circle_center, MAX_TURN_RADIUS) {
                continue;
            }

            let cc_to_cl = car.location - circle_center;

            let tangent_angle = (MAX_TURN_RADIUS / cc_to_cl.magnitude()).clamp(-1., 1.).acos();
            let cc_to_cl = cc_to_cl.normalize();

            let tangent_lines = [rotate_2d(&cc_to_cl, &-tangent_angle), rotate_2d(&cc_to_cl, &tangent_angle)];

            let cc_to_etp = (exit_turn_point - circle_center).normalize();
            let enter_turn_line;

            if side {
                enter_turn_line = tangent_lines.iter().max_by(|a, b| angle_tau_2d(&cc_to_etp, a).partial_cmp(&angle_tau_2d(&cc_to_etp, b)).unwrap_or(Ordering::Equal)).unwrap();
            } else {
                enter_turn_line = tangent_lines.iter().min_by(|a, b| angle_tau_2d(&cc_to_etp, a).partial_cmp(&angle_tau_2d(&cc_to_etp, b)).unwrap_or(Ordering::Equal)).unwrap();
            }

            let enter_turn_point = *enter_turn_line * MAX_TURN_RADIUS + circle_center;
            let turn_angle = angle_tau_2d(&enter_turn_line, &cc_to_etp);
            let turn_distance_remaining = turn_angle * MAX_TURN_RADIUS;

            distance_remaining += turn_distance_remaining + dist_2d(&exit_turn_point, &offset_target);

            if dist_2d(&car.location, &enter_turn_point) > car.hitbox.width / 2. {
                turn_info = TurnInfo::calc_turn_info(car, &enter_turn_point, turn_accel_lut, turn_accel_boost_lut, turn_decel_lut);
                turn = true;

                distance_remaining += dist_2d(&turn_info.car_location, &enter_turn_point);
            }
        }

        let mut time_remaining = ball.time - game_time;

        if turn {
            time_remaining -= turn_info.time;

            if time_remaining <= 0. {
                continue;
            }
        }

        let found = distance_remaining / time_remaining < MAX_SPEED;

        if found {
            found_ball = Some(ball.clone());
            break;
        }

        // for worst-case benchmarking only
        // returns accurate numbers, but analyzes all slices
        // if found && found_ball.is_none() {
        //     found_ball = Some(ball.clone());
        // }
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
