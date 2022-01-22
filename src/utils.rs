use pyo3::{exceptions, types::PyDict, PyAny, PyErr, PyObject, PyResult, Python};

use glam::Vec3A;

pub fn get_vec3(py_vec: &PyAny, too_few_vals_err_msg: String) -> PyResult<Vec3A> {
    let mut vec = Vec3A::ZERO;

    if let Ok(x) = py_vec.get_item::<usize>(0) {
        vec.x = x.extract()?;
    } else {
        return Err(PyErr::new::<exceptions::PyIndexError, _>(too_few_vals_err_msg));
    }

    if let Ok(y) = py_vec.get_item::<usize>(0) {
        vec.y = y.extract()?;
    } else {
        return Err(PyErr::new::<exceptions::PyIndexError, _>(too_few_vals_err_msg));
    }

    if let Ok(z) = py_vec.get_item::<usize>(0) {
        vec.z = z.extract()?;
    } else {
        return Err(PyErr::new::<exceptions::PyIndexError, _>(too_few_vals_err_msg));
    }

    Ok(vec)
}

pub fn get_vec3_named(py: Python, py_vec: PyObject) -> PyResult<Vec3A> {
    Ok(Vec3A::new(py_vec.getattr(py, "x")?.extract(py)?, py_vec.getattr(py, "y")?.extract(py)?, py_vec.getattr(py, "z")?.extract(py)?))
}

pub fn get_vec3_from_dict(py_dict: &PyDict, key: &str, name: &str) -> PyResult<Vec3A> {
    match py_dict.get_item(key) {
        Some(py_arr) => Ok(get_vec3(py_arr, format!("Key '{}' in '{}' needs to be a list of exactly 3 numbers", key, name))?),
        None => Err(PyErr::new::<exceptions::PyAttributeError, _>(format!("No key called '{}' in '{}'.", key, name))),
    }
}

pub fn get_f32_from_dict(py_dict: &PyDict, key: &str, name: &str) -> PyResult<f32> {
    match py_dict.get_item(key) {
        Some(py_num) => Ok(py_num.extract()?),
        None => Err(PyErr::new::<exceptions::PyAttributeError, _>(format!("No key called '{}' in '{}'.", key, name))),
    }
}

pub fn get_usize_from_dict(py_dict: &PyDict, key: &str, name: &str) -> PyResult<usize> {
    match py_dict.get_item(key) {
        Some(py_num) => Ok(py_num.extract()?),
        None => Err(PyErr::new::<exceptions::PyAttributeError, _>(format!("No key called '{}' in '{}'.", key, name))),
    }
}

pub fn get_u8_from_dict(py_dict: &PyDict, key: &str, name: &str) -> PyResult<u8> {
    match py_dict.get_item(key) {
        Some(py_num) => Ok(py_num.extract()?),
        None => Err(PyErr::new::<exceptions::PyAttributeError, _>(format!("No key called '{}' in '{}'.", key, name))),
    }
}

pub fn get_bool_from_dict(py_dict: &PyDict, key: &str, name: &str) -> PyResult<bool> {
    match py_dict.get_item(key) {
        Some(py_bool) => Ok(py_bool.is_true()?),
        None => Err(PyErr::new::<exceptions::PyAttributeError, _>(format!("No key called '{}' in '{}'.", key, name))),
    }
}

pub fn get_vec3_from_vec(vec: Vec<f32>, name: &str) -> PyResult<Vec3A> {
    if vec.len() != 3 {
        Err(PyErr::new::<exceptions::PyIndexError, _>(format!("Key '{}' needs to be a list of exactly 3 numbers", name)))
    } else {
        Ok(Vec3A::new(vec[0], vec[1], vec[2]))
    }
}

pub fn get_vec_from_vec3(vec: Vec3A) -> Vec<f32> {
    vec![vec.x, vec.y, vec.z]
}

// pub fn lerp<T: Copy + Add<Output = T> + Sub<Output = T> + Mul<f32, Output = T>>(a: T, b: T, t: f32) -> T {
//     // Linearly interpolate from a to b using t
//     // For instance, when t == 0, a is returned, and when t is 1, b is returned
//     // Works for both numbers and Vectors
//     (b - a) * t + a
// }

// fn invlerp<T: Copy + Sub<Output = T> + Div<Output = f32>>(a: T, b: T, v: T) -> f32 {
//     // Inverse linear interpolation from a to b with value v
//     // For instance, it returns 0 if v is a, and returns 1 if v is b, and returns 0.5 if v is exactly between a and b
//     // Works for both numbers and Vectors
//     (v - a) / (b - a)
// }

fn clamp_2d(vec: Vec3A, start: Vec3A, end: Vec3A) -> Vec3A {
    let s = vec.normalize_or_zero();
    let right = s.dot(end.cross(-Vec3A::Z)) < 0.;
    let left = s.dot(start.cross(-Vec3A::Z)) > 0.;

    let return_original = if end.dot(start.cross(-Vec3A::Z)) > 0. {
        right && left
    } else {
        right || left
    };

    if return_original {
        vec
    } else if start.dot(s) < end.dot(s) {
        end
    } else {
        start
    }
}

pub fn get_shot_vector_2d(direction: Vec3A, ball_location: Vec3A, target_left: Vec3A, target_right: Vec3A) -> Vec3A {
    clamp_2d(direction, (target_left - ball_location).normalize_or_zero(), (target_right - ball_location).normalize_or_zero())
}

pub fn flatten(vec: Vec3A) -> Vec3A {
    Vec3A::new(vec.x, vec.y, 0.)
}

// fn clockwise90_2d(vec: Vec3A) -> Vec3A {
//     Vec3A::new(vec.y, -vec.x, 0.)
// }

// fn rotate_2d(vec: Vec3A, angle: f32) -> Vec3A {
//     Vec3A::new(angle.cos() * vec.x - angle.sin() * vec.y, angle.sin() * vec.x + angle.cos() * vec.y, vec.z)
// }

#[derive(Clone, Copy, Debug, Default)]
pub struct PostCorrection {
    pub target_left: Vec3A,
    pub target_right: Vec3A,
    pub fits: bool,
}

pub fn correct_for_posts(ball_location: Vec3A, ball_radius: f32, target_left: Vec3A, target_right: Vec3A) -> PostCorrection {
    let goal_line_perp = (target_right - target_left).cross(Vec3A::Z);

    let left_adjusted = target_left + (target_left - ball_location).normalize_or_zero().cross(-Vec3A::Z) * ball_radius;
    let right_adjusted = target_right + (target_right - ball_location).normalize_or_zero().cross(Vec3A::Z) * ball_radius;

    let left_corrected = if (left_adjusted - target_left).dot(goal_line_perp) > 0. {
        target_left
    } else {
        left_adjusted
    };

    let right_corrected = if (right_adjusted - target_right).dot(goal_line_perp) > 0. {
        target_right
    } else {
        right_adjusted
    };

    let left_to_right = right_corrected - left_corrected;
    let new_goal_line = left_to_right.normalize_or_zero();
    let new_goal_width = left_to_right.length();

    let new_goal_perp = new_goal_line.cross(Vec3A::Z);
    let goal_center = left_corrected + new_goal_line * new_goal_width * 0.5;
    let ball_to_goal = (goal_center - ball_location).normalize_or_zero();

    PostCorrection {
        target_left: left_corrected,
        target_right: right_corrected,
        fits: new_goal_width * new_goal_perp.dot(ball_to_goal).abs() > ball_radius * 2.,
    }
}
