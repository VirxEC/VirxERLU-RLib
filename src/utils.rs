use dubins_paths::PosRot;
use glam::Vec3A;
use std::ops::{Add, Mul, Sub};

/// Get a vec of samples
/// Starts at the given point
/// Ends after the given distance
/// Goes in the direction of the given vector
pub fn get_samples_from_line(start: PosRot, direction: Vec3A, distance: f32, step_distance: f32) -> Vec<PosRot> {
    let mut samples = Vec::with_capacity((distance / step_distance).ceil() as usize);
    let mut current_distance = 0.;

    while current_distance < distance {
        let vec = start.pos + direction * current_distance;
        samples.push(PosRot::new(vec, start.rot));
        current_distance += step_distance;
    }

    samples
}

#[inline]
pub const fn get_tuple_from_vec3(vec: Vec3A) -> (f32, f32, f32) {
    let [x, y, z] = vec.to_array();
    (x, y, z)
}

#[inline]
pub fn lerp<T: Copy + Add<Output = T> + Sub<Output = T> + Mul<f32, Output = T>>(a: T, b: T, t: f32) -> T {
    // Linearly interpolate from a to b using t
    // For instance, when t == 0, a is returned, and when t is 1, b is returned
    // Works for both numbers and Vectors
    (b - a) * t + a
}

// #[inline]
// fn invlerp<T: Copy + Sub<Output = T> + Div<Output = f32>>(a: T, b: T, v: T) -> f32 {
//     // Inverse linear interpolation from a to b with value v
//     // For instance, it returns 0 if v is a, and returns 1 if v is b, and returns 0.5 if v is exactly between a and b
//     // Works for both numbers and Vectors
//     (v - a) / (b - a)
// }

enum ClampDirection {
    Middle,
    Left,
    Right,
}

fn clamp_index(s: Vec3A, start: Vec3A, end: Vec3A) -> ClampDirection {
    let right = s.dot(end.cross(-Vec3A::Z)) < 0.;
    let left = s.dot(start.cross(-Vec3A::Z)) > 0.;

    let return_original = if end.dot(start.cross(-Vec3A::Z)) > 0. { right && left } else { right || left };

    if return_original {
        ClampDirection::Middle
    } else if start.dot(s) < end.dot(s) {
        ClampDirection::Right
    } else {
        ClampDirection::Left
    }
}

#[inline]
pub const fn flatten(vec: Vec3A) -> Vec3A {
    let [x, y, _] = vec.to_array();
    Vec3A::new(x, y, 0.)
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

impl PostCorrection {
    pub fn from(ball_location: Vec3A, ball_radius: f32, target_left: Vec3A, target_right: Vec3A) -> Self {
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

        Self {
            target_left: left_corrected,
            target_right: right_corrected,
            fits: new_goal_width * new_goal_perp.dot(ball_to_goal).abs() > ball_radius * 2.,
        }
    }

    pub fn get_shot_vector_target(&self, car_location: Vec3A, ball_location: Vec3A) -> Vec3A {
        let left_vector = (self.target_left - ball_location).normalize_or_zero();
        let left_vector_flat = flatten(left_vector);
        let right_vector = (self.target_right - ball_location).normalize_or_zero();
        let right_vector_flat = flatten(right_vector);
        let car_to_ball_flat = flatten(ball_location - car_location).normalize_or_zero();

        // All of this is so that the returned vector will always point towards the target z
        match clamp_index(car_to_ball_flat, left_vector_flat, right_vector_flat) {
            ClampDirection::Middle => {
                // angle_between between uses acos, will always be between 0 and pi
                let car_to_left_angle = left_vector_flat.angle_between(car_to_ball_flat);
                let left_to_right_angle = left_vector_flat.angle_between(right_vector);

                // convert angles to a value between 0 and 1
                let t = car_to_left_angle / left_to_right_angle;

                (lerp(self.target_left, self.target_right, t) - ball_location).normalize_or_zero()
            }
            ClampDirection::Left => left_vector,
            ClampDirection::Right => right_vector,
        }
    }
}

#[inline]
pub fn minimum_non_negative(x1: f32, x2: f32) -> f32 {
    // get the smallest, non-negative value
    if x1 < 0. {
        x2
    } else if x2 < 0. {
        x1
    } else {
        x1.min(x2)
    }
}

// solve for x
// y = a(x - h)^2 + k
// y - k = a(x - h)^2
// (y - k) / a = (x - h)^2
// sqrt((y - k) / a) = x - h
// sqrt((y - k) / a) + h = x
pub fn vertex_quadratic_solve_for_x(a: f32, h: f32, k: f32, y: f32) -> (f32, f32) {
    if a == 0. {
        return (0., 0.);
    }

    let v_sqrt = ((y - k) / a).sqrt();
    (h + v_sqrt, h - v_sqrt)
}
