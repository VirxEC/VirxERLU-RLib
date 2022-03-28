use crate::constants::*;

// pub fn naive_double_jump_simulation(time_allowed: f32) {
//     let gravity = -650.;
//     let jump_impulse = 292.;
//     let time_increment = 1. / 120.;
//     let sticky_force = -325.;
//     let sticky_timer = time_increment * 3.;
//     let hold_bonus_increment = (292. * 5.) * time_increment;
//     let max_hold_time = 0.2;
//     let mut simulated_z_velocity = 0.;
//     let mut simulated_height = 17.;
//     let mut simulation_time = 0.;
//     let mut double_jumped = false;

//     while simulation_time < time_allowed {
//         if simulation_time <= f32::EPSILON {
//             simulated_z_velocity += jump_impulse
//         } else if simulation_time > max_hold_time + time_increment && !double_jumped {
//             simulated_z_velocity += jump_impulse;
//             double_jumped = true;
//         }

//         if simulation_time < max_hold_time {
//             simulated_z_velocity += hold_bonus_increment;
//         }

//         if simulation_time < sticky_timer {
//             simulated_z_velocity += sticky_force * time_increment;
//         }

//         simulated_z_velocity += gravity * time_increment;
//         simulated_height += simulated_z_velocity * time_increment;
//         simulation_time += time_increment;
//     }
// }

pub fn max_jump_height(gravity: f32) -> f32 {
    let g = gravity * SIMULATION_DT;

    let mut t = 0.;
    let mut v_z = 0.;
    let mut l_z = 17.;

    while v_z > 0. {
        if t <= f32::EPSILON {
            v_z += JUMP_IMPULSE;
        }

        if t < MAX_HOLD_TME {
            v_z += HOLD_BONUS;
        }

        if t < STICKY_TIMER {
            v_z += STICKY_FORCE * SIMULATION_DT;
        }

        t += SIMULATION_DT;
        v_z += g;
        l_z += v_z * SIMULATION_DT;
    }

    l_z
}

pub fn _jump_time_to_height(gravity: f32, height_goal: f32) -> f32 {
    let g = gravity * SIMULATION_DT;

    let mut t = 0.;
    let mut v_z = 0.;
    let mut l_z = 17.;

    while l_z < height_goal {
        if t <= f32::EPSILON {
            v_z += JUMP_IMPULSE;
        }

        if t < MAX_HOLD_TME {
            v_z += HOLD_BONUS;
        }

        if t < STICKY_TIMER {
            v_z += STICKY_FORCE * SIMULATION_DT;
        }

        t += SIMULATION_DT;
        v_z += g;
        l_z += v_z * SIMULATION_DT;
    }

    t
}

pub fn _double_jump_time_to_height(gravity: f32, height_goal: f32) -> f32 {
    let g = gravity * SIMULATION_DT;

    let mut t = 0.;
    let mut v_z = 0.;
    let mut l_z = 17.;
    let mut double_jumped = false;

    while l_z < height_goal {
        if t <= f32::EPSILON {
            v_z += JUMP_IMPULSE;
        } else if t > MAX_HOLD_TME + SIMULATION_DT && !double_jumped {
            v_z += JUMP_IMPULSE;
            double_jumped = true;
        }

        if t < MAX_HOLD_TME {
            v_z += HOLD_BONUS;
        }

        if t < STICKY_TIMER {
            v_z += STICKY_FORCE * SIMULATION_DT;
        }

        v_z += g;
        l_z += v_z * SIMULATION_DT;
        t += SIMULATION_DT;
    }

    t
}
