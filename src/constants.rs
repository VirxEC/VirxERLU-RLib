pub const MAX_SPEED: f32 = 2300.;
pub const MAX_SPEED_NO_BOOST: f32 = 1410.;
pub const MIN_SPEED: f32 = -MAX_SPEED_NO_BOOST;
pub const TPS: f32 = 120.;
pub const SIMULATION_DT: f32 = 1. / TPS;
pub const BOOST_CONSUMPTION: f32 = 33.3 + 1. / 33.;
pub const BRAKE_ACC: f32 = 3500.;
pub const COAST_ACC: f32 = 525.;
pub const MIN_BOOST_TIME: f32 = 3. / 120.;

pub const THROTTLE_ACCEL_DIVISION: f32 = 1400.;
pub const START_THROTTLE_ACCEL_M: f32 = -36. / 35.;
pub const START_THROTTLE_ACCEL_B: f32 = 1600.;
pub const END_THROTTLE_ACCEL_M: f32 = -16.;
pub const END_THROTTLE_ACCEL_B: f32 = 160.;

pub const BOOST_ACCEL: f32 = 991. + 2. / 3.;
pub const BOOST_ACCEL_DT: f32 = BOOST_ACCEL * SIMULATION_DT;

pub const MIN_BOOST_CONSUMPTION: f32 = BOOST_CONSUMPTION * MIN_BOOST_TIME;
pub const BOOST_CONSUMPTION_DT: f32 = BOOST_CONSUMPTION * SIMULATION_DT;
pub const BRAKE_ACC_DT: f32 = BRAKE_ACC * SIMULATION_DT;

pub const BRAKE_COAST_TRANSITION: f32 = -(0.45 * BRAKE_ACC + 0.55 * COAST_ACC);
pub const COASTING_THROTTLE_TRANSITION: f32 = -0.5 * COAST_ACC;

pub const REACTION_TIME: f32 = 0.04;
pub const STEER_REACTION_TIME: f32 = 0.25;

pub const NO_GAME_ERR: &str = "GAME is unset. Call a function like load_soccar first.";
pub const NO_CAR_ERR: &str = "No car at the provided index.";
pub const NO_SLICES_ERR: &str = "Ball prediction struct has not been initialized yet. Try calling a function like tick() first.";
pub const NO_TARGET_ERR: &str = "Target no longer exists.";
pub const NO_SHOT_ERR: &str = "Specified target has no found shot.";
pub const BALL_CHANGED_ERR: &str = "Ball has changed too much from the original prediction.";
pub const NO_SHOT_SELECTED_ERR: &str = "All shots were disabled.";
