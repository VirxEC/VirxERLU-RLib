use pyo3::exceptions;

pub const MAX_SPEED: f32 = 2300.;
pub const MAX_SPEED_NO_BOOST: f32 = 1410.;
pub const MIN_SPEED: f32 = -MAX_SPEED_NO_BOOST;
pub const TPS: f32 = 120.;
pub const SIMULATION_DT: f32 = 1. / TPS;
pub const BOOST_CONSUMPTION: f32 = 100. * (1. / 3.);
pub const BRAKE_ACC: f32 = 3500.;
pub const COAST_ACC: f32 = 525.;
pub const MIN_BOOST_TIME: f32 = 3. / 120.;

pub const THROTTLE_ACCEL_DIVISION: f32 = 1400.;
pub const START_THROTTLE_ACCEL_M: f32 = -36. / 35.;
pub const START_THROTTLE_ACCEL_B: f32 = 1600.;
pub const END_THROTTLE_ACCEL_M: f32 = -16.;
pub const END_THROTTLE_ACCEL_B: f32 = 160.;

pub const AERIAL_THROTTLE_ACCEL: f32 = 100. * (2. / 3.);
pub const BOOST_ACCEL: f32 = 991. + 2. / 3.;
pub const AERIAL_START_BOOST_ANGLE: f32 = 0.5;

pub const MIN_BOOST_CONSUMPTION: f32 = BOOST_CONSUMPTION * MIN_BOOST_TIME;
pub const BOOST_CONSUMPTION_DT: f32 = BOOST_CONSUMPTION * SIMULATION_DT;
pub const BRAKE_ACC_DT: f32 = BRAKE_ACC * SIMULATION_DT;

pub const BRAKE_COAST_TRANSITION: f32 = -(0.45 * BRAKE_ACC + 0.55 * COAST_ACC);
pub const COASTING_THROTTLE_TRANSITION: f32 = -0.5 * COAST_ACC;

pub const REACTION_TIME: f32 = 0.04;
pub const STEER_REACTION_TIME: f32 = 0.25;

pub const JUMP_MAX_DURATION: f32 = 0.2;
pub const JUMP_SPEED: f32 = 291. + (2. / 3.);
pub const JUMP_ACC: f32 = 1458. + (1. / 3.);
pub const DOUBLE_JUMP_DURATION: f32 = JUMP_MAX_DURATION + SIMULATION_DT * 2.;

pub type NoGamePyErr = exceptions::PyNameError;
pub const NO_GAME_ERR: &str = "GAME is unset. Call a function like load_soccar first.";
pub type NoCarPyErr = exceptions::PyIndexError;
pub const NO_CAR_ERR: &str = "No car at the provided index.";
pub type NoSlicesPyErr = exceptions::PyValueError;
pub const NO_SLICES_ERR: &str = "Ball prediction struct has not been initialized yet. Try calling a function like tick() first.";
pub type NoTargetPyErr = exceptions::PyIndexError;
pub const NO_TARGET_ERR: &str = "Target no longer exists.";
pub type NoShotPyErr = exceptions::PyLookupError;
pub const NO_SHOT_ERR: &str = "Specified target has no found shot.";
pub type BallChangedPyErr = exceptions::PyAssertionError;
pub const BALL_CHANGED_ERR: &str = "Ball has changed too much from the original prediction.";
pub type NoShotSelectedPyErr = exceptions::PyAssertionError;
pub const NO_SHOT_SELECTED_ERR: &str = "All shots were disabled.";
pub type NoTimeRemainingPyErr = exceptions::PyValueError;
pub const NO_TIME_REMAINING_ERR: &str = "Time expired for the shot.";
pub type BadAccelerationPyErr = exceptions::PyAssertionError;
pub const BAD_ACCELERATION_ERR: &str = "Acceleration is slower than expected.";
pub type StrayedFromPathPyErr = exceptions::PyAssertionError;
pub const STRAYED_FROM_PATH_ERR: &str = "Car has strayed from the path.";

pub const STICKY_FORCE: f32 = -325.;
pub const STICKY_TIMER: f32 = SIMULATION_DT * 3.;
pub const HOLD_BONUS: f32 = 292. * 5.;
pub const MAX_HOLD_TIME: f32 = 0.2;
