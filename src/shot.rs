use cpython::{PyDict, PyResult, Python};
use dubins_paths::DubinsPath;
use glam::Vec3A;

use crate::utils::{get_bool_from_dict, get_usize_from_dict};

#[derive(Clone, Debug)]
pub struct Shot {
    pub time: f32,
    pub path: DubinsPath,
    pub distances: [f32; 4],
    pub sections: Vec<Vec3A>,
}

impl Shot {
    pub const STEP_DISTANCE: f32 = 10.;

    pub fn from(time: f32, path: DubinsPath, distances: [f32; 4]) -> Self {
        let pre_sections = path.sample_many(Self::STEP_DISTANCE).unwrap();
        let mut sections = Vec::with_capacity(pre_sections.len());

        for section in pre_sections {
            sections.push(Vec3A::new(section[0], section[1], section[2]));
        }

        Self {
            time,
            path,
            distances,
            sections,
        }
    }

    pub fn ternary_sections(&self, location: Vec3A) -> usize {
        self.unimodal_min(location, 0, self.sections.len())
    }

    fn unimodal_min(&self, location: Vec3A, low: usize, high: usize) -> usize {
        if low == high - 1 {
            return low;
        }

        let mid = (high + low) / 2;

        if self.sections[mid].distance(location) > self.sections[mid + 1].distance(location) {
            self.unimodal_min(location, low, mid)
        } else {
            self.unimodal_min(location, mid + 1, high)
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Options {
    pub all: bool,
    pub use_absolute_max_values: bool,
    pub min_slice: usize,
    pub max_slice: usize,
}

impl Options {
    pub fn from(py: Python, py_options: PyDict, max_slices: usize) -> PyResult<Self> {
        let all = get_bool_from_dict(py, &py_options, "all", "options").unwrap_or(false);

        let use_absolute_max_values = get_bool_from_dict(py, &py_options, "use_absolute_max_values", "options").unwrap_or(false);

        let min_slice = match get_usize_from_dict(py, &py_options, "min_slice", "options") {
            Ok(u) => u.max(0),
            Err(_) => 0,
        };

        let max_slice = match get_usize_from_dict(py, &py_options, "max_slice", "options") {
            Ok(u) => u.min(max_slices),
            Err(_) => max_slices,
        };

        Ok(Self {
            all,
            use_absolute_max_values,
            min_slice,
            max_slice,
        })
    }
}

#[derive(Clone, Debug, Default)]
pub struct Target {
    pub car_index: usize,
    pub target_left: Vec3A,
    pub target_right: Vec3A,
    pub options: Options,
    pub shot: Option<Shot>,
    confirmed: bool,
}

impl Target {
    pub fn new(target_left: Vec3A, target_right: Vec3A, car_index: usize, options: Options) -> Self {
        Self {
            car_index,
            target_left,
            target_right,
            options,
            shot: None,
            confirmed: false,
        }
    }

    pub fn confirm(&mut self) {
        self.confirmed = true;
    }

    pub fn is_confirmed(&self) -> bool {
        self.confirmed
    }
}
