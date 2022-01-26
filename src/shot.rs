use dubins_paths::{DubinsError, DubinsPath};
use glam::Vec3A;
use pyo3::{types::PyDict, PyResult};

use crate::utils::{get_bool_from_dict, get_usize_from_dict};

#[derive(Clone, Debug)]
pub struct Shot {
    pub time: f32,
    pub path: DubinsPath,
    pub distances: [f32; 4],
    pub sections: Vec<Vec3A>,
}

impl Shot {
    const STEP_DISTANCE: f32 = 10.;

    pub fn from(time: f32, path: DubinsPath, distances: [f32; 4]) -> Self {
        let pre_sections = path.sample_many(Self::STEP_DISTANCE);
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

    /// Find the minimum distance between a Vec3A and the shot sections
    /// Use a ternary search
    /// Sections is sorted in a unimodal fashion
    /// Returns the index of the section
    fn find_min_distance_index(&self, target: Vec3A) -> usize {
        let mut start_index = 0;
        let mut end_index = self.sections.len() - 1;

        while start_index < end_index {
            let mid_index = (start_index + end_index) / 2;

            if self.sections[mid_index].distance(target) < self.sections[mid_index + 1].distance(target) {
                end_index = mid_index - 1;
            } else {
                start_index = mid_index + 1;
            }
        }

        start_index
    }

    /// Use find_min_distance_index to find the index using the target
    /// Multiply the index by the step distance to get the path distance
    /// Extract the subpath from the path distance
    /// Return either the subpath or a DubinsError
    pub fn extract_subpath_from_target(&self, target: Vec3A) -> Result<DubinsPath, DubinsError> {
        let index = self.find_min_distance_index(target);
        let path_distance = index as f32 * Shot::STEP_DISTANCE;

        Ok(self.path.extract_subpath(path_distance))
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
    pub fn from(py_options: &PyDict, max_slices: usize) -> PyResult<Self> {
        let all = get_bool_from_dict(py_options, "all", "options").unwrap_or(false);

        let use_absolute_max_values = get_bool_from_dict(py_options, "use_absolute_max_values", "options").unwrap_or(false);

        let min_slice = match get_usize_from_dict(py_options, "min_slice", "options") {
            Ok(u) => u.max(0),
            Err(_) => 0,
        };

        let max_slice = match get_usize_from_dict(py_options, "max_slice", "options") {
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
    pub const fn new(target_left: Vec3A, target_right: Vec3A, car_index: usize, options: Options) -> Self {
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

    pub const fn is_confirmed(&self) -> bool {
        self.confirmed
    }
}
