use dubins_paths::{DubinsPath, DubinsResult};
use glam::Vec3A;

#[derive(Clone, Debug)]
pub struct Shot {
    pub time: f32,
    pub subpaths: [DubinsPath; 3],
    pub distances: [f32; 4],
    pub all_samples: Vec<Vec<f32>>,
    pub samples: [Vec<Vec3A>; 3],
}

impl Shot {
    const STEP_DISTANCE: f32 = 10.;

    pub fn from(time: f32, path: DubinsPath, distances: [f32; 4]) -> Self {
        // the distance of each segment
        let segment_distances = [path.segment_length(0), path.segment_length(0) + path.segment_length(1), path.length()];

        // extract the subpath at each distance in segment_distances
        let subpaths = [
            path.extract_subpath(segment_distances[0]),
            path.extract_subpath(segment_distances[1]),
            path.extract_subpath(segment_distances[2]),
        ];

        // the samples for each subpath
        let raw_samples = [
            subpaths[0].sample_many(Shot::STEP_DISTANCE),
            subpaths[1].sample_many(Shot::STEP_DISTANCE),
            subpaths[2].sample_many(Shot::STEP_DISTANCE),
        ];

        let all_samples = raw_samples[0]
            .iter()
            .map(|x| vec![x[0], x[1]])
            .chain(raw_samples[1].iter().map(|x| vec![x[0], x[1]]))
            .chain(raw_samples[2].iter().map(|x| vec![x[0], x[1]]))
            .collect();

        let samples: [Vec<Vec3A>; 3] = [
            raw_samples[0].iter().map(|v| Vec3A::new(v[0], v[1], 0.)).collect(),
            raw_samples[1].iter().map(|v| Vec3A::new(v[0], v[1], 0.)).collect(),
            raw_samples[2].iter().map(|v| Vec3A::new(v[0], v[1], 0.)).collect(),
        ];

        Self {
            time,
            subpaths,
            distances,
            all_samples,
            samples,
        }
    }

    /// Find the minimum distance between a Vec3A and the shot sections
    /// Use a binary search
    /// Sections is sorted in a unimodal fashion
    /// Returns the index of the section
    fn find_min_distance_in_segment_index(&self, segment: usize, target: Vec3A) -> (usize, f32) {
        let mut min_distance = f32::MAX;
        let mut start_index = 0;
        let mut end_index = self.samples[segment].len() - 1;

        while start_index < end_index {
            let mid_index = (start_index + end_index) / 2;
            min_distance = self.samples[segment][mid_index].distance(target);

            if min_distance < self.samples[segment][mid_index + 1].distance(target) {
                end_index = mid_index - 1;
            } else {
                start_index = mid_index + 1;
            }
        }

        (start_index, min_distance)
    }

    // Returns the index of the section and the index in the section
    fn find_min_distance_index(&self, target: Vec3A) -> (usize, usize) {
        let mut min_distance = f32::MAX;
        let mut min_distance_index = 0;
        let mut min_distance_index_in_section = 0;

        for segment in 0..self.samples.len() {
            let (index, distance) = self.find_min_distance_in_segment_index(segment, target);

            if distance < min_distance {
                min_distance = distance;
                min_distance_index = segment;
                min_distance_index_in_section = index;
            }
        }

        (min_distance_index, min_distance_index_in_section)
    }

    /// Use find_min_distance_index to find the index using the target
    /// Multiply the index by the step distance to get the path distance
    /// Extract the subpath from the path distance
    /// Return either the subpath or a DubinsError
    pub fn extract_subpath_from_target(&self, target: Vec3A) -> DubinsResult<DubinsPath> {
        let (segment, index) = self.find_min_distance_index(target);
        let path_distance = index as f32 * Shot::STEP_DISTANCE;

        Ok(self.subpaths[segment].extract_subpath(path_distance))
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
    pub fn from(min_slice: Option<usize>, max_slice: Option<usize>, use_absolute_max_values: Option<bool>, all: Option<bool>, max_slices: usize) -> Self {
        let min_slice = min_slice.unwrap_or(0);
        let max_slice = max_slice.unwrap_or(max_slices);
        let use_absolute_max_values = use_absolute_max_values.unwrap_or(false);
        let all = all.unwrap_or(false);

        Self {
            all,
            use_absolute_max_values,
            min_slice,
            max_slice,
        }
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
