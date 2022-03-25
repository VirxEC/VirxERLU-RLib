use crate::{
    ground::TargetInfo,
    pytypes::TargetOptions,
    utils::{get_samples_from_line, get_samples_from_path, get_vec3_from_array},
};
use dubins_paths::DubinsPath;
use glam::Vec3A;
use rl_ball_sym::simulation::ball::Ball;

#[derive(Clone, Debug)]
pub struct Shot {
    pub time: f32,
    pub ball_location: Vec3A,
    pub direction: Vec3A,
    pub distances: [f32; 4],
    pub all_samples: Vec<(f32, f32)>,
    pub samples: [Vec<Vec3A>; 4],
    pub path: DubinsPath,
    pub path_endpoint: Vec3A,
    pub shot_type: usize,
}

impl Shot {
    const STEP_DISTANCE: f32 = 10.;
    pub const ALL_STEP: usize = 3;

    pub fn from(ball: &Ball, target: &TargetInfo, direction: Vec3A) -> Self {
        let path_endpoint = get_vec3_from_array(target.path.endpoint());

        let (all_samples, samples) = {
            // the distance of each segment
            let segment_distances = [
                target.path.segment_length(0),
                target.path.segment_length(0) + target.path.segment_length(1),
                target.path.length(),
            ];

            // the samples for each subpath
            let raw_samples = [
                get_samples_from_path(&target.path, 0., segment_distances[0], Self::STEP_DISTANCE),
                get_samples_from_path(&target.path, segment_distances[0], segment_distances[1], Self::STEP_DISTANCE),
                get_samples_from_path(&target.path, segment_distances[1], segment_distances[2], Self::STEP_DISTANCE),
                get_samples_from_line(path_endpoint, direction, target.distances[3], Self::STEP_DISTANCE),
            ];

            (
                raw_samples[0]
                    .iter()
                    .map(|x| (x[0], x[1]))
                    .chain(raw_samples[1].iter().map(|x| (x[0], x[1])))
                    .chain(raw_samples[2].iter().map(|x| (x[0], x[1])))
                    .chain(raw_samples[3].iter().map(|x| (x[0], x[1])))
                    .step_by(Self::ALL_STEP)
                    .collect(),
                [
                    raw_samples[0].iter().map(|v| Vec3A::new(v[0], v[1], 0.)).collect(),
                    raw_samples[1].iter().map(|v| Vec3A::new(v[0], v[1], 0.)).collect(),
                    raw_samples[2].iter().map(|v| Vec3A::new(v[0], v[1], 0.)).collect(),
                    raw_samples[3].iter().map(|v| Vec3A::new(v[0], v[1], 0.)).collect(),
                ],
            )
        };

        Self {
            time: ball.time,
            ball_location: ball.location,
            direction,
            distances: target.distances,
            all_samples,
            samples,
            path: target.path,
            path_endpoint,
            shot_type: target.shot_type,
        }
    }

    fn find_min_distance_in_segment_index(&self, segment: usize, target: Vec3A) -> (usize, f32) {
        let mut min_distance = f32::MAX;
        let mut start_index = 0;

        let length = self.samples[segment].len();
        if length == 0 {
            return (start_index, min_distance);
        }

        let mut end_index = length - 1;

        while start_index < end_index {
            let mid_index = (start_index + end_index) / 2;
            min_distance = self.samples[segment][mid_index].distance(target);

            if min_distance < self.samples[segment][mid_index + 1].distance(target) {
                end_index = mid_index;
            } else {
                start_index = mid_index + 1;
            }
        }

        (start_index, min_distance)
    }

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

    pub fn get_distance_along_shot_and_index(&self, target: Vec3A) -> (f32, usize) {
        let (segment, index) = self.find_min_distance_index(target);

        let pre_distance = match segment {
            0 => 0.,
            1 => self.distances[0],
            2 => self.distances[0] + self.distances[1],
            3 => self.distances[0] + self.distances[1] + self.distances[2],
            _ => unreachable!(),
        };

        let pre_index = match segment {
            0 => 0,
            1 => self.samples[0].len(),
            2 => self.samples[0].len() + self.samples[1].len(),
            3 => self.samples[0].len() + self.samples[1].len() + self.samples[2].len(),
            _ => unreachable!(),
        };

        (pre_distance + index as f32 * Self::STEP_DISTANCE, pre_index + index)
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
    pub fn from(options: Option<TargetOptions>, max_slices: usize) -> Self {
        match options {
            Some(options) => {
                let min_slice = options.min_slice.unwrap_or(0);
                let max_slice = options.max_slice.unwrap_or(max_slices);
                let use_absolute_max_values = options.use_absolute_max_values.unwrap_or(false);
                let all = options.all.unwrap_or(false);

                Self {
                    all,
                    use_absolute_max_values,
                    min_slice,
                    max_slice,
                }
            }
            None => Self {
                min_slice: 0,
                max_slice: max_slices,
                use_absolute_max_values: false,
                all: false,
            },
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
