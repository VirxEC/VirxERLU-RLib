use rand::{rngs::ThreadRng, Rng};

use virx_erlu_rlib::{pytypes::*, *};

fn get_random_packet(rng: &mut ThreadRng) -> GamePacket {
    GamePacket {
        game_ball: GameBall {
            physics: GamePhysics {
                location: GameVec {
                    x: rng.gen_range(-3000f32..3000.),
                    y: rng.gen_range(-4000f32..4000.),
                    z: rng.gen_range(20f32..1900.),
                },
                velocity: GameVec {
                    x: rng.gen_range(-1000f32..1000.),
                    y: rng.gen_range(-1000f32..1000.),
                    z: rng.gen_range(-1000f32..1000.),
                },
                angular_velocity: GameVec::default(),
                rotation: GameRot::default(),
            },
            collision_shape: GameCollisionShape {
                shape_type: 1,
                sphere: GameSphere { diameter: 182.5 },
                ..Default::default()
            },
        },
        game_info: GameInfo {
            seconds_elapsed: 0.,
            world_gravity_z: -650.,
        },
        num_cars: 64,
        game_cars: (0..64)
            .map(|_| GameCar {
                physics: GamePhysics {
                    location: GameVec { x: 3500., y: -3500., z: 100. },
                    velocity: GameVec {
                        x: rng.gen_range(-1000f32..1000.),
                        y: rng.gen_range(-1000f32..1000.),
                        z: rng.gen_range(-1000f32..1000.),
                    },
                    angular_velocity: GameVec::default(),
                    rotation: GameRot { pitch: 0., yaw: 1.1, roll: 0. },
                },
                boost: 50,
                hitbox: Hitbox {
                    length: 118.,
                    width: 84.2,
                    height: 36.2,
                },
                hitbox_offset: GameVec { x: 13.9, y: 0., z: 20.8 },
                ..Default::default()
            })
            .collect(),
    }
}

// Run with flamegraph via `cargo flamegraph --profile release-with-debug --example main --no-default-features`
fn main() {
    load_standard();

    let mut thread_rng = rand::thread_rng();

    let left_target = [800., 5120., 0.];
    let right_target = [-800., 5120., 0.];

    for _ in 0..10000 {
        let packet = get_random_packet(&mut thread_rng);
        tick(packet, None).unwrap();

        let car_index = 0;

        new_target(
            left_target,
            right_target,
            car_index,
            Some(TargetOptions {
                use_absolute_max_values: Some(true),
                all: Some(true),
                ..Default::default()
            }),
        )
        .unwrap();

        new_target(
            left_target,
            right_target,
            car_index,
            Some(TargetOptions {
                use_absolute_max_values: Some(true),
                ..Default::default()
            }),
        )
        .unwrap();

        new_target(
            left_target,
            right_target,
            car_index,
            Some(TargetOptions {
                all: Some(true),
                ..Default::default()
            }),
        )
        .unwrap();

        new_target(left_target, right_target, car_index, None).unwrap();

        new_any_target(
            car_index,
            Some(TargetOptions {
                use_absolute_max_values: Some(true),
                all: Some(true),
                ..Default::default()
            }),
        )
        .unwrap();

        new_any_target(
            car_index,
            Some(TargetOptions {
                use_absolute_max_values: Some(true),
                ..Default::default()
            }),
        )
        .unwrap();

        new_any_target(
            car_index,
            Some(TargetOptions {
                all: Some(true),
                ..Default::default()
            }),
        )
        .unwrap();

        new_any_target(car_index, None).unwrap();

        get_shot_with_target(0, None, None, None, None, None, None).unwrap();
        get_shot_with_target(1, None, None, None, None, None, None).unwrap();
        get_shot_with_target(2, None, None, None, None, None, None).unwrap();
        get_shot_with_target(3, None, None, None, None, None, None).unwrap();
        get_shot_with_target(3, Some(true), None, None, None, None, None).unwrap();
        get_shot_with_target(4, None, None, None, None, None, None).unwrap();
        get_shot_with_target(5, None, None, None, None, None, None).unwrap();
        get_shot_with_target(6, None, None, None, None, None, None).unwrap();
        get_shot_with_target(7, None, None, None, None, None, None).unwrap();
        get_shot_with_target(7, Some(true), None, None, None, None, None).unwrap();
    }
}
