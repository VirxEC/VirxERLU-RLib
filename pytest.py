from time import time_ns
from rlbot.utils.structures.game_data_struct import *

import virx_erlu_rlib as rlru

print("")

print(rlru.__doc__)

print("")

print("Loading soccar...")
rlru.load_soccar()

packet = GameTickPacket()

# set 6 cars to test the speed of tick()
packet.num_cars = 6
for i in range(packet.num_cars):
    packet.game_cars[i] = PlayerInfo(
        physics=Physics(
            location=Vector3(-3000., 1500., 100.),
            rotation=Rotator(0., 0.5, 0.),
            velocity=Vector3(0., 0., 0.),
            angular_velocity=Vector3(0., 0., 0.),
        ),
        score_info=ScoreInfo(0, 0, 0, 0, 0, 0, 0),
        is_demolished=False,
        has_wheel_contact=True,
        is_super_sonic=False,
        is_bot=True,
        team=0,
        name="DownToEarth",
        jumped=False,
        double_jumped=False,
        boost=48,
        hitbox=BoxShape(
            length=118.,
            width=84.2,
            height=36.2,
        ),
        hitbox_offset=Vector3(13.9, 0., 20.8),
        spawn_id=1793714700,
    )
packet.game_ball.physics.location.z = 93.
packet.game_ball.collision_shape.type = 1
packet.game_ball.collision_shape.sphere.diameter = 182.5
packet.game_info.world_gravity_z = 650.
packet.teams[1].team_index = 1
packet.num_teams = 2

times = [[], [], [], [], [], []]

print("")

print("Benchmarking...")

for _ in range(5000):
    start = time_ns()

    rlru.tick(packet)

    rlru.new_target((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), 0, {
        "all": True,
        "use_absolute_max_values": True,
    })

    rlru.new_target((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), 0, {
        "use_absolute_max_values": True,
    })

    rlru.new_target((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), 0, {
        "all": True,
    })
    
    rlru.new_target((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), 0, {})

    times[0].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target(0)

    times[4].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target(1)

    times[5].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target(2)

    times[1].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target(3)

    times[2].append(time_ns() - start)

    start = time_ns()

    # rlru.get_data_for_shot_with_target(3)

    times[3].append(time_ns() - start)

print("")

print("Starting:")
print(f"Total test time: {round(sum(times[0]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[0]) / len(times[0]) / 1000000, 3)}ms")

print("")

print("get_shot_with_target(use_absolute_max_values) worst-case:")
print(f"Total test time: {round(sum(times[4]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[4]) / len(times[1]) / 1000000, 3)}ms")

print("")

print("get_shot_with_target(use_absolute_max_values):")
print(f"Total test time: {round(sum(times[5]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[5]) / len(times[1]) / 1000000, 3)}ms")

print("")

print("get_shot_with_target() worst-case:")
print(f"Total test time: {round(sum(times[1]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[1]) / len(times[1]) / 1000000, 3)}ms")

print("")

print("get_shot_with_target():")
print(f"Total test time: {round(sum(times[2]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[2]) / len(times[2]) / 1000000, 3)}ms")

print("")

print("get_data_for_shot_with_target():")
print(f"Total test time: {round(sum(times[3]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[3]) / len(times[3]) / 1000000, 3)}ms")

print("")
