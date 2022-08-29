from time import time_ns
from rlbot.utils.structures.game_data_struct import *
from math import pi
import virx_erlu_rlib as rlru

print()

print(rlru.__doc__)

print()

print("Loading soccar...")
rlru.load_soccar()

packet = GameTickPacket()

# set 6 cars to test the speed of tick()
packet.num_cars = 64
for i in range(packet.num_cars):
    packet.game_cars[i] = PlayerInfo(
        physics=Physics(
            location=Vector3(3500, -3500, 100),
            rotation=Rotator(0, 1.1, 0),
            velocity=Vector3(0, 0, 0),
            angular_velocity=Vector3(0, 0, 0),
        ),
        # physics=Physics(
        #     location=Vector3(-88.84, -1071.1, 17.01),
        #     rotation=Rotator(0, -1.894, 0),
        #     velocity=Vector3(0, 0, 0),
        #     angular_velocity=Vector3(0, 0, 0),
        # ),
        score_info=ScoreInfo(0, 0, 0, 0, 0, 0, 0),
        is_demolished=False,
        has_wheel_contact=False,
        # has_wheel_contact=True,
        is_super_sonic=False,
        is_bot=True,
        team=0,
        name="DownToEarth",
        jumped=False,
        double_jumped=False,
        boost=50,
        hitbox=BoxShape(
            length=118,
            width=84.2,
            height=36.2,
        ),
        hitbox_offset=Vector3(13.9, 0, 20.8),
        spawn_id=1793714700,
    )
packet.game_ball.physics.location.z = 1000.1
packet.game_ball.physics.velocity.z = -0.001
# packet.game_ball.physics.location.x = 0
# packet.game_ball.physics.location.y = 0
# packet.game_ball.physics.location.z = 93.14
packet.game_ball.collision_shape.type = 1
packet.game_ball.collision_shape.sphere.diameter = 182.5
packet.game_info.world_gravity_z = -650
packet.teams[1].team_index = 1
packet.num_teams = 2

times = [[], [], [], [], [], [], [], [], [], [], [], [], []]

print()

print("Testing...")

print()

rlru.tick(packet)

use_abs_all = rlru.TargetOptions(use_absolute_max_values=True, all=True)
use_abs = rlru.TargetOptions(use_absolute_max_values=True)
use_all = rlru.TargetOptions(all=True)

print(use_abs_all)
print(use_abs)
print(use_all)
print()

car_index = 0
target_args = ((800, 5120, 0), (-800, 5120, 0), car_index)
rlru.new_target(*target_args, use_abs)
rlru.new_target(*target_args)
rlru.new_any_target(car_index, use_abs)
rlru.new_any_target(car_index)

print("get_slice(1.2):")
slice = rlru.get_slice(1.2)
print(slice)
print(repr(slice))

print()

print("get_shot_with_target(use_absolute_max_values):")
shot = rlru.get_shot_with_target(0)
print(shot)
print(repr(shot))

print()

print("get_data_for_shot_with_target(use_absolute_max_values):")
data = rlru.get_data_for_shot_with_target(0)
print(data)
print(repr(data))

print()

print("get_shot_with_target():")
shot = rlru.get_shot_with_target(1)
print(shot)
print(repr(shot))

print()

print("get_data_for_shot_with_target():")
data = rlru.get_data_for_shot_with_target(1)
print(data)
print(repr(data))

print()

print("get_shot_with_target(any, use_absolute_max_values):")
shot = rlru.get_shot_with_target(2)
print(shot)
print(repr(shot))

print()

print("get_data_for_shot_with_target(any, use_absolute_max_values):")
data = rlru.get_data_for_shot_with_target(2)
print(data)
print(repr(data))

print()

print("get_shot_with_target(any):")
shot = rlru.get_shot_with_target(3)
print(shot)
print(repr(shot))

print()

print("get_data_for_shot_with_target(any):")
data = rlru.get_data_for_shot_with_target(3)
print(data)
print(repr(data))

print()
rlru.print_targets()

# exit()
print()

print("Benchmarking...")

for _ in range(5000):
    start = time_ns()

    rlru.tick(packet)

    times[0].append(time_ns() - start)

    start = time_ns()

    car_index = 0
    target_args = ((800, 5120, 0), (-800, 5120, 0), car_index)
    rlru.new_target(*target_args, use_abs_all)
    rlru.new_target(*target_args, use_abs)
    rlru.new_target(*target_args, use_all)
    rlru.new_target(*target_args)
    rlru.new_any_target(car_index, use_abs_all)
    rlru.new_any_target(car_index, use_abs)
    rlru.new_any_target(car_index, use_all)
    rlru.new_any_target(car_index)

    times[7].append(time_ns() - start)

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

    rlru.get_shot_with_target(3, temporary=True)

    times[6].append(time_ns() - start)

    start = time_ns()

    rlru.get_data_for_shot_with_target(3)

    times[3].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target(4)

    times[8].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target(5)

    times[9].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target(6)

    times[10].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target(7)

    times[11].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target(7, temporary=True)

    times[12].append(time_ns() - start)

print()

print("tick():")
print(f"Total test time: {round(sum(times[0]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[0]) / len(times[0]) / 1000000, 3)}ms")

print()

print("new_target():")
print(f"Total test time: {round(sum(times[7]) / 1000000000, 6)}s")
print(f"Avg. time of execution: {round(sum(times[7]) / len(times[0]) / 8, 1)}ns")

print()

print("get_shot_with_target(use_absolute_max_values) worst-case:")
print(f"Total test time: {round(sum(times[4]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[4]) / len(times[1]) / 1000000, 3)}ms")

print()

print("get_shot_with_target(use_absolute_max_values):")
print(f"Total test time: {round(sum(times[5]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[5]) / len(times[1]) / 1000000, 3)}ms")

print()

print("get_shot_with_target() worst-case:")
print(f"Total test time: {round(sum(times[1]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[1]) / len(times[1]) / 1000000, 3)}ms")

print()

print("get_shot_with_target():")
print(f"Total test time: {round(sum(times[2]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[2]) / len(times[2]) / 1000000, 3)}ms")

print()

print("get_shot_with_target(temporary):")
print(f"Total test time: {round(sum(times[6]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[6]) / len(times[6]) / 1000000, 3)}ms")

print()

print("get_shot_with_target(any, use_absolute_max_values) worst-case:")
print(f"Total test time: {round(sum(times[8]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[8]) / len(times[8]) / 1000000, 3)}ms")

print()

print("get_shot_with_target(any, use_absolute_max_values):")
print(f"Total test time: {round(sum(times[9]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[9]) / len(times[9]) / 1000000, 3)}ms")

print()

print("get_shot_with_target(any) worst-case:")
print(f"Total test time: {round(sum(times[10]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[10]) / len(times[10]) / 1000000, 3)}ms")

print()

print("get_shot_with_target(any):")
print(f"Total test time: {round(sum(times[11]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[11]) / len(times[11]) / 1000000, 3)}ms")

print()

print("get_shot_with_target(any, temporary):")
print(f"Total test time: {round(sum(times[12]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[12]) / len(times[12]) / 1000000, 3)}ms")

print()

print("get_data_for_shot_with_target():")
print(f"Total test time: {round(sum(times[3]) / 1000000000, 6)}s")
print(f"Avg. time of execution: {round(sum(times[3]) / len(times[3]), 1)}ns")

print()
