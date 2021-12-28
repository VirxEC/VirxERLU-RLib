from time import time_ns

import virxrlru as rlru

print("")

print(rlru.__doc__)

print("")

print("Loading soccar...")
rlru.load_soccar()

print("")

print("Benchmarking...")
time = 0.
times = [[], [], [], [], [], []]

for _ in range(5000):
    start = time_ns()
    rlru.tick_dict(
        time=time,
        ball={
            "location": (
                0,
                0,
                100,
            ),
            "velocity": (
                0,
                0,
                0,
            ),
            "angular_velocity": (
                0,
                0,
                0,
            ),
        },
        car={
            "location": (
                -3000,
                1500,
                100,
            ),
            "velocity": (
                0,
                0,
                0,
            ),
            "angular_velocity": (
                0,
                0,
                0,
            ),
            "hitbox": (
                118,
                84,
                36,
            ),
            "hitbox_offset": (
                0,
                0,
                0,
            ),
            "pitch": 0,
            "yaw": 1,
            "roll": 0,
            "boost": 48,
            "demolished": False,
            "airborne": False,
            "jumped": False,
            "doublejumped": False,
            "index": 0,
        },
    )

    times[0].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), 0, options={
        "all": True,
        "use_absolute_max_values": True,
    })

    times[4].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), 0, options={
        "use_absolute_max_values": True,
    })

    times[5].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), 0, options={
        "all": True,
    })

    times[1].append(time_ns() - start)

    start = time_ns()

    rlru.get_shot_with_target((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), 0, {})

    times[2].append(time_ns() - start)

    start = time_ns()

    rlru.get_data_for_shot_with_target((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), 5.9, 0, {})

    times[3].append(time_ns() - start)

    time += 1/120

print("")

print("tick():")
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
