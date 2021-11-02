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
times = [[], [], []]

for _ in range(5000):
    start = time_ns()
    rlru.tick(
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
        }
    )

    times[0].append(time_ns() - start)

    start = time_ns()

    rlru.calculate_intercept((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), all=True)

    times[1].append(time_ns() - start)

    start = time_ns()

    rlru.calculate_intercept((
        800, 5120, 0,
    ), (
        -800, 5120, 0,
    ), all=False)

    times[2].append(time_ns() - start)

    time += 1/120

print("")

print("tick():")
print(f"Total test time: {round(sum(times[0]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[0]) / len(times[0]) / 1000000, 3)}ms")

print("")

print("calculate_intercept() worst-case:")
print(f"Total test time: {round(sum(times[1]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[1]) / len(times[1]) / 1000000, 3)}ms")

print("")

print("calculate_intercept():")
print(f"Total test time: {round(sum(times[2]) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times[2]) / len(times[2]) / 1000000, 3)}ms")

print("")
