MAX_SPEED = 2300
MAX_SPEED_NO_BOOST = 1410
BOOST_ACCEL = 991.667
COAST_ACC = 525
BRAKE_ACC = 3500
MIN_BOOST_TIME = 3 / 120
REACTION_TIME = 0.04
THROTTLE_ACCEL_DIVISION = 1400
START_THROTTLE_ACCEL_M = -36 / 35
START_THROTTLE_ACCEL_B = 1600
END_THROTTLE_ACCEL_M = -16
END_THROTTLE_ACCEL_B = 160
BRAKE_COAST_TRANSITION = -(0.45 * BRAKE_ACC + 0.55 * COAST_ACC)
COASTING_THROTTLE_TRANSITION = -0.5 * COAST_ACC
BOOST_CONSUMPTION = 33.3 + 1. / 33.
MIN_BOOST_CONSUMPTION = BOOST_CONSUMPTION * MIN_BOOST_TIME


def throttle_acc_from_speed(speed):
    # https://samuelpmish.github.io/notes/RocketLeague/ground_control/#throttle
    if speed < 0:
        return BRAKE_ACC
    if speed < 1400:
        return (-36 / 35) * speed + 1600
    elif speed < 1410:
        return -16 * speed + 22560
    else:
        return 0


class Sim:
    def __init__(self):
        self.vel = 0.0
        self.boosting = False
        self.boost_time = 0.0

    def step(self, dt, throttle, boost):
        acc = 0.0
        
        if boost:
            self.boosting = True
        elif self.boost_time > MIN_BOOST_TIME:
            self.boosting = False
            self.boost_time = 0.0

        if self.boosting:
            throttle = 1.0
            acc += BOOST_ACCEL
            self.boost_time += dt
        
        if throttle > 0.0:
            if self.vel < 0.0:
                acc += BRAKE_ACC
            else:
                acc += throttle_acc_from_speed(self.vel) * throttle
        elif throttle < 0.0:
            if self.vel > 0.0:
                acc += -BRAKE_ACC
            else:
                acc += throttle_acc_from_speed(-self.vel) * throttle
        else:
            if self.vel > 0.0:
                acc += -COAST_ACC
            elif self.vel < 0.0:
                acc += COAST_ACC        

        self.vel += acc * dt
        self.vel = max(-MAX_SPEED, min(MAX_SPEED, self.vel))


def throttle_acceleration(forward_velocity):
    x = abs(forward_velocity)

    if x >= MAX_SPEED_NO_BOOST:
        return 0

    if x < THROTTLE_ACCEL_DIVISION:
        return START_THROTTLE_ACCEL_M * x + START_THROTTLE_ACCEL_B

    return END_THROTTLE_ACCEL_M * (x - THROTTLE_ACCEL_DIVISION) + END_THROTTLE_ACCEL_B


def clamp(x, min_val, max_val):
    return max(min(x, max_val), min_val)


if __name__ == "__main__":
    from math import sin
    import matplotlib.pyplot as plt

    # Replace with your own speed controller!
    def speed_controller(v, r, _dt):
        t = r - v
        acceleration = t / REACTION_TIME
        throttle_accel = throttle_acceleration(v)
        throttle_boost_transition = throttle_accel + 0.5 * BOOST_ACCEL

        if acceleration <= BRAKE_COAST_TRANSITION:
            return (-1., False)
        elif BRAKE_COAST_TRANSITION < acceleration and acceleration < COASTING_THROTTLE_TRANSITION:
            return (0., False)
        elif COASTING_THROTTLE_TRANSITION <= acceleration and acceleration <= throttle_boost_transition:
            throttle = 1 if throttle_accel == 0. else clamp(acceleration / throttle_accel, 0.02, 1.)
            return (throttle, False)
        elif throttle_boost_transition < acceleration:
            return (1., t > 0.)
        
        return (0., False)

    dt = 1 / 120
    seconds = 6
    t = [i * dt for i in range(int(seconds * (1 / dt)))]
    a = []
    b = []
    sim = Sim()
    for time in t:
        target = sin(time) * 2300  # Target velocity
        a.append(target)
        b.append(sim.vel)
        throttle, boost = speed_controller(sim.vel, target, dt)
        sim.step(dt, throttle, boost)
    plt.plot(t, a)
    plt.plot(t, b)
    plt.show()