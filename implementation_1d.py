import numpy as np
from simulation_1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s
        x_t_1 = x_t + v_t * dt
        v_t_1 = v_t + (pedal) * dt - v_t/25.0
        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        # print(state)
        ref = args[1]
        cost = 0.0
        for i in range(self.horizon):
            v_start = state[3]
            state = self.plant_model(state, self.dt, u[i*2], u[i*2+1])
            cost += abs(ref[0] - state[0])

            speed_kph = state[3] * 3.6
            if ((speed_kph) > 10.0):
                cost += speed_kph * 100

        return cost

sim_run(options, ModelPredictiveControl)
