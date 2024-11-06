import numpy as np
from simulation_2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 10
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, 0]
        self.reference2 = None

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        x_t_1 = x_t + v_t * (np.cos(psi_t)) * dt
        y_t_1 = y_t + v_t * (np.sin(psi_t)) * dt
        v_t_1 = v_t + pedal * dt - v_t /25
        psi_t_1 = psi_t + v_t * (np.tan(steering)/2.5) * dt

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        for i in range(self.horizon):
            v_start = state[3]
            state = self.plant_model(state, self.dt, u[i*2], u[i*2+1])

            # Position costs
            cost += abs(ref[0]-state[0])
            cost += abs(ref[1] - state[1])

            # Angle cost
            cost += abs(ref[2] - state[2])**2

            # Acceleration cost
            cost += abs(state[3] - v_start)**2

            # Steering angle cost
            # cost += abs(u[i*2+1])**2

        return cost

sim_run(options, ModelPredictiveControl)
