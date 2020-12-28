import sys
sys.path.append(".")
from lib import nmpc
from tests._helpers import get_params_and_coeffs, get_state
import time

def timer_func(func):
    def function_timer(*args, **kwargs):
        start = time.time()
        value = func(*args, **kwargs)
        end = time.time()
        runtime = end - start

        msg = '[ BENCHMARK ] Function: {func} executed in {time} seconds.'

        print(msg.format(func=func.__name__, time=runtime))
        return value

    return function_timer

@timer_func
def NMPC_Solver():
    params, coeffs = get_params_and_coeffs()
    state = get_state()
    coeffs = list(coeffs)
    mpc = nmpc.NMPC(params, coeffs)
    _ = mpc.solve(state)


if __name__ == '__main__':
    NMPC_Solver()