from time import perf_counter, sleep
from multiprocessing import Process, Manager
from numpy import array, zeros, sin, pi


# Set the control loop timings
frequency = 500
sampling_time = 1/frequency
sim_ratio = 5

pendulum = Manager().Namespace()
# SET INITIAL STATE
pendulum.state = zeros(2)
pendulum.state = array([pi/2, 0])
pendulum.control = 0


def simulator(system):
    try:
        last_execution = 0
        initial_time = perf_counter()
        while True:
            # ///////////////////////////////////////////
            time = perf_counter() - initial_time  # get actual time in secs
            dt = time - last_execution
            if dt >= sampling_time/sim_ratio:
                last_execution = time
                control = system.control

                # DO SIMULATION
                # IMPLEMENT YOUR SIMULATOR HERE
                system.state = system.state

    except KeyboardInterrupt:
        print('\nSimulator is terminated')


simulator_proc = Process(target=simulator, args=(pendulum,))
simulator_proc.start()


try:
    last_execution = 0
    control = 0
    # find the global time before intering control loop
    initial_time = perf_counter()
    while True:
        time = perf_counter() - initial_time  # get actual time in secs

        theta, dtheta = pendulum.state
        # ///////////////////////////////////////////
        # Update the control only on specific timings
        # ///////////////////////////////////////////
        if (time - last_execution) >= sampling_time:
            last_execution = time
            control = 0

        pendulum.control = 0

        print(f'State: {pendulum.state}', end='    \r', flush=True)


except KeyboardInterrupt:

    print('Disabled by interrupt')
except Exception as e:
    print(f'\n!!!! EXCEPTION !!!!\n {e} \n!!!! EXCEPTION !!!!\n')


finally:
    sleep(0.5)
    simulator_proc.join()
