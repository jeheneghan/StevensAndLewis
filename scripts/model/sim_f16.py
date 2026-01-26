from tools.trim_f16 import cost_trim_f16_straight_level as cost_trim_f16
import numpy as np
from pfd.primary_flight_display import AircraftState, PrimaryFlightDisplay

def RK4(func, y0, h, controls, params):

    k1, outputs = func(y0, controls, params)
    k2, _ = func(y0+ h / 2 * k1, controls, params)
    k3, _ = func(y0 + h / 2 * k2, controls, params)
    k4, _ = func(y0+ h * k3, controls, params)
    y = y0 + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    outputs = func(y, controls, params)

    return y, outputs

def simulate(func, X0, t, controls_list, params):

    n = len(t)
    m = len(X0)

    y = np.zeros((m, n))
    y[:, 0:1] = X0
    outputs_list = []
    _, initial_outputs = func(X0, controls_list[0], params)
    outputs_list.append(initial_outputs)

    for i in range(1, n):
        y_next, outputs = RK4(func=func, y0=y[:, i-1], h=t[i]-t[i-1], controls=controls_list[i], params=params)
        y[:, i] = y_next
        outputs_list.append(outputs)

    return y, outputs_list

import time
import threading
from queue import Queue

def simulate_realtime_decoupled(func, X0, t, controls_list, params):
    """
    Runs physics simulation at 200 Hz and graphics at 60 FPS independently.
    
    Physics: Updates at 200 cycles/second (5ms per step)
    Graphics: Renders at 60 FPS (16.67ms per frame)
    """
    
    n = len(t)
    m = len(X0)
    y = np.zeros((m, n))
    y[:, 0:1] = X0
    outputs_list = []
    
    FPS = 60
    SCREEN_WIDTH = 1000
    SCREEN_HEIGHT = 800
    PHYSICS_HZ = 200
    
    PFD = PrimaryFlightDisplay((SCREEN_WIDTH, SCREEN_HEIGHT), masked=True, max_fps=FPS)
    
    # Shared state between threads
    state_queue = Queue(maxsize=1)
    running = True
    current_state = None
    
    physics_dt = 1.0 / PHYSICS_HZ  # 0.005 seconds
    graphics_dt = 1.0 / FPS         # ~0.0167 seconds
    
    # Physics thread
    def physics_loop():
        nonlocal y, outputs_list, current_state
        
        t0 = time.time()
        i = 1
        last_physics_time = 0
        
        while running and i < n:
            physics_start = time.time()
            elapsed = physics_start - t0
            
            # Run physics step
            y_next, outputs = RK4(
                func=func, 
                y0=y[:, i-1], 
                h=t[i] - t[i-1],
                controls=controls_list[i], 
                params=params
            )
            y[:, i] = y_next
            outputs_list.append(outputs)
            
            # Create aircraft state
            current_state = AircraftState(
                pitch=y[4, i] * 57.296,
                roll=y[3, i] * 57.296,
                airspeed=y[0, i] * 0.592484,
                airspeed_cmd=200,
                vspeed=(y[0, i] - y[0, i-1]) / (t[i] - t[i-1]) * 60,
                altitude=y[11, i],
                altitude_cmd=10000,
                heading=y[6, i] * 57.296,
                heading_cmd=0,
                course=(y[6, i] + y[2, i]) * 57.296,
            )
            
            # Push to queue (overwrites old state if not consumed)
            try:
                state_queue.put_nowait(current_state)
            except:
                pass  # Queue full, skip update
            
            # Sleep to maintain 200 Hz
            physics_elapsed = time.time() - physics_start
            sleep_time = physics_dt - physics_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            i += 1
            last_physics_time = elapsed
    
    # Graphics thread (main thread continues for rendering)
    physics_thread = threading.Thread(target=physics_loop, daemon=True)
    physics_thread.start()
    
    t0 = time.time()
    
    # Graphics loop (main thread)
    try:
        while True:
            graphics_start = time.time()
            elapsed = graphics_start - t0
            
            # Get latest state from queue
            if not state_queue.empty():
                try:
                    current_state = state_queue.get_nowait()
                except:
                    pass
            
            # Update and render graphics with latest state
            if current_state:
                PFD.update(current_state, elapsed)
                PFD.draw()
                PFD.render()
            
            # Sleep to maintain 60 FPS
            graphics_elapsed = time.time() - graphics_start
            sleep_time = graphics_dt - graphics_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Check if physics thread is done
            if not physics_thread.is_alive():
                break
    
    finally:
        running = False
        physics_thread.join(timeout=1)
    
    return y, outputs_list