"""
Real-time simulation with independent physics and graphics threads.

Physics runs at 200 Hz (5ms per step)
Graphics renders at 60 FPS (16.67ms per frame)

Communication between threads uses a shared state dictionary.
"""

import time
import threading
import numpy as np
import keyboard
import tkinter as tk
from pfd.primary_flight_display import AircraftState, PrimaryFlightDisplay
from modeling.sim_f16 import RK4
from tools.lin_f16 import trim_f16 as trimmer
import pygame
from flightcontrols.flight_controls import update_controls_from_fcs

# Constants
PHYSICS_HZ = 200
GRAPHICS_FPS = 60
SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 800

PHYSICS_DT = 1.0 / PHYSICS_HZ      # ~5ms
GRAPHICS_DT = 1.0 / GRAPHICS_FPS   # ~16.67ms

sim_state = {
    'running': True,        # Flag to control simulation loop
}

class CommandState:
    """Container for MFD command values."""
    def __init__(self, airspeed_cmd=200, altitude_cmd=10000, heading_cmd=0):
        self.airspeed_cmd = airspeed_cmd
        self.altitude_cmd = altitude_cmd
        self.heading_cmd = heading_cmd

# MFD commands
commands = CommandState()

def _state_vector_to_aircraft_state(outputs, commands):
    """
    Convert state vector to AircraftState for display.
    
    Args:
        y_current: Current state vector
        y_previous: Previous state vector
        dt: Time step
        power: Power setting (0.0 to 1.0)
    """
    
    return AircraftState(
        pitch=outputs.pitch_deg,
        roll=outputs.roll_deg,
        airspeed=outputs.keas,
        airspeed_cmd=commands.airspeed_cmd,
        vspeed=outputs.vspeed_fpm,
        altitude=outputs.altitude_ft,
        altitude_cmd=commands.altitude_cmd,
        heading=outputs.heading_deg,
        heading_cmd=commands.heading_cmd,
        course=outputs.track_deg,
        power=outputs.power,
    )


def _update_controls_from_keyboard(controls_state, shared_state):
    """
    Update control inputs based on keyboard input.
    
    Keyboard mappings:
        'a': Aileron left (ail_deg = -1)
        'd': Aileron right (ail_deg = 1)
        's': Elevator down (elev_deg = -1)
        'w': Elevator up (elev_deg = 1)
    """

    # Trim Input
    if keyboard.is_pressed('u'):  
        shared_state['elev_neutral'] += 3 * PHYSICS_DT
    elif keyboard.is_pressed('n'):
        shared_state['elev_neutral'] -= 3 * PHYSICS_DT
    
    elev_neutral = shared_state['elev_neutral']

    # Reset to neutral
    controls_state.ail_deg = 0
    controls_state.elev_deg = elev_neutral
    controls_state.rudder_deg = 0
    
    # Check aileron inputs
    if keyboard.is_pressed('a'):
        controls_state.ail_deg = 5
    elif keyboard.is_pressed('d'):
        controls_state.ail_deg = -5
    
    # Check elevator inputs
    if keyboard.is_pressed('s'):
        controls_state.elev_deg = elev_neutral - 5
    elif keyboard.is_pressed('w'):
        controls_state.elev_deg = elev_neutral + 5

    # Check rudder inputs
    if keyboard.is_pressed('right'):
        controls_state.rudder_deg = -10
    elif keyboard.is_pressed('left'):
        controls_state.rudder_deg = 10

    # Check throttle inputs
    if keyboard.is_pressed('up'):
        controls_state.throttle = min(controls_state.throttle + 0.1*PHYSICS_DT, 1)
    elif keyboard.is_pressed('down'):
        controls_state.throttle = max(controls_state.throttle - 0.1*PHYSICS_DT, 0.0)

def _update_controls_from_joystick(joystick, controls_state, shared_state):
    """
    Update control inputs based on joystick input.
    
    Joystick mappings (example):
        Axis 0: Aileron (-1 left, +1 right)
        Axis 1: Elevator (-1 up, +1 down)
        Axis 2: Rudder (-1 left, +1 right)
    """

    # Trim Input (using POV hat)
    hat = joystick.get_hat(0)  # Get POV hat position
    if hat[1] == 1:  # POV hat up to increase elevator trim
        shared_state['elev_neutral'] += 3 * PHYSICS_DT
    elif hat[1] == -1:  # POV hat down to decrease elevator trim
        shared_state['elev_neutral'] -= 3 * PHYSICS_DT
    
    elev_neutral = shared_state['elev_neutral']
    throttle_neutral = shared_state['throttle_neutral']

    # Aileron Input
    controls_state.ail_deg = -joystick.get_axis(0) * 21.5  # Scale to degrees

    # Elevator Input
    elev_deg_unclipped = elev_neutral - joystick.get_axis(1) * 25 # Scale to degrees
    controls_state.elev_deg = np.clip(elev_deg_unclipped, - 25, + 25)

    # Throttle Input
    throttle_unclipped = throttle_neutral - (joystick.get_axis(2)) # Scale to 0-100%
    controls_state.throttle = np.clip(throttle_unclipped, 0, 1)

    # Rudder Input
    controls_state.rudder_deg = 0
    if keyboard.is_pressed('right'):
        controls_state.rudder_deg = -10
    elif keyboard.is_pressed('left'):
        controls_state.rudder_deg = 10

def _check_exit_condition():
    if keyboard.is_pressed('e') or keyboard.is_pressed('esc'):
        print("Exit key pressed. Stopping simulation.")
        sim_state['running'] = False


# Flight Control System options
FCS_OPTIONS = {
    'n': 'None',
    'sas': 'Stability Augmentation System',
}


def _show_combined_dialog(current_sas, params):
    """
    Show a combined dialog to select flight control system and update trim parameters.
    Returns a dict with 'sas' and trim parameters, or empty dict if cancelled.
    """
    root = tk.Tk()
    root.withdraw()  # Hide the root window
    
    # Create a custom dialog
    result = {}
    
    def on_submit():
        try:
            result['sas'] = fcs_var.get()
            result['alt_ft'] = float(alt_entry.get())
            result['VT_ftps'] = float(speed_entry.get())
            result['xcg'] = float(xcg_entry.get())
            dialog.destroy()
            root.destroy()
        except ValueError:
            status_label.config(text="Error: Please enter valid numbers", fg="red")
    
    dialog = tk.Toplevel(root)
    dialog.title("Flight Parameters and Control System")
    dialog.geometry("400x350")
    
    # Create canvas and scrollbar
    canvas = tk.Canvas(dialog, highlightthickness=0)
    scrollbar = tk.Scrollbar(dialog, orient="vertical", command=canvas.yview)
    scrollable_frame = tk.Frame(canvas)
    
    scrollable_frame.bind(
        "<Configure>",
        lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
    )
    
    canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)
    
    # Mouse wheel scrolling support
    def _on_mousewheel(event):
        canvas.yview_scroll(int(-1*(event.delta/120)), "units")
    
    canvas.bind_all("<MouseWheel>", _on_mousewheel)
    
    # Pack canvas and scrollbar
    canvas.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)
    scrollbar.grid(row=0, column=2, sticky="ns", pady=5)
    dialog.grid_rowconfigure(0, weight=1)
    
    # FCS Selection Section
    tk.Label(scrollable_frame, text="Flight Control System:", font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=2, sticky="w", padx=10, pady=(10, 5))
    
    fcs_var = tk.StringVar(value=current_sas)
    fcs_menu = tk.OptionMenu(scrollable_frame, fcs_var, *FCS_OPTIONS.keys())
    fcs_menu.grid(row=1, column=0, columnspan=2, padx=10, pady=5)
    
    # Display FCS description
    desc_label = tk.Label(scrollable_frame, text=f"Selected: {FCS_OPTIONS.get(current_sas, 'Unknown')}", fg="blue")
    desc_label.grid(row=2, column=0, columnspan=2, padx=10, pady=5)
    
    def update_desc(*args):
        desc_label.config(text=f"Selected: {FCS_OPTIONS.get(fcs_var.get(), 'Unknown')}")
    
    fcs_var.trace('w', update_desc)
    
    # Separator
    tk.Frame(scrollable_frame, height=2, bd=1, relief="sunken").grid(row=3, column=0, columnspan=2, sticky="ew", padx=10, pady=10)
    
    # Trimming Parameters Section
    tk.Label(scrollable_frame, text="Trim Parameters:", font=("Arial", 10, "bold")).grid(row=4, column=0, columnspan=2, sticky="w", padx=10, pady=(5, 5))
    
    # Altitude
    tk.Label(scrollable_frame, text="Altitude (ft):").grid(row=5, column=0, sticky="w", padx=10, pady=3)
    alt_entry = tk.Entry(scrollable_frame)
    alt_entry.insert(0, str(params.alt_ft))
    alt_entry.grid(row=5, column=1, padx=10, pady=3)
    
    # Speed
    tk.Label(scrollable_frame, text="Speed (ft/s):").grid(row=6, column=0, sticky="w", padx=10, pady=3)
    speed_entry = tk.Entry(scrollable_frame)
    speed_entry.insert(0, str(params.VT_ftps))
    speed_entry.grid(row=6, column=1, padx=10, pady=3)
    
    # XCG
    tk.Label(scrollable_frame, text="XCG (fraction):").grid(row=7, column=0, sticky="w", padx=10, pady=3)
    xcg_entry = tk.Entry(scrollable_frame)
    xcg_entry.insert(0, str(params.xcg))
    xcg_entry.grid(row=7, column=1, padx=10, pady=3)
    
    # Status label
    status_label = tk.Label(scrollable_frame, text="", fg="green")
    status_label.grid(row=8, column=0, columnspan=2, pady=5)
    
    # Buttons (fixed at bottom, not scrollable)
    button_frame = tk.Frame(dialog)
    button_frame.grid(row=1, column=0, columnspan=3, sticky="ew", padx=5, pady=5)
    
    submit_button = tk.Button(button_frame, text="Apply", command=on_submit)
    submit_button.pack(side="right", padx=5)
    
    cancel_button = tk.Button(button_frame, text="Cancel", command=dialog.destroy)
    cancel_button.pack(side="right", padx=5)
    
    root.mainloop()
    
    return result





def simulate_realtime(func, X0, controls_state, params, SAS='n'):
    """
    Run F16 simulation with real-time visualization.
    
    Physics and graphics run in separate threads:
    - Physics thread: 200 Hz update rate
    - Graphics thread: 60 FPS render rate
    
    Args:
        func: Physics function (eqm for F16)
        X0: Initial state vector (12-element array)
        controls_state: Control inputs (Controls object)
        params: Aircraft parameters (F16Params object)
        input_method: 'keyboard' or 'joystick' for control input
    """
    
    # Initialize state
    m = len(X0)
    y_init = np.zeros((m, 1))
    y_init[:, 0:1] = X0  # Ensure column vector format
    
    # Shared state between threads (dict allows updates by reference)
    shared_state = {
        'y': y_init,
        'y_prev': y_init.copy(),
        'elev_neutral': controls_state.elev_deg,
        'throttle_neutral': controls_state.throttle,
        'SAS': SAS,
        'aircraft_state': None,
        'running': True,
    }
    
    # Setup graphics
    pfd = PrimaryFlightDisplay(
        (SCREEN_WIDTH, SCREEN_HEIGHT),
        masked=True,
        max_fps=GRAPHICS_FPS
    )

    input_method = 'keyboard'
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected. Using keyboard input.")
        print("  'w': Elevator up")
        print("  's': Elevator down")
        print("  'a': Aileron left")
        print("  'd': Aileron right")
        print("  'arrow left': Rudder left")
        print("  'arrow right': Rudder right")
        print("  'arrow up': Throttle up")
        print("  'arrow down': Throttle down")
        print("  'u': Elevator trim down")
        print("  'n': Elevator trim up")
        print("  't': Select flight control system")
        print("  'e' or 'esc': Exit simulation")
    else:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Using joystick: {joystick.get_name()}")
        print(f"Axes: {joystick.get_numaxes()}, Buttons: {joystick.get_numbuttons()}")
        print("  'arrow left': Rudder left")
        print("  'arrow right': Rudder right")
        print("  't': Select flight control system")
        print("  'e' or 'esc': Exit simulation")
        input_method = 'joystick'
    
    # =========================================================================
    # PHYSICS THREAD
    # =========================================================================
    def physics_loop():
        """Run physics simulation at constant 200 Hz."""
        while sim_state['running']:
            loop_start = time.time()         
            
            # Check for combined dialog (press 't')
            if keyboard.is_pressed('t'):
                print("Opening flight parameters dialog...")
                time.sleep(0.2)  # Debounce
                result = _show_combined_dialog(shared_state['SAS'], params)
                if result:
                    # Update SAS
                    if 'sas' in result:
                        shared_state['SAS'] = result['sas']
                        print(f"Flight Control System updated to: {FCS_OPTIONS.get(result['sas'], 'Unknown')}")
                    
                    # Update trim parameters
                    if 'alt_ft' in result:
                        params.alt_ft = result['alt_ft']
                        params.VT_ftps = result['VT_ftps']
                        params.xcg = result['xcg']
                        print(f"Parameters updated: Alt={params.alt_ft} ft, Speed={params.VT_ftps} ft/s, XCG={params.xcg}")
                        
                        # Re-trim the aircraft with new parameters
                        try:
                            X0_new, U0_new = trimmer(controls_state, params)
                            shared_state['y'] = X0_new
                            shared_state['y_prev'] = shared_state['y'].copy()
                            controls_state.throttle = U0_new.throttle
                            shared_state['elev_neutral'] = U0_new.elev_deg
                            print("Aircraft re-trimmed successfully.")
                        except Exception as e:
                            print(f"Error re-trimming: {e}")
            
            # Compute next state
            y_next, outputs = RK4(
                func=func,
                y0=shared_state['y'][:, 0],      # Pass 1D state
                h=PHYSICS_DT,
                controls=controls_state,
                params=params
            )

            if input_method == 'joystick':
                # Update controls from joystick input
                _update_controls_from_joystick(joystick, controls_state, shared_state)
            else:
                # Update controls from keyboard input
                _update_controls_from_keyboard(controls_state, shared_state)

            if shared_state['SAS'] != 'n':
                # Update controls from Flight Control System
                update_controls_from_fcs(shared_state['SAS'], outputs, controls_state, shared_state)

            # Update shared state
            shared_state['y_prev'] = shared_state['y'].copy()
            shared_state['y'] = y_next.reshape(-1, 1)  # Reshape to 2D column
            
            # Convert to visualization format
            shared_state['aircraft_state'] = _state_vector_to_aircraft_state(
                outputs,
                commands
            )
            
            # Check for exit signal
            _check_exit_condition()
            if not sim_state['running']:
                break
            
            # Sleep to maintain 200 Hz
            elapsed = time.time() - loop_start
            sleep_time = PHYSICS_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    # =========================================================================
    # START PHYSICS THREAD
    # =========================================================================
    physics_thread = threading.Thread(target=physics_loop, daemon=True)
    physics_thread.start()
    
    # =========================================================================
    # GRAPHICS LOOP (main thread)
    # =========================================================================
    start_time = time.time()
    
    try:
        while shared_state['running']:
            loop_start = time.time()
            elapsed = loop_start - start_time
            
            # Render with latest state from physics thread
            if shared_state['aircraft_state']:
                pfd.update(shared_state['aircraft_state'], elapsed)
                pfd.draw()
                pfd.render()
            
            # Sleep to maintain 60 FPS
            loop_elapsed = time.time() - loop_start
            sleep_time = GRAPHICS_DT - loop_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Exit if physics thread is done
            if not physics_thread.is_alive():
                break
    
    finally:
        # Cleanup
        shared_state['running'] = False
        physics_thread.join(timeout=1.0)
        print("Simulation complete.")
