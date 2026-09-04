import argparse
import json
import time
from collections import OrderedDict

from pynput.keyboard import Key, Listener
import robosuite
from robosuite.controllers import load_composite_controller_config
from robosuite.wrappers import VisualizationWrapper
from termcolor import colored

import robocasa.macros as macros
from robocasa.scripts.collect_demos import collect_human_trajectory
from robocasa.wrappers.enclosing_wall_render_wrapper import (
    EnclosingWallRenderWrapper,
    install_enclosing_wall_hotkeys,
)


def choose_option(
    options, option_name, show_keys=False, default=None, default_message=None
):
    """
    Prints out environment options, and returns the selected env_name choice

    Returns:
        str: Chosen environment name
    """
    # get the list of all tasks

    if default is None:
        default = options[0]

    if default_message is None:
        default_message = default

    # Select environment to run
    print("Here is a list of {}s:\n".format(option_name))

    for i, (k, v) in enumerate(options.items()):
        if show_keys:
            print("[{}] {}: {}".format(i, k, v))
        else:
            print("[{}] {}".format(i, v))
    print()
    try:
        s = input(
            "Choose an option 0 to {}, or any other key for default ({}): ".format(
                len(options) - 1,
                default_message,
            )
        )
        # parse input into a number within range
        k = min(max(int(s), 0), len(options) - 1)
        choice = list(options.keys())[k]
    except:
        if default is None:
            choice = options[0]
        else:
            choice = default
        print("Use {} by default.\n".format(choice))

    # Return the chosen environment name
    return choice


def install_key_press_diagnostics(env):
    """
    Annotate the robot pose diagnostics csv (robot-pose-diagnostics.csv) with
    the keyboard keys that command the robot. Each recorded key press is
    queued on the base env and tagged onto the "key" column of the next
    diagnostics row written, so the arm poses responding to each key can be
    identified in the csv.

    Only keys the Keyboard device turns into robot commands are recorded:
    motion keys on press (arrows, . ; e r y h o p) and action keys on release
    (space toggles gripper, b base mode, s arm switch, = robot switch, q reset).
    """
    # idempotent: if already installed, no-op
    if getattr(env, "_key_press_diag_listener", None) is not None:
        return

    base_env = env
    while hasattr(base_env, "env"):
        base_env = base_env.env

    motion_chars = {".", ";", "e", "r", "y", "h", "o", "p"}
    action_chars = {"b", "s", "=", "q"}

    def record(key_name):
        key_presses = getattr(base_env, "_diag_key_presses", None)
        if key_presses is not None:
            key_presses.append(key_name)

    def on_press(key):
        if key in (Key.up, Key.down, Key.left, Key.right):
            record(key.name)
            return
        char = getattr(key, "char", None)
        if char in motion_chars:
            record(char)

    def on_release(key):
        if key == Key.space:
            record("space")
            return
        char = getattr(key, "char", None)
        if char in action_chars:
            record(char)

    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()
    env._key_press_diag_listener = listener


if __name__ == "__main__":
    # Arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=str, help="task (choose among 365 tasks)")
    parser.add_argument(
        "--layout", type=int, help="kitchen layout (choose number 1-60)"
    )
    parser.add_argument("--style", type=int, help="kitchen style (choose number 1-60)")
    parser.add_argument(
        "--device",
        type=str,
        default="keyboard",
        choices=["keyboard", "spacemouse"],
        help="Teleop device (default: keyboard)",
    )
    args = parser.parse_args()

    tasks = OrderedDict(
        [
            ("PickPlaceCounterToCabinet", "pick and place from counter to cabinet"),
            ("PickPlaceCounterToSink", "pick and place from counter to sink"),
            ("PickPlaceMicrowaveToCounter", "pick and place from microwave to counter"),
            ("PickPlaceStoveToCounter", "pick and place from stove to counter"),
            ("OpenSingleDoor", "open cabinet or microwave door"),
            ("CloseDrawer", "close drawer"),
            ("TurnOnMicrowave", "turn on microwave"),
            ("TurnOnSinkFaucet", "turn on sink faucet"),
            ("TurnOnStove", "turn on stove"),
            ("ArrangeVegetables", "arrange vegetables on a cutting board"),
            ("MicrowaveThawing", "place frozen food in microwave for thawing"),
            ("RestockPantry", "restock cans in pantry"),
            ("PreSoakPan", "prepare pan for washing"),
            ("PrepareCoffee", "make coffee"),
        ]
    )

    if args.task is None:
        args.task = choose_option(
            tasks, "task", default="PickPlaceCounterToCabinet", show_keys=True
        )

    # Create argument configuration
    config = {
        "env_name": args.task,
        "robots": "PandaOmron",
        "controller_configs": load_composite_controller_config(robot="PandaOmron"),
        "layout_ids": args.layout,
        "style_ids": args.style,
        "translucent_robot": True,
    }

    args.renderer = "mjviewer"

    print(colored(f"Initializing environment...", "yellow"))
    env = robosuite.make(
        **config,
        has_renderer=True,
        has_offscreen_renderer=False,
        render_camera="robot0_frontview",
        ignore_done=True,
        use_camera_obs=False,
        control_freq=20,
        renderer=args.renderer,
        seed=28,
    )

    # Wrap this with visualization wrapper
    env = VisualizationWrapper(env)
    env = EnclosingWallRenderWrapper(env, alpha=0.1, enabled=False)
    install_enclosing_wall_hotkeys(env)

    # Grab reference to controller config and convert it to json-encoded string
    env_info = json.dumps(config)

    # initialize device
    device = args.device
    if device == "keyboard":
        from robosuite.devices import Keyboard

        device = Keyboard(env=env, pos_sensitivity=4.0, rot_sensitivity=4.0)
        install_key_press_diagnostics(env)
    elif device == "spacemouse":
        from robosuite.devices import SpaceMouse

        device = SpaceMouse(
            env=env,
            pos_sensitivity=4.0,
            rot_sensitivity=4.0,
            vendor_id=macros.SPACEMOUSE_VENDOR_ID,
            product_id=macros.SPACEMOUSE_PRODUCT_ID,
        )
    else:
        raise ValueError

    # collect demonstrations
    while True:
        ep_directory, discard_traj = collect_human_trajectory(
            env,
            device,
            "right",
            "single-arm-opposed",
            mirror_actions=True,
            render=(args.renderer != "mjviewer"),
            max_fr=30,
        )
        print()
