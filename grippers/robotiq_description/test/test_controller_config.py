# Copyright (c) 2026 Robotiq
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# The gripper controller can only activate if the command interfaces it is
# configured to claim exist under the exact names the hardware exports (see
# RobotiqGripperHardwareInterface::export_command_interfaces in
# robotiq_driver). A mismatch is silent until runtime, where
# robotiq_gripper_controller fails to activate.
#
# There are two configs — Humble has no parallel_gripper_controller package — and
# robotiq_control.launch.py picks one per $ROS_DISTRO. Both are checked here
# regardless of the distro the tests run on, so neither can rot unnoticed.

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext

CONFIG_DIR = Path(__file__).parents[1] / "config"
JAZZY_CONFIG = CONFIG_DIR / "robotiq_controllers.yaml"
HUMBLE_CONFIG = CONFIG_DIR / "robotiq_controllers.humble.yaml"
ALL_CONFIGS = (JAZZY_CONFIG, HUMBLE_CONFIG)

# The configs name the joint through this launch placeholder so one file serves
# both models; robotiq_control.launch.py resolves it via ParameterFile.
JOINT_PLACEHOLDER = "$(var gripper_joint)"

# The simulated plugins (sim_isaac / sim_gazebo) export neither the
# set_gripper_max_* command interfaces nor the reactivate_gripper GPIO, so they
# get a config of their own per distro, selected by the same launch file.
JAZZY_SIM_CONFIG = CONFIG_DIR / "robotiq_controllers.sim.yaml"
HUMBLE_SIM_CONFIG = CONFIG_DIR / "robotiq_controllers.sim.humble.yaml"
SIM_CONFIGS = (JAZZY_SIM_CONFIG, HUMBLE_SIM_CONFIG)
SIM_OF = {JAZZY_CONFIG: JAZZY_SIM_CONFIG, HUMBLE_CONFIG: HUMBLE_SIM_CONFIG}

# Names exported by robotiq_driver's hardware interface. Kept in sync by
# robotiq_driver's test_robotiq_gripper_hardware_interface, which asserts the
# hardware exports exactly these.
JOINT = "robotiq_85_left_knuckle_joint"
UPDATE_RATE_HZ = 500
EXPORTED_COMMAND_INTERFACES = {
    f"{JOINT}/position",
    f"{JOINT}/set_gripper_max_velocity",
    f"{JOINT}/set_gripper_max_effort",
}

# Plugin the launch expects per distro. Humble's stock controller takes a
# control_msgs/GripperCommand goal, matching PickNik's humble branch; Jazzy and
# newer take ParallelGripperCommand.
EXPECTED_CONTROLLER_TYPES = {
    JAZZY_CONFIG: "parallel_gripper_action_controller/GripperActionController",
    HUMBLE_CONFIG: "position_controllers/GripperActionController",
}
EXPECTED_CONTROLLER_TYPES.update(
    {SIM_OF[config]: plugin for config, plugin in EXPECTED_CONTROLLER_TYPES.items()}
)


def load_launch_module():
    """Import robotiq_control.launch.py for its distro -> config-file mapping."""
    path = Path(__file__).parents[1] / "launch" / "robotiq_control.launch.py"
    spec = importlib.util.spec_from_file_location("robotiq_control_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load(config):
    with open(config) as f:
        return yaml.safe_load(f)


def gripper_controller_params(config, joint=JOINT):
    params = load(config)["robotiq_gripper_controller"]["ros__parameters"]
    return {
        k: v.replace(JOINT_PLACEHOLDER, joint) if isinstance(v, str) else v
        for k, v in params.items()
    }


def perform(substitution, **launch_configurations):
    context = LaunchContext()
    context.launch_configurations.update(launch_configurations)
    return substitution.perform(context)


def test_max_effort_interface_is_exported():
    params = gripper_controller_params(JAZZY_CONFIG)
    assert params["max_effort_interface"] in EXPORTED_COMMAND_INTERFACES


def test_max_velocity_interface_is_exported():
    params = gripper_controller_params(JAZZY_CONFIG)
    assert params["max_velocity_interface"] in EXPORTED_COMMAND_INTERFACES


def test_humble_config_claims_no_unsupported_interfaces():
    # Humble's gripper_controllers declares neither parameter; leaving them in
    # would read as working speed/force control that Humble silently ignores.
    params = gripper_controller_params(HUMBLE_CONFIG)
    assert "max_effort_interface" not in params
    assert "max_velocity_interface" not in params


@pytest.mark.parametrize("config", ALL_CONFIGS + SIM_CONFIGS)
def test_joint_matches_exported_interfaces(config):
    assert gripper_controller_params(config)["joint"] == JOINT


@pytest.mark.parametrize("config", ALL_CONFIGS + SIM_CONFIGS)
def test_joint_is_left_to_the_launch(config):
    # Hardcoding the 2F-85 joint here is why a 2F-140 launch used to fail to
    # activate robotiq_gripper_controller.
    raw = load(config)["robotiq_gripper_controller"]["ros__parameters"]
    assert raw["joint"] == JOINT_PLACEHOLDER
    assert JOINT not in yaml.safe_dump(raw)


def test_launch_description_builds():
    # Import errors in the launch file only surface when `ros2 launch` runs it.
    load_launch_module().generate_launch_description()


@pytest.mark.parametrize(
    "model,expected",
    [
        ("/opt/share/urdf/robotiq_2f_85_gripper.urdf.xacro", JOINT),
        ("/opt/share/urdf/robotiq_2f_140_gripper.urdf.xacro", "finger_joint"),
        ("/home/me/my_cell.urdf.xacro", JOINT),
    ],
)
def test_launch_defaults_the_joint_from_the_model(model, expected):
    launch_module = load_launch_module()
    assert perform(launch_module.default_gripper_joint(), model=model) == expected


@pytest.mark.parametrize("config", ALL_CONFIGS + SIM_CONFIGS)
def test_controller_type_matches_distro(config):
    controllers = load(config)["controller_manager"]["ros__parameters"]
    assert (
        controllers["robotiq_gripper_controller"]["type"]
        == EXPECTED_CONTROLLER_TYPES[config]
    )


@pytest.mark.parametrize(
    "distro,expected",
    [
        ("humble", HUMBLE_CONFIG),
        ("jazzy", JAZZY_CONFIG),
        ("lyrical", JAZZY_CONFIG),
        (None, JAZZY_CONFIG),  # ROS_DISTRO unset: newest layout is the default
    ],
)
def test_launch_selects_the_config_for_the_distro(distro, expected):
    launch_module = load_launch_module()
    selected = launch_module.controllers_file_for_distro(distro)
    assert selected == expected.name
    assert (CONFIG_DIR / selected).is_file()

    simulated = launch_module.controllers_file_for_distro(distro, simulated=True)
    assert simulated == SIM_OF[expected].name
    assert (CONFIG_DIR / simulated).is_file()


requires_launch = pytest.mark.skipif(
    importlib.util.find_spec("launch_ros") is None, reason="launch_ros not importable"
)


def spawned_controllers(distro, monkeypatch, **launch_arguments):
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node

    if distro is None:
        monkeypatch.delenv("ROS_DISTRO", raising=False)
    else:
        monkeypatch.setenv("ROS_DISTRO", distro)
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "sim_isaac": "false",
            "sim_gazebo": "false",
            "gripper_joint": JOINT,
            **launch_arguments,
        }
    )

    spawned = {}
    for action in load_launch_module().generate_launch_description().entities:
        if not isinstance(action, Node) or action.node_executable != "spawner":
            continue
        if action.condition is not None and not action.condition.evaluate(context):
            continue
        cmd = [perform_substitutions(context, part) for part in action.cmd]
        spawned[cmd[1]] = Path(cmd[cmd.index("--param-file") + 1]).read_text()
    return spawned


def resolved(config, joint=JOINT):
    return config.read_text().replace(JOINT_PLACEHOLDER, joint)


@requires_launch
@pytest.mark.parametrize(
    "distro,hardware_config,sim_config",
    [
        ("humble", HUMBLE_CONFIG, HUMBLE_SIM_CONFIG),
        ("jazzy", JAZZY_CONFIG, JAZZY_SIM_CONFIG),
        (None, JAZZY_CONFIG, JAZZY_SIM_CONFIG),
    ],
)
@pytest.mark.parametrize("sim_argument", ["sim_isaac", "sim_gazebo"])
def test_launch_spawns_per_plugin(
    distro, hardware_config, sim_config, sim_argument, monkeypatch
):
    # Every spawner must be handed the same config the controller_manager loaded,
    # and the activation controller only exists where its GPIO does.
    hardware = spawned_controllers(distro, monkeypatch)
    assert hardware == {
        "joint_state_broadcaster": resolved(hardware_config),
        "robotiq_gripper_controller": resolved(hardware_config),
        "robotiq_activation_controller": resolved(hardware_config),
    }

    simulated = spawned_controllers(distro, monkeypatch, **{sim_argument: "true"})
    assert simulated == {
        "joint_state_broadcaster": resolved(sim_config),
        "robotiq_gripper_controller": resolved(sim_config),
    }


@pytest.mark.parametrize("config", SIM_CONFIGS)
def test_sim_configs_claim_no_driver_only_command_interfaces(config):
    # TopicBasedSystem and GazeboSimSystem export the standard joint interfaces
    # only; claiming set_gripper_max_* here is why the sim paths never activated.
    params = gripper_controller_params(config)
    assert "max_effort_interface" not in params
    assert "max_velocity_interface" not in params


@pytest.mark.parametrize("config", SIM_CONFIGS)
def test_sim_configs_spawn_no_activation_controller(config):
    # No reactivate_gripper GPIO is declared under sim_isaac / sim_gazebo (see
    # 2f_85.ros2_control.xacro), so the controller would have nothing to claim.
    controllers = load(config)["controller_manager"]["ros__parameters"]
    assert set(controllers) == {
        "update_rate",
        "joint_state_broadcaster",
        "robotiq_gripper_controller",
    }
    assert controllers["update_rate"] == UPDATE_RATE_HZ
    assert "robotiq_activation_controller" not in load(config)


@pytest.mark.parametrize("config", ALL_CONFIGS)
def test_sim_config_keeps_the_action_surface_of_its_distro(config):
    # Same controller name, type, joint and goal tolerance as the hardware config
    # for that distro: an action client must not notice which plugin is behind.
    hardware = gripper_controller_params(config)
    sim = gripper_controller_params(SIM_OF[config])
    assert sim["joint"] == hardware["joint"]
    assert sim["goal_tolerance"] == hardware["goal_tolerance"]
    assert sim["allow_stalling"] == hardware["allow_stalling"]
    assert (
        load(SIM_OF[config])["controller_manager"]["ros__parameters"][
            "robotiq_gripper_controller"
        ]
        == load(config)["controller_manager"]["ros__parameters"][
            "robotiq_gripper_controller"
        ]
    )


@pytest.mark.parametrize("config", ALL_CONFIGS)
def test_controller_names_and_update_rate_are_identical_across_distros(config):
    # The controller names are the action/service namespaces users depend on
    # (/robotiq_gripper_controller/gripper_cmd), so they must not drift between
    # the two configs.
    controllers = load(config)["controller_manager"]["ros__parameters"]
    assert set(controllers) == {
        "update_rate",
        "joint_state_broadcaster",
        "robotiq_gripper_controller",
        "robotiq_activation_controller",
    }
    assert controllers["update_rate"] == UPDATE_RATE_HZ
    assert (
        controllers["robotiq_activation_controller"]["type"]
        == "robotiq_controllers/RobotiqActivationController"
    )
    assert (
        controllers["joint_state_broadcaster"]["type"]
        == "joint_state_broadcaster/JointStateBroadcaster"
    )
