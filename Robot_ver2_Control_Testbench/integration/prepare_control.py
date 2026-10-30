"""Create a separate integration checkout. Never modifies the supplied repository."""
from pathlib import Path
import argparse
import shutil

ROOT = Path(__file__).resolve().parents[1]
PIN = "15aa22efc071554fe06f1e42e1007633dee15cc9"

def function_span(text, signature):
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        if text[end] == "{": depth += 1
        if text[end] == "}": depth -= 1
        end += 1
    return opening, end

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--destination", type=Path, default=ROOT / "build" / "control_integrated")
    args = parser.parse_args()
    destination = args.destination.resolve()
    source = ROOT / "vendor" / ("GPRobotControl_Simulation-" + PIN)
    if destination.exists():
        raise SystemExit("Destination exists; choose a new empty destination. No files changed.")
    if destination == source or source in destination.parents:
        raise SystemExit("Destination must be separate from the source snapshot.")
    shutil.copytree(source, destination)
    local = destination / "RobotVer2"
    local.mkdir()
    for filename in ("robot_ver2_control_adapter.c", "robot_ver2_control_adapter.h"):
        shutil.copy2(ROOT / "integration" / filename, local / filename)
    for filename in ("robot_ver2_kinematics.c", "robot_ver2_kinematics.h", "robot_ver2_geometry.h", "robot_ver2_profile.h"):
        shutil.copy2(ROOT / "control" / filename, local / filename)
    replacements = {
        "control_fk.c": ("void control_fk(", "if (rb2_is_robot(robot)) { if (!rb2_fk(q, &T_B_TCP[0][0])) memset(T_B_TCP,0,16*sizeof(double)); return; }"),
        "control_jacobian.c": ("void control_jacobian(", "if (rb2_is_robot(robot)) { if (!rb2_jacobian(q, &J[0][0])) memset(J,0,36*sizeof(double)); return; }"),
        "adls_ik.c": ("bool adls_ik(", "if (rb2_is_robot(robot)) return rb2_control_ik(robot,T_B_TCP_target,qSeed,qSolution,info);")
    }
    for filename, (signature, dispatch) in replacements.items():
        path = destination / "ControlCore" / "Kinematics" / filename
        text = path.read_text()
        opening, _ = function_span(text, signature)
        text = text[:opening+1] + "\n    " + dispatch + "\n" + text[opening+1:]
        path.write_text('#include "robot_ver2_control_adapter.h"\n#include <string.h>\n' + text)
    cmake = destination / "ControlCore" / "CMakeLists.txt"
    cmake.write_text(cmake.read_text() + '\ntarget_sources(control_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../RobotVer2/robot_ver2_control_adapter.c ${CMAKE_CURRENT_SOURCE_DIR}/../RobotVer2/robot_ver2_kinematics.c)\ntarget_include_directories(control_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR}/../RobotVer2)\n')
    # Only the simulation composition root selects the new robot. UR5 tests stay intact.
    for relative in ("Simulation/supervisor_silkit_main.c", "Simulation/Robot/robot_runtime.c"):
        path = destination / relative
        text = path.read_text()
        opening, end = function_span(text, "static void configure_robot(void)")
        text = text[:opening] + "{ robot_config_init_robot_ver2(&robot); }" + text[end:]
        path.write_text('#include "robot_ver2_control_adapter.h"\n' + text)
    # Preserve original transport implementation, replace ONLY motor-bank main.
    shutil.copy2(ROOT / "integration" / "physics_motor_bank.cpp", destination / "Simulation" / "AvatarM_CAN" / "avatar_m_silkit_motor_bank.cpp")
    shutil.copy2(ROOT / "integration" / "physics_protocol.h", destination / "Simulation" / "AvatarM_CAN" / "physics_protocol.h")
    (destination / "ROBOT_VER2_INTEGRATION.txt").write_text(
        "SIMULATION COPY ONLY. Source snapshot " + PIN + "\n"
        "ControlCore dispatches robot_ver2 to packaged FK/Jacobian/analytical IK.\n"
        "UR5 remains available for its regression tests. Legacy adls_ik API returns the nearest analytical branch for robot_ver2; iterations=0.\n"
        "Simulation composition selects compact home. Original hardware authorization is unchanged.\n"
        "Motor bank uses measured MATLAB feedback instead of its ideal follower.\n"
        "No physical commissioning is provided. See package TEAMMATE_CHANGES.md.\n")
    print(destination)

if __name__ == "__main__":
    main()
