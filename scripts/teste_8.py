from pathlib import Path

import yaml
from xarm.wrapper import XArmAPI

cfg = yaml.safe_load((Path(__file__).parent.parent / "config" / "robot.yaml").read_text())


def check(code, what):
    if code != 0:
        raise RuntimeError(f"{what} falhou com código {code}")


arm = XArmAPI(cfg["ip"], is_radian=False, check_joint_limit=False)
try:
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)

    code, angles = arm.get_servo_angle()
    check(code, "get_servo_angle")
    code, pose = arm.get_position()
    check(code, "get_position")
    print("angulos (graus):", angles)
    print("pose TCP [mm, graus]:", pose)

    pose_a = [0, 0, 0, 0, 0, 0]
    code, fk = arm.get_forward_kinematics(pose_a, input_is_radian=False, return_is_radian=False)
    check(code, "get_forward_kinematics")
    print("FK da Pose A (SDK):", fk)

    tf2 = [87.0, 0.0, 154.0]  # mm, do tf2_echo (0.087, 0.000, 0.154 m)
    diff = [fk[i] - tf2[i] for i in range(3)]
    print("diferenca SDK - tf2 (mm):", diff)
finally:
    arm.disconnect()