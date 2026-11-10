#!/usr/bin/env python3
"""Inject a joint force/torque sensor into an already-converted SDF.

Why a separate step: the URDF->SDF conversion drops <sensor> elements that are
mounted on a <joint> via <gazebo reference="...">, so the sensor has to be added
after conversion.

    python3 inject_joint_ft_sensor.py <in.sdf> <out.sdf> [joint_name]

Default joint: left_finger1_joint.  The topic is /<joint>_ft and the message is
ignition.msgs.WrenchStamped (gz sim force_torque sensor).
"""
import sys
import xml.etree.ElementTree as ET

SENSOR_TMPL = """
      <sensor name="{joint}_ft" type="force_torque">
        <always_on>true</always_on>
        <update_rate>200</update_rate>
        <topic>/{joint}_ft</topic>
        <force_torque>
          <frame>child</frame>
          <measure_direction>child_to_parent</measure_direction>
        </force_torque>
      </sensor>
"""


def main():
    src, dst = sys.argv[1], sys.argv[2]
    joint_name = sys.argv[3] if len(sys.argv) > 3 else "left_finger1_joint"

    tree = ET.parse(src)
    root = tree.getroot()
    hits = 0
    for joint in root.iter("joint"):
        if joint.get("name") != joint_name:
            continue
        if any(s.get("name") == f"{joint_name}_ft" for s in joint.iter("sensor")):
            hits += 1
            continue
        joint.append(ET.fromstring(SENSOR_TMPL.format(joint=joint_name)))
        hits += 1
    if hits == 0:
        sys.stderr.write(f"joint '{joint_name}' not found in {src}\n")
        return 1
    tree.write(dst, encoding="utf-8", xml_declaration=False)
    print(f"injected {joint_name}_ft -> {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
