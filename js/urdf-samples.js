// Sample robots built from primitives, so they load with no mesh files.
// Proportions are loosely modeled on common arms; they are not real products.

const MATERIALS = `
  <material name="shell"><color rgba="0.86 0.88 0.91 1"/></material>
  <material name="joint"><color rgba="0.90 0.65 0.04 1"/></material>
  <material name="dark"><color rgba="0.17 0.20 0.25 1"/></material>`;

// A cylinder along an axis, as a joint housing.
const housing = (axis, r, l, xyz = "0 0 0") => {
  const rpy = axis === "y" ? "1.5708 0 0" : axis === "x" ? "0 1.5708 0" : "0 0 0";
  return `<visual><origin xyz="${xyz}" rpy="${rpy}"/><geometry><cylinder radius="${r}" length="${l}"/></geometry><material name="joint"/></visual>`;
};

const arm6 = `<?xml version="1.0"?>
<robot name="six_axis_arm">${MATERIALS}
  <link name="base_link">
    <visual><origin xyz="0 0 0.04"/><geometry><cylinder radius="0.09" length="0.08"/></geometry><material name="dark"/></visual>
    <collision><origin xyz="0 0 0.04"/><geometry><cylinder radius="0.09" length="0.08"/></geometry></collision>
  </link>
  <link name="shoulder_link">
    <visual><origin xyz="0 0 0.06"/><geometry><cylinder radius="0.065" length="0.12"/></geometry><material name="shell"/></visual>
    ${housing("y", 0.07, 0.16, "0 0 0.12")}
    <collision><origin xyz="0 0 0.06"/><geometry><cylinder radius="0.065" length="0.12"/></geometry></collision>
  </link>
  <link name="upper_arm_link">
    <visual><origin xyz="0 0 0.21"/><geometry><cylinder radius="0.05" length="0.42"/></geometry><material name="shell"/></visual>
    ${housing("y", 0.06, 0.14, "0 0 0.42")}
    <collision><origin xyz="0 0 0.21"/><geometry><box size="0.1 0.1 0.42"/></geometry></collision>
  </link>
  <link name="forearm_link">
    <visual><origin xyz="0 0 0.19"/><geometry><cylinder radius="0.04" length="0.38"/></geometry><material name="shell"/></visual>
    ${housing("y", 0.045, 0.11, "0 0 0.38")}
    <collision><origin xyz="0 0 0.19"/><geometry><box size="0.08 0.08 0.38"/></geometry></collision>
  </link>
  <link name="wrist_1_link">
    <visual><origin xyz="0 0 0.04"/><geometry><cylinder radius="0.038" length="0.08"/></geometry><material name="shell"/></visual>
    ${housing("z", 0.042, 0.04, "0 0 0.08")}
  </link>
  <link name="wrist_2_link">
    <visual><origin xyz="0 0 0.04"/><geometry><cylinder radius="0.036" length="0.08"/></geometry><material name="shell"/></visual>
    ${housing("y", 0.04, 0.09, "0 0 0.08")}
  </link>
  <link name="wrist_3_link">
    <visual><origin xyz="0 0 0.02"/><geometry><cylinder radius="0.032" length="0.04"/></geometry><material name="dark"/></visual>
  </link>
  <link name="tool0">
    <visual><origin xyz="0 0 0.02"/><geometry><box size="0.09 0.03 0.04"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="0.035 0 0.065"/><geometry><box size="0.015 0.025 0.05"/></geometry><material name="joint"/></visual>
    <visual><origin xyz="-0.035 0 0.065"/><geometry><box size="0.015 0.025 0.05"/></geometry><material name="joint"/></visual>
  </link>
  <link name="tcp"/>

  <joint name="shoulder_pan" type="revolute"><parent link="base_link"/><child link="shoulder_link"/>
    <origin xyz="0 0 0.08"/><axis xyz="0 0 1"/><limit lower="-3.1416" upper="3.1416" effort="150" velocity="3.1"/></joint>
  <joint name="shoulder_lift" type="revolute"><parent link="shoulder_link"/><child link="upper_arm_link"/>
    <origin xyz="0 0 0.12"/><axis xyz="0 1 0"/><limit lower="-2.2" upper="2.2" effort="150" velocity="3.1"/></joint>
  <joint name="elbow" type="revolute"><parent link="upper_arm_link"/><child link="forearm_link"/>
    <origin xyz="0 0 0.42"/><axis xyz="0 1 0"/><limit lower="-2.7" upper="2.7" effort="100" velocity="3.1"/></joint>
  <joint name="wrist_1" type="revolute"><parent link="forearm_link"/><child link="wrist_1_link"/>
    <origin xyz="0 0 0.38"/><axis xyz="0 1 0"/><limit lower="-3.1416" upper="3.1416" effort="30" velocity="3.1"/></joint>
  <joint name="wrist_2" type="revolute"><parent link="wrist_1_link"/><child link="wrist_2_link"/>
    <origin xyz="0 0 0.08"/><axis xyz="0 0 1"/><limit lower="-3.1416" upper="3.1416" effort="30" velocity="3.1"/></joint>
  <joint name="wrist_3" type="revolute"><parent link="wrist_2_link"/><child link="wrist_3_link"/>
    <origin xyz="0 0 0.08"/><axis xyz="0 1 0"/><limit lower="-3.1416" upper="3.1416" effort="30" velocity="3.1"/></joint>
  <joint name="flange" type="fixed"><parent link="wrist_3_link"/><child link="tool0"/><origin xyz="0 0 0.04"/></joint>
  <joint name="tcp_joint" type="fixed"><parent link="tool0"/><child link="tcp"/><origin xyz="0 0 0.09"/></joint>
</robot>`;

const scara = `<?xml version="1.0"?>
<robot name="scara">${MATERIALS}
  <link name="base_link">
    <visual><origin xyz="0 0 0.02"/><geometry><box size="0.26 0.26 0.04"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="0 0 0.22"/><geometry><cylinder radius="0.08" length="0.4"/></geometry><material name="shell"/></visual>
  </link>
  <link name="arm_1">
    ${housing("z", 0.085, 0.08, "0 0 0.04")}
    <visual><origin xyz="0.175 0 0.04"/><geometry><box size="0.35 0.11 0.07"/></geometry><material name="shell"/></visual>
  </link>
  <link name="arm_2">
    ${housing("z", 0.07, 0.07, "0 0 0.035")}
    <visual><origin xyz="0.15 0 0.035"/><geometry><box size="0.3 0.09 0.06"/></geometry><material name="shell"/></visual>
    ${housing("z", 0.045, 0.12, "0.3 0 0.05")}
  </link>
  <link name="quill">
    <visual><origin xyz="0 0 0.0"/><geometry><cylinder radius="0.014" length="0.4"/></geometry><material name="dark"/></visual>
  </link>
  <link name="tool">
    <visual><origin xyz="0 0 -0.015"/><geometry><box size="0.07 0.03 0.03"/></geometry><material name="joint"/></visual>
  </link>

  <joint name="j1" type="revolute"><parent link="base_link"/><child link="arm_1"/>
    <origin xyz="0 0 0.42"/><axis xyz="0 0 1"/><limit lower="-2.5" upper="2.5" effort="50" velocity="3"/></joint>
  <joint name="j2" type="revolute"><parent link="arm_1"/><child link="arm_2"/>
    <origin xyz="0.35 0 0.07"/><axis xyz="0 0 1"/><limit lower="-2.6" upper="2.6" effort="50" velocity="3"/></joint>
  <joint name="j3_quill" type="prismatic"><parent link="arm_2"/><child link="quill"/>
    <origin xyz="0.3 0 0.05"/><axis xyz="0 0 1"/><limit lower="-0.2" upper="0" effort="100" velocity="0.5"/></joint>
  <joint name="j4" type="continuous"><parent link="quill"/><child link="tool"/>
    <origin xyz="0 0 -0.2"/><axis xyz="0 0 1"/></joint>
</robot>`;

const planar = `<?xml version="1.0"?>
<robot name="two_link_planar">${MATERIALS}
  <link name="base_link">
    <visual><origin xyz="0 0 0.025"/><geometry><cylinder radius="0.07" length="0.05"/></geometry><material name="dark"/></visual>
  </link>
  <link name="link_1">
    ${housing("z", 0.05, 0.05, "0 0 0.025")}
    <visual><origin xyz="0.25 0 0.025"/><geometry><box size="0.5 0.05 0.03"/></geometry><material name="shell"/></visual>
  </link>
  <link name="link_2">
    ${housing("z", 0.042, 0.05, "0 0 0.025")}
    <visual><origin xyz="0.2 0 0.025"/><geometry><box size="0.4 0.04 0.025"/></geometry><material name="shell"/></visual>
  </link>
  <link name="tip">
    <visual><geometry><sphere radius="0.025"/></geometry><material name="joint"/></visual>
  </link>

  <joint name="theta_1" type="revolute"><parent link="base_link"/><child link="link_1"/>
    <origin xyz="0 0 0.05"/><axis xyz="0 0 1"/><limit lower="-3.1416" upper="3.1416" effort="10" velocity="3"/></joint>
  <joint name="theta_2" type="revolute"><parent link="link_1"/><child link="link_2"/>
    <origin xyz="0.5 0 0.05"/><axis xyz="0 0 1"/><limit lower="-2.8" upper="2.8" effort="10" velocity="3"/></joint>
  <joint name="tip_joint" type="fixed"><parent link="link_2"/><child link="tip"/><origin xyz="0.4 0 0.025"/></joint>
</robot>`;

export const SAMPLES = [
  { id: "arm6", name: "6-axis arm", urdf: arm6, pose: { shoulder_lift: -0.6, elbow: 1.2, wrist_1: 0.5, wrist_2: 0.7 } },
  { id: "scara", name: "SCARA (with a prismatic joint)", urdf: scara, pose: { j1: 0.5, j2: -0.9, j3_quill: -0.08 } },
  { id: "planar", name: "2-link planar arm", urdf: planar, pose: { theta_1: 0.5, theta_2: 0.9 } },
];
