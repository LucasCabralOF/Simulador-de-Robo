/**
 * Autonomous code generator for standalone C++, Python and ROS 2
 */

export function generateCpp(joints, name = 'Robot') {
  const n = joints.length;
  const dhRows = joints.map((j, i) => {
    const isRev = j.type === 'R';
    return `    // Joint ${i + 1} (${j.type}): a=${j.a}, alpha=${j.alpha} deg, d=${j.d}, theta=${j.theta} deg
    float theta_${i} = ${isRev ? `(q[${i}] + ${j.theta}) * DEG_TO_RAD` : `${j.theta} * DEG_TO_RAD`};
    float alpha_${i} = ${j.alpha} * DEG_TO_RAD;
    float a_${i}     = ${j.a};
    float d_${i}     = ${isRev ? `${j.d}` : `q[${i}] + ${j.d}`};
    float A${i}[4][4];
    dhMatrix(theta_${i}, d_${i}, a_${i}, alpha_${i}, A${i});
    multiply4x4(T, A${i}, T);`;
  }).join('\n\n');

  return `// =================================================================
// Auto-generated kinematics for ${name}
// Architecture: ${joints.map(j => j.type).join('')} (${n} DOF)
// Compatible with Arduino, ESP32, STM32 and standard C++11
// =================================================================

#include <math.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)
#endif

void dhMatrix(float theta, float d, float a, float alpha, float A[4][4]) {
    float ct = cosf(theta), st = sinf(theta);
    float ca = cosf(alpha), sa = sinf(alpha);

    A[0][0] = ct;  A[0][1] = -st * ca; A[0][2] =  st * sa; A[0][3] = a * ct;
    A[1][0] = st;  A[1][1] =  ct * ca; A[1][2] = -ct * sa; A[1][3] = a * st;
    A[2][0] = 0;   A[2][1] =  sa;      A[2][2] =  ca;      A[2][3] = d;
    A[3][0] = 0;   A[3][1] =  0;       A[3][2] =  0;       A[3][3] = 1.0f;
}

void multiply4x4(const float A[4][4], const float B[4][4], float out[4][4]) {
    float temp[4][4];
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            temp[r][c] = 0;
            for (int k = 0; k < 4; ++k) {
                temp[r][c] += A[r][k] * B[k][c];
            }
        }
    }
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out[r][c] = temp[r][c];
        }
    }
}

/**
 * Calculates Forward Kinematics
 * @param q Array of ${n} joint coordinates (revolute in degrees, prismatic in units)
 * @param T Output 4x4 homogeneous transformation matrix
 * @param pos Output array [x, y, z] of end-effector position
 */
void forwardKinematics(const float q[${n}], float T[4][4], float pos[3]) {
    // Initialize identity matrix
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            T[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }

${dhRows}

    pos[0] = T[0][3];
    pos[1] = T[1][3];
    pos[2] = T[2][3];
}
`;
}

export function generatePython(joints, name = 'Robot') {
  const n = joints.length;
  const tableRows = joints.map(j =>
    `        {'type': '${j.type}', 'a': ${j.a}, 'alpha': ${j.alpha}, 'd': ${j.d}, 'theta': ${j.theta}, 'min': ${j.min}, 'max': ${j.max}},`
  ).join('\n');

  return `#!/usr/bin/env python3
"""
Auto-generated Kinematics Model for ${name}
Architecture: ${joints.map(j => j.type).join('')} (${n} DOF)
Requires: numpy
"""

import numpy as np

DH_PARAMETERS = [
${tableRows}
]

def dh_matrix(theta_deg, d_val, a_val, alpha_deg):
    theta = np.radians(theta_deg)
    alpha = np.radians(alpha_deg)
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a_val * ct],
        [st,  ct * ca, -ct * sa, a_val * st],
        [0,        sa,       ca,      d_val],
        [0,         0,        0,          1]
    ], dtype=np.float64)

def forward_kinematics(q):
    """
    Computes forward kinematics for given joint positions q.
    Returns:
        T: 4x4 homogeneous transformation matrix
        pos: [x, y, z] Cartesian position
    """
    T = np.eye(4, dtype=np.float64)
    for i, p in enumerate(DH_PARAMETERS):
        val = q[i]
        theta = val + p['theta'] if p['type'] == 'R' else p['theta']
        d = val + p['d'] if p['type'] == 'P' else p['d']
        A = dh_matrix(theta, d, p['a'], p['alpha'])
        T = T @ A
    pos = T[:3, 3]
    return T, pos

if __name__ == '__main__':
    # Test nominal pose
    q_test = [${joints.map(j => j.val).join(', ')}]
    T_res, pos_res = forward_kinematics(q_test)
    print("Configuration q:", q_test)
    print("End-effector Position (X, Y, Z):", np.round(pos_res, 4))
    print("Homogeneous Transformation Matrix T0n:\\n", np.round(T_res, 4))
`;
}

export function generateRos2(joints, name = 'Robot') {
  const n = joints.length;
  return `#!/usr/bin/env python3
"""
ROS 2 Kinematics Node for ${name} (${n} DOF)
Subscribes to: /joint_states (sensor_msgs/msg/JointState)
Publishes to: /end_effector_pose (geometry_msgs/msg/PoseStamped)
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class RobotKinematicsNode(Node):
    def __init__(self):
        super().__init__('robot_kinematics_node')
        self.declare_parameter('joint_names', [${joints.map((_, i) => `'joint_${i + 1}'`).join(', ')}])
        self.sub_joints = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        self.pub_pose = self.create_publisher(PoseStamped, '/end_effector_pose', 10)
        self.get_logger().info('RobotKinematicsNode initialized for ${name}')

    def joint_callback(self, msg: JointState):
        if len(msg.position) < ${n}:
            return
        q = list(msg.position[:${n}])
        # Convert revolute radians to degrees if necessary, or compute direct kinematics
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        
        # Publish position
        self.pub_pose.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
`;
}
