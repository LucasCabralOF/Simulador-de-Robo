/**
 * Shared JSDoc domain typedefs for the DH Robot Simulator.
 * Zero runtime overhead: imported solely via JSDoc `import('./types.js').*` annotations.
 */

/** @typedef {[number, number, number]} Vec3 */
/** @typedef {[number, number, number]} RpyDeg */
/** @typedef {'xy' | 'position' | 'pose'} TaskMode */
/** @typedef {'gripper' | 'vacuum' | 'welder'} ToolType */
/** @typedef {'cylinder' | 'square' | 'hex' | 'flat'} LinkShape */

/**
 * @typedef {Object} Joint
 * @property {number} id - Unique joint identifier
 * @property {'R' | 'P'} type - Joint actuation type (revolute or prismatic)
 * @property {number} a - DH link length along X axis
 * @property {number} alpha - DH link twist around X axis (degrees)
 * @property {number} d - DH link offset along Z axis
 * @property {number} theta - DH joint angle offset (degrees)
 * @property {number} val - Active joint variable value (deg for R, u for P)
 * @property {number} min - Minimum joint travel limit
 * @property {number} max - Maximum joint travel limit
 * @property {number} speed - Instantaneous joint rate (deg/s or u/s)
 */

/**
 * @typedef {Object} CartesianTarget
 * @property {Vec3} position - Target Cartesian coordinates [x, y, z]
 * @property {RpyDeg} rpy - Target ZYX Euler angles [roll, pitch, yaw] in degrees
 */

/**
 * @typedef {Object} JointStyleOverride
 * @property {LinkShape} [linkShape] - Cross-section shape override for this link
 * @property {number} [linkRadius] - Cross-section half-width/radius override
 * @property {number} [revRadius] - Revolute actuator radius override
 * @property {number} [revLength] - Revolute actuator length override
 * @property {number} [prismWidth] - Prismatic housing width override
 * @property {number} [prismLength] - Prismatic housing length override
 * @property {string} [jointColor] - Custom hex color for this joint actuator
 * @property {string} [linkColor] - Custom hex color for this link shaft
 */

/**
 * @typedef {Object} ModelStyle
 * @property {LinkShape} linkShape - Global cross-section shape for links
 * @property {number} linkRadius - Global link shaft radius
 * @property {number} revRadius - Global revolute motor radius
 * @property {number} revLength - Global revolute motor height
 * @property {number} prismWidth - Global prismatic block width
 * @property {number} prismLength - Global prismatic sleeve length
 * @property {string} [revColor] - Global hex color for revolute joints
 * @property {string} [prismColor] - Global hex color for prismatic joints
 * @property {string} [linkColor] - Global hex color for link shafts
 * @property {Record<string, JointStyleOverride>} [perJoint] - Per-joint geometry/color overrides keyed by joint ID
 */

/**
 * @typedef {Object} ObstacleConfig
 * @property {boolean} enabled - Whether the 3D obstacle collision check is active
 * @property {'sphere' | 'box'} shape - Geometric collider primitive
 * @property {Vec3} position - World center position [x, y, z]
 * @property {number} radius - Sphere radius or half-extent for boxes
 */

/**
 * @typedef {Object} EnvironmentConfig
 * @property {boolean} floorEnabled - Whether the workbench floor constraint is active
 * @property {number} floorZ - Height Z of the workbench plane
 * @property {ObstacleConfig} obstacle - Configurable 3D obstacle settings
 */

/**
 * @typedef {Object} LinkMeshEntry
 * @property {string} fileName - Basename of the attached STL/mesh file
 * @property {number} [scale] - Uniform scale factor applied to the mesh
 * @property {string} [uri] - Original package/file URI from parsed URDF
 */

/**
 * @typedef {Object} ToolConfig
 * @property {ToolType} [toolType] - Selected end-effector tool type
 * @property {RpyDeg} [toolOffsetRpy] - Mount orientation offset [roll, pitch, yaw] in degrees
 * @property {number} [toolGrip] - Tool actuation percentage [0..100]
 * @property {number} [toolScale] - Uniform 3D scale multiplier for the tool
 * @property {ModelStyle} [modelStyle] - Active robot visual/geometry style config
 * @property {Record<string, LinkMeshEntry>} [linkMeshes] - Per-link STL mesh metadata keyed by joint ID
 * @property {string} [toolMeshFileName] - Attached custom STL filename for tool0
 */

/**
 * @typedef {Object} CollisionInfo
 * @property {boolean} colliding - True when a physical self or environment collision occurs
 * @property {string} [rule] - Identifier of the triggered collision rule
 * @property {Vec3} [point] - Estimated 3D contact coordinate
 * @property {string} [message] - Human-readable localized collision description
 */

/**
 * @typedef {Object} ForwardKinematicsResult
 * @property {import('three').Matrix4} total - End-effector homogeneous matrix T^0_n
 * @property {import('three').Matrix4[]} transforms - Cumulative frame matrices [T_0, T^0_1, ..., T^0_n]
 * @property {import('three').Matrix4[]} links - Individual DH link matrices [A_1, ..., A_n]
 * @property {Vec3[]} positions - Cartesian origins [O_0, O_1, ..., O_n]
 * @property {number[][]} jacobian - 6xN geometric Jacobian matrix
 * @property {{ matrix: number[][], rpy: RpyDeg, gimbalLock: boolean, angle: number, axis: Vec3 | null, euler: number[] } | null} orientation - Decomposed rotation metrics
 */

/**
 * @typedef {'auto' | { shoulder?: string, elbow?: string, wrist?: string }} PosturePreference
 */

/**
 * @typedef {Object} PostureBranch
 * @property {string} id - Deterministic posture branch key
 * @property {Joint[]} joints - Solved joint array for this branch
 * @property {{ shoulder?: string, elbow?: string, wrist?: string }} posture - Branch classification flags
 * @property {'scara' | 'puma560' | 'ur5'} architecture - Detected analytical robot family
 * @property {string} label - Localized branch display label
 * @property {number} error - Task residual norm
 * @property {number[]} residual - 6-element pose error vector [dx,dy,dz,rx,ry,rz]
 * @property {boolean} converged - Whether branch error meets tolerance
 * @property {number} iterations - Iteration count (0 for exact analytical)
 * @property {'analytical' | 'dls'} solver - Solver method identifier
 * @property {number} distance - Joint-space distance from seed configuration
 * @property {boolean} [orientationAdjusted] - True when orientation was relaxed in position mode
 */

/**
 * @typedef {Object} InverseKinematicsResult
 * @property {Joint[]} joints - Resulting joint configuration
 * @property {number} error - Final task residual norm
 * @property {number[]} residual - Final 6-element pose error vector
 * @property {boolean} converged - True if convergence tolerance and posture rules succeeded
 * @property {number} iterations - Numerical DLS iterations executed (0 for analytical)
 * @property {'analytical' | 'dls'} [solver] - Solver strategy used
 * @property {string | null} [architecture] - Detected closed-form architecture if applicable
 * @property {Record<string, string> | null} [posture] - Classified posture flags of solution
 * @property {PostureBranch[]} [branches] - Alternative collision-free branches discovered
 * @property {boolean} [postureUnavailable] - True when strict requested posture cannot reach safely
 */

/**
 * @typedef {Object} Waypoint
 * @property {number} id - Unique waypoint ID
 * @property {string} name - User-visible waypoint label
 * @property {number[]} q - Joint vector values matching robot DOFs
 * @property {number} duration - Transition duration from previous waypoint in seconds
 */

/**
 * @typedef {Object} TrajectoryData
 * @property {Vec3[]} pathPoints - Sampled 3D Cartesian curve points
 * @property {Vec3[]} waypointCoords - Exact Cartesian coordinates of each waypoint
 * @property {number} totalDuration - Total trajectory playback duration in seconds
 * @property {number[]} times - Timestamp array across samples
 * @property {number[]} waypointTimes - Exact cumulative timestamp per waypoint
 * @property {{ q: number[], v: number[], a: number[] }[]} jointProfiles - Per-joint q, v, a time series
 * @property {number[]} tcpSpeeds - Cartesian TCP linear speed |v(t)| across samples
 * @property {{ type: string, maxVelRatio: number, accelPct: number, cruisePct: number } | null} profileInfo - Velocity profile summary metadata
 */

/**
 * @typedef {Object} WorkspaceCloud
 * @property {Float32Array} positions - Packed XYZ coordinates (count * 3)
 * @property {Float32Array} colors - Packed RGB manipulability heat colors (count * 3)
 * @property {number} count - Total valid sampled points
 * @property {{ minX: number, maxX: number, minY: number, maxY: number, minZ: number, maxZ: number, dx: number, dy: number, dz: number }} bounds - Cartesian bounding box extents
 * @property {number} boundingVolume - Estimated bounding box volume
 * @property {number} maxManipulability - Peak Yoshikawa manipulability index observed
 */

/**
 * @typedef {Object} SingularityInterval
 * @property {number} start - Interval start coordinate along axis
 * @property {number} end - Interval end coordinate along axis
 * @property {'low' | 'valley' | 'deadzone' | 'boundary'} kind - Singularity classification
 * @property {string} label - Formatted interval description
 */

/**
 * @typedef {Object} SingularityAxisDetail
 * @property {number} axis - Axis index (0=X, 1=Y, 2=Z)
 * @property {string} label - Axis letter ('X', 'Y', or 'Z')
 * @property {number} min - Reachable minimum on axis
 * @property {number} max - Reachable maximum on axis
 * @property {number} span - Reachable span length
 * @property {number} currentCoord - Reference coordinate on this axis
 * @property {SingularityInterval[]} intervals - Critical low-dexterity or dead-zone intervals
 * @property {Array<{ coord: number, wNorm: number | null, deadZone: boolean }>} profile - Sampled normalized manipulability bins
 */

/**
 * @typedef {Object} SingularityJointSweep
 * @property {number} jointIndex - Zero-based joint index
 * @property {number} id - Joint ID
 * @property {'R' | 'P'} type - Joint type
 * @property {string} unit - Joint display unit ('°' or 'u')
 * @property {number} min - Lower joint limit
 * @property {number} max - Upper joint limit
 * @property {number} currentVal - Current joint value
 * @property {Array<{ val: number, w: number, wPct: number, qVec: number[], pos: Vec3 }>} criticalValues - Discovered singular/boundary joint positions
 */

/**
 * @typedef {Object} SingularityCriticalPose
 * @property {string} id - Unique pose identifier
 * @property {string} category - Critical zone classification label
 * @property {string} label - Descriptive label for UI lists
 * @property {Vec3} pos - 3D Cartesian TCP location
 * @property {number[]} q - Joint configuration vector achieving this pose
 * @property {number} wPct - Normalized dexterity percentage [0..100]
 */

/**
 * @typedef {Object} SingularityReport
 * @property {Vec3} referencePos - Reference 3D coordinate used for X/Y/Z slices
 * @property {number} globalMaxW - Global reference maximum manipulability
 * @property {number} currentW - Current configuration manipulability index
 * @property {number} currentDexterityPct - Current manipulability relative percentage [0..100]
 * @property {SingularityAxisDetail[]} axes - Per-axis 1D slice singularity profiles
 * @property {SingularityJointSweep[]} jointSweeps - Per-joint 25-step sweep critical points
 * @property {SingularityCriticalPose[]} criticalPoses - Spatially diverse 3D low-dexterity poses
 */

/**
 * @typedef {Object} SimulationSnapshot
 * @property {Joint[]} joints - Captured DH joint definitions and values
 * @property {ToolType} toolType - Active tool type
 * @property {RpyDeg} toolOffsetRpy - Tool mount RPY offset
 * @property {number} toolGrip - Tool actuation value
 * @property {number} toolScale - Tool scale factor
 * @property {ModelStyle} modelStyle - Robot geometry and color styling
 * @property {EnvironmentConfig} environment - Floor and obstacle settings
 * @property {Waypoint[]} waypoints - Saved trajectory waypoints
 */

export {};
