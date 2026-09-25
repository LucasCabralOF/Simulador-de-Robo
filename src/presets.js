export const PRESETS = {
  rpr: {
    id: 'rpr',
    name: 'Didático RPR (3-DOF)',
    description: 'Manipulador didático planar das aulas com 2 juntas rotativas e 1 prismática.',
    dof: 3,
    joints: [
      { id: 1, type: 'R', a: 0, alpha: 90, d: 0, theta: 0, val: 45, min: -180, max: 180, speed: 0 },
      { id: 2, type: 'P', a: 0, alpha: -90, d: 0, theta: 0, val: 5, min: 0, max: 10, speed: 0 },
      { id: 3, type: 'R', a: 5, alpha: 0, d: 0, theta: 0, val: 30, min: -180, max: 180, speed: 0 },
    ],
  },
  scara: {
    id: 'scara',
    name: 'SCARA (RRPR)',
    description: 'Robô de montagem industrial e pick-and-place de alta velocidade com 4 eixos.',
    dof: 4,
    joints: [
      { id: 1, type: 'R', a: 3.5, alpha: 0, d: 2, theta: 0, val: 25, min: -150, max: 150, speed: 0 },
      { id: 2, type: 'R', a: 2.5, alpha: 180, d: 0, theta: 0, val: 35, min: -150, max: 150, speed: 0 },
      { id: 3, type: 'P', a: 0, alpha: 0, d: 0, theta: 0, val: 1.8, min: 0, max: 4, speed: 0 },
      { id: 4, type: 'R', a: 0, alpha: 0, d: 0, theta: 0, val: 0, min: -180, max: 180, speed: 0 },
    ],
  },
  puma560: {
    id: 'puma560',
    name: 'PUMA 560 (6R)',
    description: 'Manipulador antropomórfico clássico de 6 graus de liberdade com punho esférico desacoplado.',
    dof: 6,
    joints: [
      { id: 1, type: 'R', a: 0, alpha: 90, d: 3, theta: 0, val: 0, min: -160, max: 160, speed: 0 },
      { id: 2, type: 'R', a: 3.5, alpha: 0, d: 0, theta: 0, val: -30, min: -225, max: 45, speed: 0 },
      { id: 3, type: 'R', a: 0.5, alpha: -90, d: 1, theta: 0, val: 45, min: -45, max: 225, speed: 0 },
      { id: 4, type: 'R', a: 0, alpha: 90, d: 3.5, theta: 0, val: 0, min: -110, max: 170, speed: 0 },
      { id: 5, type: 'R', a: 0, alpha: -90, d: 0, theta: 0, val: 30, min: -100, max: 100, speed: 0 },
      { id: 6, type: 'R', a: 0, alpha: 0, d: 1, theta: 0, val: 0, min: -266, max: 266, speed: 0 },
    ],
  },
  stanford: {
    id: 'stanford',
    name: 'Stanford Arm (RRPRRR)',
    description: 'Braço robótico esférico com junta prismática intermediária e punho de 3 rotações.',
    dof: 6,
    joints: [
      { id: 1, type: 'R', a: 0, alpha: -90, d: 2.5, theta: 0, val: 0, min: -160, max: 160, speed: 0 },
      { id: 2, type: 'R', a: 0, alpha: 90, d: 1.5, theta: 0, val: 30, min: -120, max: 120, speed: 0 },
      { id: 3, type: 'P', a: 0, alpha: 0, d: 0, theta: 0, val: 3, min: 1, max: 6, speed: 0 },
      { id: 4, type: 'R', a: 0, alpha: -90, d: 0, theta: 0, val: 0, min: -160, max: 160, speed: 0 },
      { id: 5, type: 'R', a: 0, alpha: 90, d: 0, theta: 0, val: 45, min: -90, max: 90, speed: 0 },
      { id: 6, type: 'R', a: 0, alpha: 0, d: 1, theta: 0, val: 0, min: -180, max: 180, speed: 0 },
    ],
  },
  ur5: {
    id: 'ur5',
    name: 'Universal Cobot UR5 (6R)',
    description: 'Cobot colaborativo moderno com 6 juntas rotativas e arquitetura antropomórfica.',
    dof: 6,
    joints: [
      { id: 1, type: 'R', a: 0, alpha: 90, d: 2.5, theta: 0, val: 0, min: -180, max: 180, speed: 0 },
      { id: 2, type: 'R', a: -3.5, alpha: 0, d: 0, theta: 0, val: -45, min: -180, max: 180, speed: 0 },
      { id: 3, type: 'R', a: -3.0, alpha: 0, d: 0, theta: 0, val: 45, min: -180, max: 180, speed: 0 },
      { id: 4, type: 'R', a: 0, alpha: 90, d: 1.5, theta: 0, val: 0, min: -180, max: 180, speed: 0 },
      { id: 5, type: 'R', a: 0, alpha: -90, d: 1.5, theta: 0, val: 45, min: -180, max: 180, speed: 0 },
      { id: 6, type: 'R', a: 0, alpha: 0, d: 1.0, theta: 0, val: 0, min: -180, max: 180, speed: 0 },
    ],
  },
};

export const PRESET_LIST = Object.values(PRESETS);
