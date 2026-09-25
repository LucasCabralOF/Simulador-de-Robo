# Implementation Plan

## Objective
Complete the simulator's missing lecture topics and validate the numerical and visual behavior.

## Steps
1. Extract standard DH kinematics, orientation, geometric Jacobian, SVD diagnostics, and bounded numerical inverse solvers into a shared module.
2. Add unit-labelled controls, joint limits and validation, matrix/orientation views, differential and inverse workflows.
3. Make API configuration portable and development startup run both services; validate stored models.
4. Verify analytical examples, finite-difference Jacobians, inverse residuals, persistence, lint/build, and desktop/mobile WebGL interaction.
5. Document conventions, limitations, and validation outcomes in the project memory and README.

## Result
Completed all five steps. Validated with 14 numerical/API tests, four desktop/mobile browser tests against the production build, clean lint, and a clean build. Inspected screenshots and verified nonblank, correctly framed, interactive WebGL output. The existing SQLite database was not used for test writes.

## Current Fix: Shoulder Motor Placement
1. Reproduce the user's RRPRR model with a1=0 and d2=2 in an isolated browser test.
2. Position non-base revolute motor bodies at their axial d offset, before the a translation and alpha twist; keep the base motor anchored.
3. Verify pixels at the expected shoulder location and absence of a motor at the old location, including negative offsets and joint movement on desktop/mobile.
4. Run numerical/API tests, lint, build and browser tests; document this visual convention without changing DH or stored models.

Fix completed. Inspected desktop/mobile screenshots and confirmed actual motor pixels at the expected shoulder position, no motor pixels at the old origin, and an anchored base. All 20 tests passed; lint and build are clean.
