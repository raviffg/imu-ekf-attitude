# Timestamp ordering, gaps, and duplicate samples

## Project context
MPU6050 six-degree-of-freedom attitude EKF and inertial calibration. This scenario exercises the project's core behavior: gyro-bias states, accelerometer gravity updates, Joseph covariance update, Allan-variance calibration.

## Objective
Define a repeatable check for **timestamp ordering, gaps, and duplicate samples** without assuming unverified hardware behavior or undocumented numerical limits.

## Relevant implementation
- Primary implementation: `src/ekf_6dof.cpp`
- Supporting implementation: `scripts/allan_variance_calib.py`
- Main engineering concerns: timestamp quality, gyro bias drift, accel gating, covariance positivity, Euler singularity, static calibration.

## Procedure
1. Record the source revision, toolchain/runtime versions, and configuration.
2. Prepare a controlled input fixture and preserve its original bytes.
3. Execute the relevant function or command using the documented interface.
4. Capture raw input, output, diagnostics, and run metadata.
5. Repeat with the boundary or fault condition relevant to this scenario.
6. Compare the result with an independently reviewed expectation.

## Expected evidence
- Reproducible command or test harness invocation.
- Input fixture identifier and checksum.
- Observed output and diagnostic log.
- Pass/fail rationale tied to explicit acceptance criteria.

## Acceptance criteria
The behavior is deterministic for identical inputs, invalid data is handled according to the documented contract, and no safety-relevant failure is silently converted into a valid result.

## Open items
- Add measured acceptance limits when bench evidence is available.
- Link a checked-in fixture and automated test when implemented.
- Record unresolved assumptions rather than treating them as verified facts.
