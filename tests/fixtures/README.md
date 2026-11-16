# Golden test data

This directory contains version-controlled input fixtures and expected reference results.

## Rules

1. Input fixtures are immutable test inputs. A test must never regenerate or edit them during execution.
2. Expected files are reviewed reference data. Never update them automatically because a test failed.
3. Floating-point comparisons use the tolerance declared by the expected file or test case. Topology, IDs, counts, state transitions and JSON schema fields use exact comparisons.
4. Every implementation phase must add at least one positive case, one boundary/degenerate case and one negative/error case where that class of failure is meaningful.
5. Each implemented phase must expose a named test group in `SkinPhysicsTests`. A fixture existing on disk does not count as a passing test.
6. Golden changes require a reason, a numerical derivation or independent reference, and a test review in the same commit.
7. Performance fixtures define workload size and correctness invariants; they do not define machine-independent FPS thresholds.

## Phase status

- Phase 01 Importer: persistent OBJ and glTF fixtures are exercised by automated tests.
- Phase 02 Physics Mesh / Skin Weight / Cache: persistent MSH fixtures are exercised by automated tests. Additional reference JSON documents derived values.
- Phase 03–11: canonical input and expected-result fixtures are prepared. The corresponding runtime tests are added alongside each implementation phase. Their presence is marked `fixture_ready`, not `implemented`.

The mapping from phase/test ID to input and expected result is in `manifest.json`.