# Source Snapshots

Original repositories are preserved under `vendor/`; integration edits are made
only in a newly created copy. Sources were inspected and pinned, not silently
updated to a moving branch on startup.

| Repository | Branch / commit | Original ZIP SHA256 |
|---|---|---|
| sara-eng963/GPRobotControl_Simulation | sara/can-replacement / 15aa22efc071554fe06f1e42e1007633dee15cc9 | D638F90D388951A12AFD81526A170D0B4F1DAF54C3BE5BEA49AB9419855B346E |
| sara-eng963/AdmittanceControl_GP | main / 589df87e0b36dfb707394ef959a6441456e408e6 | ED60B7BA11834EDE1E2112779CE70C744029E8D991337B6E6D50C7BEDAB6CFDE |

Robot geometry and mass properties are copied from the corrected robot_ver2
package in the existing project. Explicit rebuilding imports this packaged URDF
afresh and exports C geometry from the same source. The saved simulation checks
its URDF hash before starting. Runtime collision certificates are recreated for
each run and include STL-content hashes; no old project collision cache is used.

`docs/` contains previews of the actual delivered dashboard and model diagram.
`tests/verified_results.mat` and `tests/verified_relocation.mat` preserve executed
MATLAB verification summaries. Paths inside these records are test provenance,
not runtime dependencies. See VERIFICATION.md for coverage and exclusions.
