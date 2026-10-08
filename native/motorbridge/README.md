# Optional fresh motor feedback

`fast_feedback.py` uses this Linux ABI extension to read six independent motor
transactions concurrently. Motor commands remain on the existing owner thread.
Each position-register reply and status reply must arrive after its request;
cached status cannot satisfy the native transaction. One missing status reply
gets one fresh request with a 40 ms timeout; a second failure or a batch over
120 ms faults the controller. All worker reads finish before commands resume
or motor handles close. Temperature, speed and motor-status checks remain active.

The patch adds one exported function to motorbridge 0.5.6, commit
`c652ce420e7da1008fa1864cc7f47d7d7c57fdb6`, and two freshness tests. It uses the
upstream Damiao timestamped feedback implementation. It does not change firmware,
motor gains, acceleration limits or the installed Python package.

With Git and Rust 1.90.0 installed, and motorbridge 0.5.6 in the Python environment:

```sh
python3 native/motorbridge/build.py
.venv/bin/python -m unittest tests.test_fast_feedback
```

The build downloads pinned upstream sources, runs the native tests and writes
`build/libmotor_abi.so` plus a checksum manifest. These generated files are ignored
by Git. `--cargo /path/to/cargo` supports an isolated toolchain. The source retains
upstream's license; see the pinned upstream repository.

`AxisArm` automatically selects a matching local build before opening hardware.
A checksum/source mismatch is an error. Without a build, it uses the original
sequential reader. `B601_FRESH_FEEDBACK=0` explicitly selects that original reader
for comparison. Stop the controller with motors disabled before switching builds.

Read-only hardware timing, with the arm supported and disabled:

```sh
.venv/bin/python scripts/arm_response_probe.py --fresh
```

The first 100-read hardware sample measured 5.19 ms median and 5.45 ms maximum
for all six joints, compared with about 45 ms for the original sequential read.
This measures feedback latency, not arm trajectory completion time.

A subsequent full camera/rail/arm trial completed the unchanged open-jaw route
in 114.14 s versus the previous 129.24 s. Median motion-loop interval was
11.68 ms, maximum tracking error 0.981 degrees, and the arm returned with all
motors disabled. A lost status reply during the initial disabled soak led to
the bounded one-retry behavior above; no cached-response fallback was added.
