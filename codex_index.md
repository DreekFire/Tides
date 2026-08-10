# Codex index

## Multi-script FtD acceptance tests

- `tests/build_tests.py` — concatenates the child scripts into the paste-ready `tests/combined.lua`.
- `tests/build_module_tests.py` — pastes `tests/module_*.lua` into
  `ModuleExample.lua` and generates the integration chunk.
- `tests/module_header.lua`, `tests/module_footer.lua` — FtD mock and
  assertions for the actual ModuleExample manager, including reader/writer
  registries, every override path, conflict modes, exceptions, and ALE globals.
- `tests/module_my.lua`, `tests/module_thy.lua` — ordinary child scripts with
  identical local names, isolated globals, and shared propulsion, missile,
  turret, subobject, weapon, and component endpoint access.
- `tests/run_tests.py` — rebuilds and runs the standalone chunk plus the module
  mode/exception matrix through `lupa`; `--ale` selects Lua 5.1 for fenv tests.
- `tests/combined_header.lua` — local FtD API test double and ROS-style reader/writer tracker.
- `tests/turret_aim.lua`, `tests/turret_fire.lua`, `tests/subobject_aim.lua`, `tests/subobject_fire.lua`, `tests/hull_weapon.lua` — isolated scripts exercising shared turret, ordinary subobject, and hull weapon endpoints.
- `tests/component_writer.lua`, `tests/component_reader.lua` — isolated scripts exercising a shared component endpoint.
- `tests/combined_footer.lua` — cross-script assertions and `_ALE`-gated global-context assertions.
- `ModuleExample.lua` — example module wrappers; each wrapper predeclares
  `Update`, and the real build script must insert that local before each child.

### Reusable verification workflow

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r tests/requirements.txt
tests/run_tests.sh
```

Use `tests/run_tests.sh --ale` when the local runtime provides the ALE
`setfenv`/`getfenv` extensions and `_ALE` should be defined.

## NdArray iterator layers

- `Low/NdArray.lua` — minimal scalar `NdArray.nditer`, direct-axis
  `NdArray.coreIter`, and single-output `NdArray.gufunc`. Operands use the
  generic `{shape, stride, data, offset}` representation; user-facing axis
  indices are zero-based and `-1` in `nditer` `opAxes` means newaxis.
  `NdArray._strideIter` is the lowest-level generic offset-only traversal:
  it advances reusable zero-based offsets without reading operand data.
  Its explicit `"K"` order sorts logical axes by the largest nonzero absolute
  stride across operands, keeping large-stride axes outermost and the
  smallest-stride axis innermost; `"C"`/default and `"F"` retain fixed orders.
  Passing its internal `withIndices` flag makes the same walker additionally
  return reusable zero-based coordinates for scalar/core/gufunc layers.
  `_strideIter1/2/3` remain separate constant-operand-count hot paths with
  scalar offset locals and reusable coordinate tables.
- `tests/ndarray/ndarray_core_iterator_test.lua` — broadcasting, complete
  `opAxes`, reusable core views, exact gufunc core dimensions, output-axis
  placement, and direct generic/specialized offset traversal.
- `tests/run_ndarray_core_iterator_tests.py` — reproducible Lua/lupa test
  runner; run it with `.venv/bin/python tests/run_ndarray_core_iterator_tests.py`.

The layers are intentionally one-way: scalar iteration does not expose core
views; `coreIter` takes positional core-axis lists and broadcasts only loop
axes; `gufunc` parses a signature, validates named core sizes, and delegates
loop traversal to the same normalized representation. The current scalar
layer is read-only and returns reused index/value tables.

### NdArray verification workflow

```sh
.venv/bin/python tests/run_ndarray_core_iterator_tests.py
.venv/bin/python Util/concat.py
```

Use `order = "K"` with `NdArray.nditer`, `coreIter`, or the generic
`_strideIter` to opt into stride-aware traversal. Axes are ranked once when
the iterator is created by the largest nonzero absolute stride across all
operands; large-stride axes are outer and the smallest-stride axis is inner.

The focused math/indexing workflow is:

```sh
./.venv/bin/python tests/run_ndarray_math_tests.py
./.venv/bin/python tests/run_ndarray_core_iterator_tests.py
./.venv/bin/python tests/run_linalg_ndarray_tests.py
./.venv/bin/python tests/run_tests.py
```

### NdArray math and linear algebra

- `Low/NdArray.lua` — the original WIP public implementations for
  constructors, copy/view operations, broadcasting elementwise arithmetic,
  reductions, and NumPy-like `matMul` are retained; iterator-compatible fixes
  are made in those definitions. The later unique APIs include
  `coreIter`, `gufunc`, concatenate/stack/expandDims/squeeze/diag, and
  arg-reductions. `nonzero` and `argwhere` use explicit result positions in
  their per-element loops, while named comparison functions call the
  elementwise implementation directly. `reshape` delegates view eligibility
  to `_reshapeIsView`, including compatible non-contiguous stride blocks;
  incompatible broadcasts report the operand shapes in their formatted error
  messages. `get` and `set` now share one normalized plan: integer/slice/
  ellipsis/newaxis indexing returns writable views, while broadcasted fancy
  integer indices and boolean masks materialize or update through one scalar
  offset path. Separated advanced-index dimensions move to the front, and
  multidimensional boolean masks consume matching leading source axes as
  paired nonzero coordinates. Explicitly typed empty masks are supported;
  Lua `true`/`false` indices add length-one/zero selection dimensions. Scalar
  copy/where/nonzero operations use explicit Lua truth semantics for boolean
  arrays. Arrays expose inferred or explicit `dtype` metadata (`any`, `bool`,
  `int`, `float`, or `object`); metadata does not implicitly cast values.
  Rank-zero arrays remain tables in Lua and expose `NdArray.item`/`scalar` for
  explicit extraction, while rank-zero integer arrays are accepted as scalar
  indices. Array arithmetic metatables cover `+`, `-`, `*`, `/`, and unary `-`,
  including scalar operands; matrix multiplication remains the named
  `NdArray.matMul` operation because Lua has no `@` operator. Comparison
  operators intentionally error and direct callers to `NdArray.equal`,
  `notEqual`, `less`, `lessEqual`, `greater`, or `greaterEqual`. Lua has no
  truth-value metamethod, so `if arr` cannot be intercepted; use `NdArray.any`,
  `NdArray.all`, or the explicit `NdArray.truthValue` guard. The guard follows
  NumPy's primary scalar rule (one element returns a boolean; empty and
  multi-element arrays raise an ambiguous-truth error). Assignments whose
  source shares the destination backing table emit a warning; overlapping-copy
  semantics remain intentionally unspecified.
- `Low/Linalg.lua` — NdArray-based partial-pivoted LU, Householder QR,
  Cholesky, vector/matrix solves, full-column-rank least squares, determinant,
  inverse, damped least-squares pseudoinverse, normalization, cross/outer/skew
  products, rigid-transform composition/inversion, point transforms, power
  iteration, and symmetric QR eigendecomposition.
- `tests/ndarray/ndarray_math_test.lua` and
  `tests/linalg/linalg_ndarray_test.lua` — numerical reconstruction,
  broadcasting, simple/fancy/boolean indexing, writable views, copy/where,
  sorting, reductions, solve, robotics vector/transform helpers, and
  eigendecomposition checks.

### Reusable math verification workflow

```sh
.venv/bin/python tests/run_ndarray_math_tests.py
.venv/bin/python tests/run_linalg_ndarray_tests.py
.venv/bin/python tests/run_ndarray_core_iterator_tests.py
.venv/bin/python Util/concat.py
```

The focused math runners load `Low/NdArray.lua` or `Low/Linalg.lua` directly;
the generated paste-ready artifact is `Tides.lua` and should be regenerated
after source changes. The complete repository workflow is:

```sh
.venv/bin/python tests/run_tests.py
```

To check that a public method has only one retained definition:

```sh
rg -n '^function NdArray\.|^NdArray\.[A-Za-z0-9_]+\s*=' Low/NdArray.lua
```

To scan for removed traversal symbols after a refactor:

```sh
rg -n 'nditerPacked|_nditer|ndTraversal|ndComparison|shallowCopyWithMeta' Low tests --glob '*.lua'
```
