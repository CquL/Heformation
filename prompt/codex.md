# AGENTS.md

## Core Principle

Solve the requested problem with the **smallest reasonable change**.

Do not optimize for apparent engineering completeness. Optimize for:

1. Correctness
2. Simplicity
3. Clear data flow
4. Maintainability
5. Performance when relevant
6. Only the validation necessary for the current change

Prefer less code, fewer files, fewer abstractions, fewer checks, and shorter call chains.

---

## 1. Scope

Only implement what the current task requires.

Do not proactively add:

* unrelated refactors
* generalized frameworks
* future-proof abstractions
* compatibility layers
* plugin systems
* registries
* factories
* managers
* service layers
* wrappers around wrappers
* new configuration systems
* metadata systems
* manifests
* extra CLI layers
* additional infrastructure

Do not expand the task because something "could be improved."

If a separate improvement is not necessary for the requested functionality, leave it alone.

---

## 2. Minimal Change Policy

Always prefer the smallest effective patch.

General rule:

* If 10 lines are enough, do not write 100.
* If one file is enough, do not modify five.
* If an existing function can be extended cleanly, do not create a new abstraction.
* If a function is enough, do not create a class.
* If existing code already provides the capability, reuse it.
* If code can be removed instead of adding another compatibility layer, remove it.

Do not create parallel implementations of functionality that already exists.

---

## 3. No Over-Engineering

Do not introduce abstractions merely for architectural cleanliness or hypothetical future reuse.

Avoid unnecessary:

* `Manager`
* `Service`
* `Factory`
* `Builder`
* `Registry`
* `Adapter`
* `Provider`
* `Wrapper`
* `Base*`
* interface hierarchies
* dependency injection layers
* schema layers
* generic pipelines

Before adding an abstraction, ask:

> Does this abstraction solve a concrete problem that exists now?

If not, do not add it.

A direct implementation is preferred over a theoretically elegant multi-layer design.

Avoid call chains such as:

```text
entrypoint
→ manager
→ service
→ adapter
→ wrapper
→ implementation
```

Core logic should be easy to locate and trace.

---

## 4. Do Not Over-Standardize

Do not standardize everything simply because multiple pieces of code look similar.

Do not introduce common frameworks unless they materially simplify the current code.

Avoid converting simple project code into enterprise-style architecture.

For research and experimental code, prefer:

* explicit logic
* direct function calls
* transparent data flow
* easy debugging
* easy modification

over architectural formality.

---

## 5. Testing Policy

Do **not** automatically create or run large test suites for every modification.

Only perform tests that are directly useful for validating the current change.

Do not habitually add:

* smoke tests
* sanity tests
* full integration tests
* end-to-end tests
* benchmark tests
* duplicate unit tests
* synthetic tests
* dry-run frameworks

unless they are actually necessary.

For simple changes, a focused execution or targeted test is sufficient.

Do not run a full project test suite when a small targeted check adequately validates the change.

Do not create tests solely to increase apparent test coverage.

Do not test unchanged functionality unless it is directly affected by the modification.

---

## 6. No Unnecessary Hash or Integrity Checks

Do not calculate or verify hashes unless the task specifically involves reproducibility, file integrity, cache invalidation, or version identity.

Do not routinely generate or check:

* Git SHA
* file hashes
* checkpoint hashes
* dataset hashes
* config hashes
* evaluator hashes
* manifest hashes

Do not create manifests or metadata files simply to record information that is not needed for the task.

Do not repeatedly verify facts that are already known.

---

## 7. Avoid Excessive Defensive Programming

Do not add defensive code for every theoretically possible failure.

Avoid unnecessary:

```python
try:
    ...
except Exception:
    ...
```

Avoid broad exception handling, silent fallback, and excessive default values.

If a state represents a genuine programming or configuration error, fail clearly instead of hiding it.

Prefer:

```python
assert condition
```

or a clear exception when appropriate.

Do not make invalid states silently usable.

Do not add fallback behavior merely to keep execution running.

---

## 8. Do Not Hide Problems

Never "fix" a problem by suppressing it.

Do not:

* swallow exceptions
* return empty outputs after failures
* silently substitute default files
* silently switch algorithms
* silently switch checkpoints
* silently skip missing inputs
* automatically downgrade behavior
* convert errors into warnings without justification

Fix the underlying problem when practical.

---

## 9. Do Not Modify Tests to Manufacture Success

Do not make tests pass by weakening them.

Unless explicitly required, do not:

* reduce assertions
* increase tolerances
* skip failing tests
* remove test cases
* modify expected outputs
* add mocks around the functionality being tested
* catch exceptions only to make tests succeed

Tests should validate the implementation, not accommodate a broken implementation.

---

## 10. Prefer Existing Structure

Before creating a new file, class, helper, configuration entry, or module, inspect the existing implementation.

Prefer extending or simplifying existing code.

Do not create:

```text
foo.py
foo_new.py
foo_v2.py
foo_final.py
foo_refactored.py
```

unless parallel implementations are explicitly required.

When replacing an obsolete implementation, remove it if safe instead of keeping multiple versions indefinitely.

---

## 11. Simplify Existing Complexity

If the surrounding code is already overly layered, do not add another layer on top.

Prefer simplifying the relevant section.

Look for opportunities to remove:

* redundant wrappers
* duplicate conversions
* duplicate helper functions
* obsolete compatibility code
* unnecessary indirection
* dead code
* repeated configuration
* redundant intermediate representations

Do not preserve complexity merely because it already exists.

---

## 12. Performance

When the task concerns performance, investigate actual computational costs.

Prioritize:

* unnecessary repeated computation
* redundant I/O
* unnecessary serialization
* unnecessary tensor copies
* CPU/GPU transfers
* repeated model loading
* inefficient loops
* duplicated preprocessing
* unnecessary temporary files
* avoidable synchronization
* unnecessary sequential execution

Do not respond to performance problems by adding more abstractions or configuration layers.

---

## 13. Comments and Documentation

Do not add verbose comments explaining obvious code.

Comments should explain:

* non-obvious reasoning
* important constraints
* unusual algorithmic decisions

Do not document every line.

Do not create extensive documentation for a small implementation change unless requested.

---

## 14. Dependencies

Do not add or upgrade dependencies unless clearly necessary.

Do not run broad dependency upgrades as a troubleshooting strategy.

Avoid introducing a package when a small amount of existing code can solve the problem cleanly.

Do not modify the environment unless the task requires it.

---

## 15. Refactoring

Do not perform broad refactors while implementing a small feature or bug fix.

Separate necessary implementation changes from optional architectural cleanup.

Local simplification is encouraged when it directly improves the modified code.

Repository-wide redesign is not.

---

## 16. Validation Standard

Validation should be proportional to the risk and scope of the modification.

Examples:

### Small deterministic function change

Use a focused invocation or targeted test.

### CLI argument change

Run the relevant command or parser check.

### Training/inference logic change

Run the smallest representative execution needed to verify the changed path.

### Formatting/documentation change

No runtime testing is necessary.

Do not mechanically run smoke tests, full pipelines, or exhaustive validation after every change.

---

## 17. Before Adding Code

Before adding any significant code, ask:

1. Is this necessary for the current task?
2. Can existing code already do this?
3. Can this be implemented more directly?
4. Am I introducing a layer that provides little real value?
5. Will this make the main execution path harder to understand?
6. Can I solve the problem by simplifying existing code instead?

If the new code is not clearly justified, do not add it.

---

## 18. Before Finishing

Review the modified code and remove unnecessary complexity introduced during implementation.

Specifically check for:

* unnecessary files
* unnecessary classes
* unnecessary helpers
* duplicate logic
* redundant wrappers
* excessive configuration
* dead code
* needless compatibility branches
* redundant tests
* unnecessary validation
* unnecessary logging
* unnecessary metadata

Prefer finishing with less code than an initially obvious implementation would require.

---

## 19. Final Response

Keep completion reports concise.

Report only:

* what changed
* important design decisions
* what was actually validated
* any concrete unresolved issue

Do not provide long lists of routine checks.

Do not report trivial actions such as:

* checking every hash
* verifying every file exists
* running redundant smoke tests
* listing unchanged components
* describing generic engineering best practices

unless they materially affected the task.

---

## Default Decision Rule

When two implementations are both correct, choose the one with:

* fewer lines of code
* fewer files
* fewer abstractions
* fewer dependencies
* fewer configuration options
* shorter call chains
* clearer data flow
* easier debugging

Do not optimize for theoretical extensibility that has not been requested.

**The objective is to solve the current problem cleanly, not to build a framework around it.**
