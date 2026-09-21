# AML Python API Reference

The **Algebraic Modeling Library (AML)** is accessed via the `mipsolvers.aml` submodule. It provides a high-level, set-indexed interface for building and solving linear, quadratic, nonlinear, and mixed-integer optimization problems without manually constructing coefficient matrices.

## Quick-Start Example

```python
import mipsolvers
aml = mipsolvers.aml

m = aml.Model("transport")

# Sets
plants    = m.add_set("plants",    ["P1", "P2"])
customers = m.add_set("customers", ["C1", "C2", "C3"])

# Parameters
supply   = m.add_param("supply",   dim=1, unit="units")
demand   = m.add_param("demand",   dim=1, unit="units")
cost     = m.add_param("cost",     dim=2, unit="$/unit")

supply.load({"P1": 100.0, "P2": 150.0})
demand.load({"C1": 80.0, "C2": 90.0, "C3": 70.0})
cost.load({
    ("P1","C1"): 2.0, ("P1","C2"): 3.0, ("P1","C3"): 5.0,
    ("P2","C1"): 4.0, ("P2","C2"): 1.0, ("P2","C3"): 2.0,
})

# Variables
x = m.add_var2("x", plants, customers, aml.VarType.Continuous, lb=0.0)

# Objective
m.minimize(aml.sum_over(plants, lambda i:
    aml.sum_over(customers, lambda j:
        cost[i, j] * x[i, j])))

# Supply constraints
m.add_constraints("supply", plants, lambda i:
    aml.sum_over(customers, lambda j: x[i, j] * 1.0) <= supply[i])

# Demand constraints
m.add_constraints("demand", customers, lambda j:
    aml.sum_over(plants, lambda i: x[i, j] * 1.0) >= demand[j])

result = m.solve()
print(result)                       # <SolveResult: obj=..., OPTIMAL>
print(result.array_values(x))      # {('P1','C1'): ..., ...}
```

## Import

```python
import mipsolvers
aml = mipsolvers.aml
```

All types below live in the `aml` submodule namespace.

---

## Key

A multi-dimensional index. Sets and variables are keyed by `Key` objects.

```python
Key(atoms)                        # from str, tuple[str,...] or list[str]
Key.scalar("P1")                  # 1-D key
Key.pair("P1", "C1")             # 2-D key
Key.make(["a", "b", "c"])        # N-D key
```

**Properties:**
| Property | Type | Description |
|---|---|---|
| `values` | `list[str]` | Underlying atom list |
| `dimension` | `int` | Number of atoms |

`Key` is hashable and supports `==`, `<`, and iteration in dicts/sets.

String shorthand: wherever a `Key` is expected, you can pass a plain `str` (→ scalar key) or a `tuple[str, ...]` / `list[str]` (→ multi-dim key) directly.

---

## Set Types

### Set (abstract base)

```python
s.name          # str
s.dimension     # int – key dimension
s.cardinality   # int – number of elements
s.elements      # list[Key] – all keys in insertion order
s.contains(key) # bool
s.add_element(key) -> bool   # True if newly added
"P1" in s       # __contains__
len(s)          # __len__
for k in s: ... # __iter__
```

### ExplicitSet

User-managed, insertion-ordered set returned by `Model.add_set()`.

### OrderedSet

`ExplicitSet` with position-based navigation, returned by `Model.add_ordered_set()`. Useful for time-period indexing.

```python
s.at(pos)           # Key at 0-based position
s.position(key)     # int – 0-based position; raises if absent
s.prev(key)         # Key | None – previous element
s.next(key)         # Key | None – next element
```

---

## Parameter

Named table of `float` values indexed by `Key`. Created by `Model.add_param()` / `Model.add_param_scalar()`.

```python
p.name        # str
p.dimension   # int
p.unit        # str

# Writing
p.set(key, value)
p.set_scalar(value)        # dim-0 only
p[key] = value             # __setitem__
p.load({"k1": 1.0, ...})   # bulk load from dict

# Reading
p.get(key) -> float        # raises if missing
p.get_scalar() -> float    # dim-0 only
p.get_or(key, default=0.0) -> float
p.contains(key) -> bool
p[key]                     # __getitem__
p.to_dict() -> dict        # all values as {str|tuple: float}
```

---

## Variable Types

### VarType (enum)

| Value | Meaning |
|---|---|
| `VarType.Continuous` | Real-valued |
| `VarType.Integer` | Integer-valued |
| `VarType.Binary` | 0/1 |

### VarRef

Reference to a single scalar variable. Obtained by indexing a `VarArray`.

```python
v.id                         # int – column index in compiled matrix
# Arithmetic (returns LinearExpr)
2.0 * v, v * 2.0
v + w, v - w, -v
v + expr, v - expr
# Comparisons (return TempConstr)
v <= 5.0, v >= 0.0, v == 1.0
```

### VarArray

Indexed family of variables over a set. Returned by `Model.add_var()` / `Model.add_var2()`.

```python
a.name          # str
a.type          # VarType
a.total_count   # int

a["P1"]              # VarRef – 1-D lookup
a[("P1", "C1")]      # VarRef – 2-D lookup (tuple shorthand)
a[key]               # __getitem__ / __call__

a.set_lb(key, lb)    # update lower bound for one variable
a.set_ub(key, ub)    # update upper bound
a.fix(key, value)    # fix to value (lb == ub)
a.lb(key) -> float
a.ub(key) -> float
```

---

## Expression Types

### LinearExpr

Sparse affine expression: $c_0 + \sum_i c_i x_i$.

```python
LinearExpr()                          # zero expression
LinearExpr.from_var(var_id, coef=1.0)
LinearExpr.const_expr(c)             # constant c

expr.constant                         # float (read-write)

# Arithmetic
a + b, a - b, -a
a * s, s * a                          # scalar multiply
a += b, a += c

# Comparisons (return TempConstr)
expr <= 5.0, expr >= 0.0, expr == 1.0
expr <= other_expr, expr >= other_expr
```

### QuadExpr

Quadratic expression: linear part + quadratic terms.

```python
QuadExpr()
QuadExpr.sq(var_id, coef=1.0)            # coef * x_i^2
QuadExpr.bilinear(var_i, var_j, coef=1.0)  # coef * x_i * x_j
QuadExpr.from_linear(lin)               # upgrade LinearExpr

a + b, a - b
a * s, s * a
a += b
```

### NonlinearExpr

Opaque handle into the model's nonlinear expression tree. Created exclusively via `Model.nl_*` factory methods.

```python
e.id          # int – arena node ID
e.is_null()   # True if default-constructed
```

---

## Constraint Types

### TempConstr

Temporary constraint produced by comparison operators on expressions. Pass directly to `Model.add_constraint()`.

```python
x["P1"] + x["P2"] <= supply["P1"]   # produces TempConstr
```

### ConstraintRef

Reference to a single named constraint row.

```python
c.id    # int
c.name  # str
```

### ConstraintArray

Indexed family of constraints returned by `Model.add_constraints()`.

```python
a.name          # str
a.total_count   # int
a["C1"]         # ConstraintRef – lookup by key
a[("i", "j")]
```

---

## SolveOptions

```python
opts = aml.SolveOptions()
opts.solver_name    = ""       # "" = auto, "highs", "gurobi", "native", …
opts.time_limit_sec = 1e30
opts.mip_gap_tol    = 1e-4
opts.verbosity      = 0        # 0=silent, 1=summary, 2=verbose
```

---

## SolveResult

Returned by `Model.solve()`.

```python
res.termination_status  # TerminationStatus enum
res.primal_status       # PrimalStatus enum
res.dual_status         # DualStatus enum
res.is_optimal          # bool
res.has_primal          # bool
res.has_duals           # bool
res.objective_value     # float
res.objective_bound     # float (LP relaxation bound for MIP)
res.optimality_gap      # float
res.solve_time_sec      # float
res.simplex_iterations  # int
res.branch_and_cut_nodes # int
res.solver_used         # str

# Solution access
res.var_value(var_ref)          -> float
res.var_value_by_id(var_id)     -> float
res.array_values(var_array)     -> dict   # {str|tuple: float}

# Dual access (None if unavailable)
res.dual(constraint_ref)            -> float | None
res.dual_array(constraint_array)    -> dict
res.reduced_cost(var_ref)           -> float | None
```

### TerminationStatus (enum)

`Optimal`, `Infeasible`, `Unbounded`, `InfeasibleOrUnbounded`, `TimeLimit`, `IterationLimit`, `NodeLimit`, `ObjectiveLimit`, `NumericalError`, `UserInterrupt`, `SolverError`, `Unknown`

### PrimalStatus / DualStatus (enum)

`Optimal`, `Feasible`, `Infeasible`, `NoSolution`, `Unknown`

---

## Model

```python
m = aml.Model(name="")
m.name            # str
m.num_vars        # int
m.num_constraints # int
```

### Set Construction

```python
m.add_set(name, atoms=[])           -> ExplicitSet
m.add_ordered_set(name, atoms=[])   -> OrderedSet
```

### Parameter Construction

```python
m.add_param(name, dim=1, unit="")   -> Parameter
m.add_param_scalar(name, unit="")   -> Parameter   # dim-0
```

### Variable Construction

```python
m.add_var(name, domain, type, lb=-1e20, ub=1e20)              -> VarArray
m.add_var2(name, domain_a, domain_b, type, lb=-1e20, ub=1e20) -> VarArray
```

### Objective

```python
m.minimize(obj)        # LinearExpr
m.maximize(obj)        # LinearExpr
m.minimize_quad(obj)   # QuadExpr
m.maximize_quad(obj)   # QuadExpr
m.minimize_nl(obj)     # NonlinearExpr
m.maximize_nl(obj)     # NonlinearExpr
```

### Constraints

```python
m.add_constraint(temp_constr, name="")          -> ConstraintRef

m.add_constraints(family, domain, fn)           -> ConstraintArray
# fn: Key -> TempConstr
# e.g.: m.add_constraints("balance", nodes, lambda i: flow[i] == 0)

m.add_nl_constraint(name, expr, sense="==", rhs=0.0) -> ConstraintRef
# sense: "<=", ">=", "==", "le", "ge", "eq"
```

### Nonlinear Expression Builders

All return `NonlinearExpr`. Use these on `Model` instances.

```python
m.nl_var(v)          # VarRef -> NonlinearExpr
m.nl_const(c)        # float  -> NonlinearExpr
m.nl_neg(a)
m.nl_add(a, b), m.nl_sub(a, b)
m.nl_mul(a, b), m.nl_div(a, b)
m.nl_sq(a)           # a^2
m.nl_pow(a, n)       # a^n  (n: NonlinearExpr)
m.nl_sqrt(a)
m.nl_exp(a), m.nl_log(a)
m.nl_sin(a), m.nl_cos(a), m.nl_tan(a)
m.nl_abs(a)
m.nl_max(a, b), m.nl_min(a, b)
```

### NLP Warm Start

```python
m.set_nlp_x0(x0: list[float])   # initial guess for NLP solver
```

### Diagnostics & Export

```python
m.print_summary()
m.check_bounds()          # raises if any lb > ub
# Raises when a scalar is unset, an indexed table is empty, or a stored value
# is non-finite. Parameter declarations have dimensions but no Set, so this
# cannot prove full indexed-domain coverage.
m.check_missing_params()

m.write_lp(path)     # linear LP/MILP only; unsupported model classes raise
m.write_mps(path)    # free MPS, including integrality and objective offset
m.write_json(path)   # Beta
```

### Solve

```python
result = m.solve()
result = m.solve(opts)   # aml.SolveOptions
```

---

## Helper Functions

```python
aml.sum_over(set, fn)        -> LinearExpr
# fn: Key -> LinearExpr; sums fn(k) for all k in set

aml.sum_over_quad(set, fn)   -> QuadExpr
# fn: Key -> QuadExpr
```

---

## Complete Examples

### Integer Production Planning

```python
import mipsolvers
aml = mipsolvers.aml

m = aml.Model("prod")
products = m.add_set("products", ["A", "B", "C"])

profit   = m.add_param("profit",   dim=1, unit="$/unit")
capacity = m.add_param_scalar("capacity", unit="hours")
hours    = m.add_param("hours",    dim=1, unit="hours/unit")

profit.load({"A": 25.0, "B": 30.0, "C": 15.0})
capacity.set_scalar(100.0)
hours.load({"A": 2.0, "B": 3.0, "C": 1.0})

x = m.add_var("x", products, aml.VarType.Integer, lb=0.0)

m.maximize(aml.sum_over(products, lambda p: profit[p] * x[p]))

m.add_constraint(
    aml.sum_over(products, lambda p: hours[p] * x[p]) <= capacity.get_scalar(),
    "capacity"
)

opts = aml.SolveOptions()
opts.verbosity = 0
res = m.solve(opts)

if res.is_optimal:
    print(f"Profit: {res.objective_value:.2f}")
    for p in products:
        k = p.values[0]
        print(f"  {k}: {res.var_value(x[k]):.0f} units")
```

### Time-Indexed Energy Storage (OrderedSet)

```python
import mipsolvers
aml = mipsolvers.aml

m = aml.Model("storage")
T = m.add_ordered_set("T", [f"t{i}" for i in range(1, 25)])

price  = m.add_param("price", dim=1, unit="$/MWh")
price.load({f"t{i}": float(i % 12 + 1) * 10.0 for i in range(1, 25)})

charge    = m.add_var("charge",    T, aml.VarType.Continuous, lb=0.0, ub=50.0)
discharge = m.add_var("discharge", T, aml.VarType.Continuous, lb=0.0, ub=50.0)
soc       = m.add_var("soc",       T, aml.VarType.Continuous, lb=0.0, ub=200.0)

# Maximize revenue
m.maximize(aml.sum_over(T, lambda t:
    price[t] * discharge[t] - 0.8 * price[t] * charge[t]))

# SOC dynamics
for t in T:
    k = t.values[0]
    prev = T.prev(t)
    if prev is None:
        m.add_constraint(soc[k] == charge[k] - discharge[k], f"soc_init_{k}")
    else:
        prev_k = prev.values[0]
        m.add_constraint(
            soc[k] == soc[prev_k] + charge[k] - discharge[k],
            f"soc_{k}"
        )

res = m.solve()
print(f"Revenue: {res.objective_value:.2f}")
```

### Nonlinear Optimization (NLP)

```python
import mipsolvers
aml = mipsolvers.aml

m = aml.Model("nlp")
vars_ = m.add_set("vars", ["x", "y"])
v = m.add_var("v", vars_, aml.VarType.Continuous, lb=-10.0, ub=10.0)

x, y = m.nl_var(v["x"]), m.nl_var(v["y"])

# Rosenbrock: min (1-x)^2 + 100(y-x^2)^2
one_minus_x = m.nl_sub(m.nl_const(1.0), x)
y_minus_xsq = m.nl_sub(y, m.nl_sq(x))

obj = m.nl_add(
    m.nl_sq(one_minus_x),
    m.nl_mul(m.nl_const(100.0), m.nl_sq(y_minus_xsq))
)
m.minimize_nl(obj)
m.set_nlp_x0([0.5, 0.5])

res = m.solve()
print(f"Minimum at x={res.var_value(v['x']):.4f}, y={res.var_value(v['y']):.4f}")
```

---

## MissingPolicy (enum)

Controls behavior when a `Parameter.get()` is called for a missing key. Not yet directly configurable per-parameter in this version; the default policy is `Error`.

| Value | Meaning |
|---|---|
| `MissingPolicy.Error` | Raise on missing key (default) |
| `MissingPolicy.ReturnZero` | Return 0.0 silently |
