# C++ Design for a Three-Phase Hybrid AC/DC Transient Simulation Module

You should design the new transient module as a **DAE-based, device-stamped, sparse-matrix simulation engine** that reuses your existing `hacdcpf` static modeling pipeline.

At a high level:

```text
HybridPowerSystem
        ↓
Projection / Canonicalization
        ↓
Static PF / OPF initialization
        ↓
DynamicModelBuilder
        ↓
DynamicSolverData
        ↓
DAE / time-domain transient solver
        ↓
DynamicResults
```

The core idea is:

> Keep your existing PF/OPF module as the initialization and planning layer, then add a new `dynamics` runtime family that solves three-phase unbalanced hybrid AC/DC transient DAEs using sparse linear solvers and time integration.

---

# 1. Recommended Module Architecture

Add a new runtime family:

```text
include/hacdcpf/dynamics/
src/dynamics/
tests/dynamics/
```

Suggested layout:

```text
include/hacdcpf/dynamics/
    DynamicSystem.hpp
    DynamicModelBuilder.hpp
    DynamicSolver.hpp
    DynamicSolverOptions.hpp
    DynamicResults.hpp
    DynamicEvent.hpp
    DynamicState.hpp
    NetworkState.hpp
    DynamicStamp.hpp

    devices/
        DynamicDevice.hpp
        SynchronousMachine.hpp
        Governor.hpp
        Exciter.hpp
        GridFollowingInverter.hpp
        GridFormingInverter.hpp
        VSCConverterDynamic.hpp
        DCDCConverterDynamic.hpp
        BatteryDynamic.hpp
        PVDynamic.hpp
        DynamicLoad.hpp
        ProtectionRelay.hpp

    integration/
        TimeIntegrator.hpp
        ExplicitEuler.hpp
        RK4.hpp
        BackwardEuler.hpp
        Trapezoidal.hpp
        Rosenbrock.hpp

    solvers/
        SparseLinearSolver.hpp
        NewtonSolver.hpp
        DaeSolver.hpp
        AlgebraicNetworkSolver.hpp

src/dynamics/
    DynamicSystem.cpp
    DynamicModelBuilder.cpp
    DynamicSolver.cpp
    DynamicResults.cpp
    DynamicEvent.cpp

    devices/
        SynchronousMachine.cpp
        GridFollowingInverter.cpp
        GridFormingInverter.cpp
        VSCConverterDynamic.cpp
        DCDCConverterDynamic.cpp
        BatteryDynamic.cpp
        PVDynamic.cpp
        DynamicLoad.cpp

    integration/
        ExplicitEuler.cpp
        RK4.cpp
        BackwardEuler.cpp
        Trapezoidal.cpp
        Rosenbrock.cpp

    solvers/
        NewtonSolver.cpp
        AlgebraicNetworkSolver.cpp
```

---

# 2. Core Design Principle

Use a **component-stamping architecture**, similar to circuit simulation.

Each dynamic device contributes:

1. **Dynamic states**:

$$
\dot{x}_d = f_d(x_d,y,u,t)
$$

2. **Algebraic network injections**:

$$
i_d = i_{N,d}(x_d,u,t) - Y_{N,d}(x_d)y
$$

3. **Jacobian blocks** for implicit integration.

This is the same pattern used in:

- power-system transient stability programs,
- EMT simulators,
- circuit simulators such as SPICE,
- GridLAB-D-style electromechanical dynamic simulations.

---

# 3. Main Runtime Objects

## 3.1 `DynamicSystem`

This owns the complete dynamic model.

```cpp
class DynamicSystem {
public:
    DynamicNetwork network;
    std::vector<std::unique_ptr<DynamicDevice>> devices;
    std::vector<DynamicEvent> events;

    DynamicState x;
    NetworkState y;

    DynamicSolverOptions options;
};
```

It should contain:

- AC three-phase network matrix,
- DC network matrix,
- device list,
- global state vector,
- algebraic variable vector,
- events,
- initialization data,
- solver options.

---

## 3.2 `DynamicState`

Stores dynamic variables in one contiguous vector.

```cpp
struct DynamicState {
    Eigen::VectorXd x;
    Eigen::VectorXd dxdt;

    double time = 0.0;

    void resize(std::size_t n);
};
```

Each device receives an index range into this vector:

```cpp
struct StateIndexRange {
    int offset;
    int size;
};
```

Example:

```cpp
machine.stateRange = {0, 8};
inverter.stateRange = {8, 6};
vsc.stateRange = {14, 7};
```

This avoids fragmented memory and improves solver performance.

---

## 3.3 `NetworkState`

Stores algebraic variables.

```cpp
struct NetworkState {
    Eigen::VectorXcd Vac_abc;  // size = 3 * number of AC buses
    Eigen::VectorXd  Vdc;      // size = number of DC buses

    Eigen::VectorXcd Iac_abc;
    Eigen::VectorXd  Idc;
};
```

For Newton-based DAE solvers, you may also want a real-valued representation:

```cpp
struct NetworkStateReal {
    Eigen::VectorXd y;
    // y = [Re(Vabc), Im(Vabc), Vdc]
};
```

For implicit solvers, I strongly recommend using real-valued vectors internally:

$$
y =
\begin{bmatrix}
\operatorname{Re}(V_{abc}) \\
\operatorname{Im}(V_{abc}) \\
V_{dc}
\end{bmatrix}
$$

because most nonlinear solvers expect real systems.

---

# 4. Device Interface Design

The most important interface is `DynamicDevice`.

```cpp
class DynamicDevice {
public:
    virtual ~DynamicDevice() = default;

    virtual void assignStateIndices(int& offset) = 0;

    virtual void initializeFromPowerFlow(
        const PowerFlowResult& pf,
        DynamicState& x,
        NetworkState& y
    ) = 0;

    virtual void computeDerivatives(
        double t,
        const DynamicState& x,
        const NetworkState& y,
        Eigen::Ref<Eigen::VectorXd> dxdt
    ) const = 0;

    virtual void stamp(
        double t,
        const DynamicState& x,
        const NetworkState& y,
        DynamicStamp& stamp
    ) const = 0;

    virtual void updateAlgebraicOutputs(
        const DynamicState& x,
        const NetworkState& y
    ) = 0;

    virtual void handleEvent(
        const DynamicEvent& event,
        DynamicState& x,
        NetworkState& y
    ) {}

    virtual void addJacobian(
        double t,
        const DynamicState& x,
        const NetworkState& y,
        SparseTripletList& J
    ) const {}
};
```

This lets each device:

- initialize itself,
- compute state derivatives,
- stamp Norton equivalents,
- update measured powers/currents,
- contribute Jacobian terms.

---

# 5. Dynamic Stamp Design

Use a unified stamp object for AC and DC contributions.

```cpp
struct DynamicStamp {
    std::vector<Eigen::Triplet<std::complex<double>>> YacTriplets;
    Eigen::VectorXcd Iac;

    std::vector<Eigen::Triplet<double>> GdcTriplets;
    Eigen::VectorXd Idc;

    void clear();
    void addAcAdmittance(int row, int col, std::complex<double> value);
    void addAcCurrent(int row, std::complex<double> value);
    void addDcConductance(int row, int col, double value);
    void addDcCurrent(int row, double value);
};
```

For a device connected to AC bus `b`, phase indices are:

```cpp
int ia = 3 * b + 0;
int ib = 3 * b + 1;
int ic = 3 * b + 2;
```

For a grid-forming inverter:

```cpp
Iinj = Yv * Eabc - Yv * Vabc
```

Stamp:

```cpp
Yac += Yv;
Iac += Yv * Eabc;
```

Since the network equation is usually assembled as:

$$
Y_{eff}V = I_{eff}
$$

the virtual impedance admittance is added to the matrix, and the internal-source current is added to the RHS.

---

# 6. Network Solver Design

## 6.1 AC Network Solver

At each step, solve:

$$
Y_{eff}^{abc} V^{abc} = I_{eff}^{abc}
$$

In C++ with Eigen:

```cpp
Eigen::SparseMatrix<std::complex<double>> Yeff;
Eigen::VectorXcd Ieff;
Eigen::VectorXcd V;

solver.compute(Yeff);
V = solver.solve(Ieff);
```

Recommended sparse solvers:

| Solver | Type | Recommended use |
|---|---|---|
| Eigen `SparseLU` | Direct | small/medium feeders, robust |
| Eigen `SparseQR` | Direct | ill-conditioned cases |
| Eigen `BiCGSTAB` | Iterative | very large systems |
| Eigen `GMRES` unsupported module | Iterative | large sparse complex systems |
| KLU / SuiteSparse | Direct | best for circuit-like sparse systems |
| UMFPACK / SuiteSparse | Direct | robust sparse LU |
| Intel MKL PARDISO | Direct | high-performance production |
| MUMPS | Parallel direct | very large systems |
| PETSc + KSP | Iterative/parallel | large-scale HPC |

For your module, my practical recommendation:

```text
Default: Eigen::SparseLU
Production optional: SuiteSparse KLU
Large-scale optional: PETSc or PARDISO
```

KLU is especially suitable because your network matrices are circuit-like sparse admittance matrices.

---

## 6.2 DC Network Solver

For algebraic DC:

$$
G_{dc}V_{dc}=I_{dc}
$$

Use real sparse direct solvers:

```cpp
Eigen::SparseMatrix<double> Gdc;
Eigen::VectorXd Idc;
Eigen::VectorXd Vdc;

solver.compute(Gdc);
Vdc = solver.solve(Idc);
```

Recommended:

- Eigen `SparseLU`,
- Eigen `SimplicialLDLT` if symmetric positive definite,
- KLU,
- CHOLMOD if applicable,
- PARDISO.

If DC capacitance is modeled dynamically:

$$
C_{dc}\dot{V}_{dc}=I_{inj}-G_{dc}V_{dc}
$$

then $$V_{dc}$$ becomes part of the dynamic state vector.

---

# 7. Solver Types You Need

You should implement the transient module in layers.

## Layer 1: Algebraic Network Solver

Solves:

$$
Y_{eff}V=I_{eff}
$$

Used in explicit and semi-implicit simulation.

## Layer 2: Time Integrator

Integrates:

$$
\dot{x}=f(x,y,t)
$$

Examples:

- explicit Euler,
- RK4,
- backward Euler,
- trapezoidal,
- Rosenbrock.

## Layer 3: Nonlinear Newton Solver

Needed for implicit DAE methods.

Solves:

$$
R(x_{n+1},y_{n+1})=0
$$

## Layer 4: Event Solver

Handles:

- line trips,
- load steps,
- faults,
- converter trips,
- protection events.

---

# 8. Recommended Solver Strategy

## 8.1 Minimum viable solver

For the first implementation, use a partitioned explicit or semi-implicit approach:

```text
For each time step:
    1. Apply events.
    2. Build Norton stamps from current device states.
    3. Solve AC/DC network algebraic equations.
    4. Compute device powers/currents.
    5. Compute dx/dt.
    6. Integrate states with RK4 or modified Euler.
```

Recommended first integrator:

```text
RK4 or Heun / predictor-corrector
```

This is easier to debug and sufficient for slow electromechanical dynamics.

---

## 8.2 Production-grade solver

For converter-rich systems, use an implicit DAE solver.

Recommended:

```text
Implicit trapezoidal + Newton + sparse LU/KLU
```

or:

```text
Backward Euler + Newton + sparse LU/KLU
```

Backward Euler is more numerically damped but very robust.

Trapezoidal is more accurate but can show numerical oscillations in stiff systems.

---

## 8.3 Best long-term option

For a high-quality C++ implementation, support multiple solver modes:

```cpp
enum class DynamicSolverType {
    PartitionedRK4,
    PartitionedHeun,
    BackwardEulerNewton,
    TrapezoidalNewton,
    RosenbrockW,
    IDA_Sundials
};
```

Then expose options:

```cpp
struct DynamicSolverOptions {
    DynamicSolverType solverType = DynamicSolverType::TrapezoidalNewton;

    double tStart = 0.0;
    double tEnd = 10.0;
    double dt = 0.001;

    double absTol = 1e-8;
    double relTol = 1e-6;

    int maxNewtonIters = 20;
    double newtonTol = 1e-8;

    bool useAdaptiveStep = false;
    bool useAnalyticJacobian = false;
    bool useNumericalJacobian = true;

    LinearSolverType linearSolver = LinearSolverType::KLU;
};
```

---

# 9. External C++ Solver Libraries

## 9.1 Linear algebra

### Recommended baseline

Use **Eigen**.

Pros:

- header-only,
- easy integration,
- supports sparse and dense matrices,
- good for prototyping,
- already common in C++ scientific projects.

Use:

```cpp
#include <Eigen/Sparse>
#include <Eigen/Dense>
```

### Recommended production upgrade

Use **SuiteSparse KLU**.

Why KLU?

Because power-network admittance matrices are sparse circuit matrices. KLU is designed for that structure.

Use KLU for:

- sparse complex AC network solve,
- sparse real DC network solve,
- Newton Jacobian factorization.

### Best high-performance option

Use one or more optional backends:

- Intel MKL PARDISO,
- MUMPS,
- PETSc,
- SuperLU,
- UMFPACK.

---

## 9.2 DAE / ODE solvers

You can either implement your own integrators or use external libraries.

### Good external options

| Library | Use |
|---|---|
| SUNDIALS IDA | DAE solver |
| SUNDIALS CVODE | ODE solver |
| Boost.Odeint | ODE integration |
| PETSc TS | large-scale time stepping |
| Assimulo | Python-oriented wrapper, not ideal for pure C++ |
| CasADi | symbolic/algorithmic differentiation, optimization, not lightweight |

### Strong recommendation

For production DAE solving:

```text
Use SUNDIALS IDA.
```

SUNDIALS IDA solves systems of the form:

$$
F(t,y,\dot{y})=0
$$

This matches your transient DAE naturally.

You can define:

$$
Y =
\begin{bmatrix}
x \\
y
\end{bmatrix}
$$

and:

$$
\dot{Y}
=
\begin{bmatrix}
\dot{x} \\
\dot{y}
\end{bmatrix}
$$

Then construct residual:

$$
F(t,Y,\dot{Y})
=
\begin{bmatrix}
\dot{x}-f(x,y,t) \\
g(x,y,t)
\end{bmatrix}
=
0
$$

In code:

```cpp
int residual(double t,
             N_Vector Y,
             N_Vector Ydot,
             N_Vector R,
             void* userData);
```

This is the cleanest formulation for a general DAE.

---

# 10. Should You Implement Your Own DAE Solver or Use SUNDIALS?

## Recommended answer

Use both levels:

### Internal native solver

Implement:

- partitioned RK4,
- backward Euler,
- trapezoidal Newton.

This gives you:

- control,
- easier debugging,
- no external dependency for basic simulations.

### Optional production DAE solver

Add SUNDIALS IDA backend.

This gives you:

- adaptive time stepping,
- robust error control,
- mature DAE handling,
- better stiffness handling.

So your solver stack should be:

```text
Native simple solvers for default/basic use
SUNDIALS IDA for advanced/production transient simulation
KLU/PARDISO/SuiteSparse for sparse linear solves
```

---

# 11. Recommended Solver Matrix

| Simulation case | Recommended solver |
|---|---|
| Synchronous-machine-only transient | RK4 or trapezoidal |
| Distribution feeder RMS dynamics | trapezoidal Newton |
| Converter-rich microgrid | backward Euler or IDA |
| Fast current-control dynamics | IDA or Rosenbrock |
| Long event simulation | adaptive IDA |
| Large network with many buses | KLU/PARDISO |
| Research prototype | Eigen SparseLU + RK4 |
| Production-grade module | SUNDIALS IDA + KLU |

---

# 12. Detailed Solver Design

## 12.1 Partitioned transient solver

This is easiest to implement first.

```cpp
class PartitionedDynamicSolver {
public:
    DynamicResults solve(DynamicSystem& system) {
        auto& opt = system.options;

        for (double t = opt.tStart; t <= opt.tEnd; t += opt.dt) {
            system.applyEvents(t);

            assembleNetwork(system);
            solveNetwork(system);

            computeDeviceDerivatives(system);

            integrateStates(system);

            storeResults(system);
        }

        return results;
    }
};
```

Pseudo-code:

```cpp
void step(DynamicSystem& sys, double t, double dt) {
    sys.applyEvents(t);

    DynamicStamp stamp(sys.network.size());
    for (auto& dev : sys.devices) {
        dev->stamp(t, sys.x, sys.y, stamp);
    }

    sys.network.assemble(stamp);
    sys.network.solve(sys.y);

    for (auto& dev : sys.devices) {
        dev->computeDerivatives(t, sys.x, sys.y, sys.x.dxdt);
    }

    integrator.advance(sys.x, sys.x.dxdt, dt);
}
```

This is good for:

- initial version,
- debugging,
- paper reproduction,
- moderate machine dynamics.

---

## 12.2 Trapezoidal Newton solver

For implicit trapezoidal integration:

$$
R_x =
x_{n+1}
-
x_n
-
\frac{\Delta t}{2}
\left[
f_n+f_{n+1}
\right]
$$

$$
R_y =
g(x_{n+1},y_{n+1})
$$

Solve:

$$
R(x_{n+1},y_{n+1})=0
$$

C++ structure:

```cpp
class TrapezoidalNewtonSolver {
public:
    bool step(DynamicSystem& sys, double t, double dt) {
        Vector z = pack(sys.x, sys.y);
        Vector zGuess = predict(sys, dt);

        for (int iter = 0; iter < maxNewtonIters; ++iter) {
            Vector R = computeResidual(zGuess);
            if (R.norm() < tol) {
                unpack(zGuess, sys.x, sys.y);
                return true;
            }

            SparseMatrix J = computeJacobian(zGuess);
            Vector dz = linearSolver.solve(J, -R);

            zGuess += lineSearch(dz);
        }

        return false;
    }
};
```

Real-valued unknown vector:

$$
z =
\begin{bmatrix}
x \\
\operatorname{Re}(V^{abc}) \\
\operatorname{Im}(V^{abc}) \\
V^{dc}
\end{bmatrix}
$$

This avoids complex-valued Newton complications.

---

# 13. Numerical Jacobian vs Analytical Jacobian

## Initial implementation

Use numerical finite-difference Jacobian:

$$
J_{ij}
=
\frac{
R_i(z+\epsilon e_j)-R_i(z)
}{
\epsilon
}
$$

C++:

```cpp
for (int j = 0; j < n; ++j) {
    Vector zp = z;
    zp[j] += eps;

    Vector Rp = residual(zp);
    J.col(j) = (Rp - R0) / eps;
}
```

This is simple but slow.

## Production implementation

Use analytical or semi-analytical Jacobian.

Recommended hybrid approach:

- network admittance Jacobian analytically,
- device Jacobians analytically where possible,
- finite differences fallback for custom devices.

Interface:

```cpp
virtual void addJacobian(
    const DynamicState& x,
    const NetworkState& y,
    SparseTripletList& J
) const;
```

---

# 14. Dynamic Network Representation

Create a class for the three-phase AC network.

```cpp
class ThreePhaseACNetwork {
public:
    int numBuses;
    int numPhases; // usually 3

    Eigen::SparseMatrix<std::complex<double>> Ybase;
    Eigen::SparseMatrix<std::complex<double>> Yeff;

    void buildBaseYbus(const HybridPowerSystem& system);
    void assembleEffectiveY(const DynamicStamp& stamp);
    void solve(NetworkState& y);
};
```

For each AC branch:

```cpp
struct AC3PBranch {
    int fromBus;
    int toBus;

    Eigen::Matrix3cd Zabc;
    Eigen::Matrix3cd Yseries;
    Eigen::Matrix3cd YshuntFrom;
    Eigen::Matrix3cd YshuntTo;

    bool inService = true;
};
```

Stamp branch:

```cpp
Y[from, from] += Yseries + YshuntFrom;
Y[to, to]     += Yseries + YshuntTo;
Y[from, to]   -= Yseries;
Y[to, from]   -= Yseries;
```

---

# 15. Dynamic Device Implementations

## 15.1 Synchronous Machine class

```cpp
class SynchronousMachine : public DynamicDevice {
public:
    int bus;
    MachineParams params;
    StateIndexRange range;

    void computeDerivatives(...) const override;
    void stamp(...) const override;
    void initializeFromPowerFlow(...) override;

private:
    Eigen::Matrix3cd computeYabc(double delta) const;
    Eigen::Vector3cd computeNortonCurrent(const DynamicState& x) const;
};
```

State layout:

```cpp
enum {
    DELTA = 0,
    OMEGA = 1,
    EQP   = 2,
    EDP   = 3,
    EQPP  = 4,
    EDPP  = 5,
    PM    = 6,
    EFD   = 7
};
```

Derivative:

```cpp
dx[DELTA] = wb * (omega - 1.0);
dx[OMEGA] = (Pm - Pe - D * (omega - 1.0)) / (2.0 * H);
dx[EQP]   = (Efd - Eqp - (xd - xdp) * id) / Tdop;
...
```

Stamp:

```cpp
Yabc = T_abc * Ydq * T_dq;
INorton = -T_abc * Ydq * Epp_dq;

stamp.addAcBlock(bus, bus, Yabc);
stamp.addAcCurrent(bus, INorton);
```

---

## 15.2 Grid-following inverter class

```cpp
class GridFollowingInverter : public DynamicDevice {
public:
    int acBus;
    int dcBus;

    GFLParams params;
    StateIndexRange range;

    void computeDerivatives(...) const override;
    void stamp(...) const override;
};
```

State layout:

```cpp
enum {
    THETA_PLL = 0,
    XI_PLL    = 1,
    ID        = 2,
    IQ        = 3,
    XI_P      = 4,
    XI_Q      = 5
};
```

Stamp as current source:

```cpp
Eigen::Vector3cd Iabc = dqToAbc(id, iq, theta_pll);
stamp.addAcCurrent(acBus, Iabc);
```

Optional stabilizing admittance:

```cpp
stamp.addAcBlock(acBus, acBus, Ystab);
```

---

## 15.3 Grid-forming inverter class

```cpp
class GridFormingInverter : public DynamicDevice {
public:
    int acBus;
    GFMParams params;
    StateIndexRange range;

    void computeDerivatives(...) const override;
    void stamp(...) const override;
};
```

State layout:

```cpp
enum {
    THETA = 0,
    OMEGA = 1,
    E = 2,
    PF = 3,
    QF = 4
};
```

Stamp as voltage source behind impedance:

```cpp
Eigen::Vector3cd Eabc = makeBalancedVoltage(E, theta);
Eigen::Matrix3cd Yv = virtualAdmittance();

Eigen::Vector3cd IN = Yv * Eabc;

stamp.addAcBlock(acBus, acBus, Yv);
stamp.addAcCurrent(acBus, IN);
```

---

## 15.4 VSC converter class

```cpp
class VSCConverterDynamic : public DynamicDevice {
public:
    int acBus;
    int dcBus;

    VSCParams params;
    VSCControlMode mode;

    StateIndexRange range;

    void computeDerivatives(...) const override;
    void stamp(...) const override;
    void stampDc(...) const;
};
```

VSC should internally use:

- grid-following mode,
- grid-forming mode,
- Vdc-Q mode,
- Vdc-Vac mode.

DC current injection:

$$
I_{dc} = \frac{P_{dc}}{V_{dc}}
$$

In code:

```cpp
double Idc = Pdc / std::max(Vdc, eps);
stamp.addDcCurrent(dcBus, Idc);
```

---

## 15.5 DC/DC converter class

```cpp
class DCDCConverterDynamic : public DynamicDevice {
public:
    int dcFrom;
    int dcTo;
    DCDCParams params;
    StateIndexRange range;

    void computeDerivatives(...) const override;
    void stamp(...) const override;
};
```

Stamp DC current injections:

```cpp
double Pin  = Pout / eta;
double Iin  = -Pin / Vfrom;
double Iout =  Pout / Vto;

stamp.addDcCurrent(dcFrom, Iin);
stamp.addDcCurrent(dcTo, Iout);
```

---

# 16. Event System Design

Use an event queue.

```cpp
enum class EventType {
    LoadStep,
    LoadTrip,
    LineTrip,
    FaultApply,
    FaultClear,
    GeneratorTrip,
    ConverterTrip,
    ControlReferenceChange
};

struct DynamicEvent {
    double time;
    EventType type;
    int targetId;

    std::unordered_map<std::string, double> params;
};
```

At each time step:

```cpp
while (!events.empty() && events.front().time <= t) {
    applyEvent(events.front());
    events.pop();
}
```

For topology events, rebuild base admittance:

```cpp
network.rebuildYbase();
```

For faults, stamp temporary shunt admittance:

```cpp
Yfault = 1.0 / Zfault;
Ybus(k,k) += Yfault;
```

---

# 17. Result Storage Design

Store time series efficiently.

```cpp
struct DynamicResults {
    std::vector<double> time;

    std::vector<Eigen::VectorXd> states;
    std::vector<Eigen::VectorXd> voltagesMag;
    std::vector<Eigen::VectorXd> voltagesAng;
    std::vector<Eigen::VectorXd> dcVoltages;

    std::unordered_map<std::string, std::vector<double>> deviceSignals;
};
```

For large simulations, avoid storing every state at every time step. Add sampling:

```cpp
int outputEvery = 10;
```

or streaming output:

```cpp
class ResultWriter {
public:
    void writeStep(double t, const DynamicSystem& sys);
};
```

---

# 18. C++ Solver Choices Summary

## 18.1 Linear solvers

Use:

```text
Eigen SparseLU for initial implementation.
SuiteSparse KLU for production.
PARDISO/PETSc for very large systems.
```

Recommended enum:

```cpp
enum class LinearSolverType {
    EigenSparseLU,
    EigenSparseQR,
    EigenBiCGSTAB,
    KLU,
    UMFPACK,
    Pardiso,
    PETSc
};
```

---

## 18.2 Time-domain solvers

Recommended enum:

```cpp
enum class TimeIntegratorType {
    ExplicitEuler,
    Heun,
    RK4,
    BackwardEuler,
    Trapezoidal,
    Rosenbrock,
    SundialsIDA
};
```

Recommended defaults:

| Use case | Default |
|---|---|
| debugging | RK4 |
| synchronous-machine RMS dynamics | trapezoidal |
| converter-rich systems | backward Euler or IDA |
| production adaptive | SUNDIALS IDA |
| very large system | IDA + KLU/PETSc |

---

# 19. Recommended Development Roadmap

## Stage 1: Core phasor dynamic engine

Implement:

- `DynamicSystem`,
- `DynamicDevice`,
- `DynamicStamp`,
- `ThreePhaseACNetwork`,
- `DCNetwork`,
- partitioned RK4 solver,
- Eigen SparseLU.

Devices:

- synchronous machine,
- governor,
- exciter,
- ZIP load.

Goal:

```text
Reproduce GridLAB-D paper test cases.
```

---

## Stage 2: Native implicit solver

Add:

- backward Euler Newton,
- trapezoidal Newton,
- numerical Jacobian,
- sparse real-valued Newton system.

Goal:

```text
Stable simulations for stiff machines and inverter controls.
```

---

## Stage 3: Power electronics

Add:

- grid-following inverter,
- grid-forming inverter,
- VSC dynamic model,
- DC/DC converter,
- battery,
- PV.

Goal:

```text
Hybrid AC/DC microgrid transient analysis.
```

---

## Stage 4: Production solver backends

Add:

- SuiteSparse KLU,
- SUNDIALS IDA,
- optional PETSc/PARDISO.

Goal:

```text
Robust, adaptive, large-scale transient simulation.
```

---

## Stage 5: Protection and co-simulation

Add:

- events,
- relays,
- ride-through,
- UFLS/UVLS,
- GridLAB-D/HELICS interface.

---

# 20. Minimal C++ Skeleton

A simplified skeleton:

```cpp
class DynamicSolver {
public:
    DynamicResults solve(DynamicSystem& sys) {
        initialize(sys);

        for (double t = options.tStart; t <= options.tEnd; t += options.dt) {
            sys.applyEvents(t);

            DynamicStamp stamp(sys.network.numAcNodes(), sys.network.numDcNodes());

            for (auto& dev : sys.devices) {
                dev->stamp(t, sys.x, sys.y, stamp);
            }

            sys.network.assemble(stamp);
            sys.network.solve(sys.y);

            sys.x.dxdt.setZero();

            for (auto& dev : sys.devices) {
                dev->computeDerivatives(t, sys.x, sys.y, sys.x.dxdt);
            }

            integrator.advance(sys.x.x, sys.x.dxdt, options.dt);

            results.store(t, sys);
        }

        return results;
    }

private:
    DynamicSolverOptions options;
    TimeIntegrator integrator;
};
```

---

# 21. Practical Recommendation

For your module, I would implement the solver stack as follows:

## Initial implementation

```text
C++17 or C++20
Eigen SparseLU
Partitioned RK4 / Heun
Three-phase complex Ybus
Device Norton stamping
```

## Robust implementation

```text
Implicit trapezoidal Newton
Real-valued residual/Jacobian
SuiteSparse KLU
Analytical Jacobian for network
Numerical fallback for devices
```

## Production implementation

```text
SUNDIALS IDA
KLU or PARDISO backend
Adaptive time stepping
Event localization
Protection logic
```

---

# 22. Final Design Recommendation

Your C++ module should be designed as:

```text
Device-stamped hybrid AC/DC DAE simulator
```

with these core abstractions:

```text
DynamicSystem
DynamicDevice
DynamicStamp
NetworkState
DynamicState
DynamicSolver
TimeIntegrator
SparseLinearSolver
DynamicEvent
DynamicResults
```

Use these solvers:

```text
Eigen SparseLU for prototype
KLU/SuiteSparse for production sparse network/Newton solves
SUNDIALS IDA for production DAE integration
Backward Euler / trapezoidal Newton as native built-in solvers
```

And organize the numerical method around:

$$
\dot{x}=f(x,y,u,t)
$$

$$
0=g(x,y,u,t)
$$

with network stamping:

$$
Y_{eff}^{abc}V^{abc}=I_{eff}^{abc}
$$

and optional dynamic DC network:

$$
C_{dc}\dot{V}_{dc}=I_{dc}-G_{dc}V_{dc}
$$

This architecture will let your existing static `hacdcpf` module evolve naturally into a **three-phase unbalanced hybrid AC/DC transient simulation engine** capable of handling both synchronous machines and modern power-electronic devices.