# Model-Based Design of an Intelligent Tire Pressure Monitoring System (TPMS)
### Using Simulink, Stateflow, and Embedded Coder

---

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [Tools and Parameters](#2-tools-and-parameters)
3. [Part 1 — Tire Pressure Plant Model](#3-part-1--tire-pressure-plant-model)
4. [Part 2 — Signal Processing Layer](#4-part-2--signal-processing-layer)
5. [Part 3 — Stateflow FSM](#5-part-3--stateflow-fsm)
6. [Part 4 — Dashboard and Code Generation](#6-part-4--dashboard-and-code-generation)
7. [How to Run the Model](#7-how-to-run-the-model)
8. [Results Summary](#8-results-summary)

---

## 1. Project Overview

A conventional TPMS simply warns the driver when tire pressure drops below a fixed threshold — a purely **reactive** system. This project implements an **intelligent, predictive** TPMS that:

- Models realistic tire pressure dynamics using differential equations
- Detects **rate of pressure change (dP/dt)** to predict leaks before they become critical
- Uses a **Stateflow hierarchical FSM** to classify system state in real time
- Generates **production-ready C code** via Embedded Coder for embedded deployment
- Displays live status on a **Simulink Dashboard**

The system transitions through four states:

```
NORMAL → WARNING (SLOW_LEAK / FAST_LEAK) → CRITICAL
```

---

## 2. Tools and Parameters

### MATLAB Toolboxes Used
| Toolbox | Purpose |
|---|---|
| Simulink | Plant model + signal processing blocks |
| Stateflow | Hierarchical FSM for decision logic |
| Embedded Coder | C code generation for embedded deployment |
| Dashboard | Real-time visualization |

### All Model Parameters
```matlab
% Tire Pressure Dynamics
P_initial  = 32;       % Initial tire pressure (psi)
P_atm      = 14.7;     % Atmospheric pressure (psi)
C_leak     = 0.001;    % Leak coefficient (slow puncture)

% Signal Processing Thresholds
P_warn     = 26;       % Warning pressure threshold (psi)
P_critical = 20;       % Critical pressure threshold (psi)
dPdt_slow  = -0.02;    % Slow leak rate threshold (psi/s)
dPdt_fast  = -0.05;    % Fast leak rate threshold (psi/s)

% Protection
epsilon    = 0.0001;   % Divide-by-zero protection for time-to-flat
```

### C_leak Scenarios
| Scenario | C_leak Value | Stop Time |
|---|---|---|
| Slow puncture | 0.001 | 5000s |
| Fast puncture | 0.01 | 500s |
| Blowout | 0.05 | 100s |

---

## 3. Part 1 — Tire Pressure Plant Model

### Subsystem: `Tire_Pressure_Dynamics`

#### Physics Behind the Model
A tire is a sealed container of pressurized air. When it leaks, air escapes and pressure drops. The behavior follows a **first-order differential equation** derived from the Ideal Gas Law:

**Ideal Gas Law:**
$$P = \frac{nRT}{V}$$

**Leak Flow Equation:**
$$\frac{dP}{dt} = -C_{leak} \times (P - P_{atm})$$

This means pressure drops at a rate proportional to how much higher it is than atmosphere. The solution is an **exponential decay**:

$$P(t) = P_{atm} + (P_{initial} - P_{atm}) \times e^{-C_{leak} \times t}$$

#### Simulink Block Diagram
```
P_atm ──→ [Sum: P - P_atm] ──→ [Gain: -C_leak] ──→ [Integrator 1/s] ──→ P(t)
                ↑                                              │
                └──────────────────────────────────────────────┘
                                  (feedback)
```

#### Key Block Settings
| Block | Setting |
|---|---|
| Integrator | Initial Condition = `P_initial` (32 psi) |
| Gain | Value = `-C_leak` (negative sign critical) |
| Sum | Signs = `+-` (P minus P_atm) |

---

### Subsystem: `Temp_Compensation`

Real tire pressure increases with temperature (~0.1 psi per °C). The compensation formula:

$$P_{sensor} = P(t) + K_T \times (T - T_{ref})$$

| Parameter | Value |
|---|---|
| Temp | 35°C (ambient) |
| Temp_ref | 25°C (reference) |
| K_T | 0.1 psi/°C |
| Offset | +1 psi (35-25 × 0.1) |

---

## 4. Part 2 — Signal Processing Layer

### Subsystem: `Signal_Processing`

#### What It Does
Converts the raw pressure signal into actionable boolean flags for Stateflow.

#### Block Chain
```
P_sensor ──→ [Derivative] ──→ [LPF: 10/(s+10)] ──→ dP_dt (clean)
P_sensor ──→ [<= P_warn]     ──→ warn_flag
P_sensor ──→ [<= P_critical] ──→ critical_flag
dP_dt    ──→ [<= dPdt_slow]  ──→ slow_leak_flag
dP_dt    ──→ [<= dPdt_fast]  ──→ fast_leak_flag
```

#### Why a Low-Pass Filter on the Derivative?
The raw Derivative block amplifies noise. The transfer function:

$$H(s) = \frac{10}{s + 10}$$

attenuates high-frequency noise (cutoff at 10 rad/s) while preserving the slow pressure trend. This is critical for reliable dP/dt computation.

#### Output Signals
| Signal | Type | Meaning |
|---|---|---|
| `dP_dt` | Continuous | Rate of pressure change (psi/s) |
| `warn_flag` | Boolean | 1 when P < 26 psi |
| `critical_flag` | Boolean | 1 when P < 20 psi |
| `slow_leak_flag` | Boolean | 1 when dP/dt < -0.02 |
| `fast_leak_flag` | Boolean | 1 when dP/dt < -0.05 |

---

## 5. Part 3 — Stateflow FSM

### Chart Configuration
| Setting | Value |
|---|---|
| Update Method | Discrete |
| Sample Time | 1 second |
| Action Language | MATLAB |
| Initialize outputs on wake | Enabled |

### State Hierarchy
```
○ ──→ NORMAL
         │
         │ [warn_flag == 1]
         ↓
      WARNING ◄─────────────────────────┐
      ├── SLOW_LEAK (default)           │ [critical_flag == 0 && warn_flag == 1]
      └── FAST_LEAK                     │
         │                              │
         │ [critical_flag == 1]         │
         ↓                              │
      CRITICAL ──────────────────────────┘
         │
         │ [critical_flag == 0 && warn_flag == 0]
         ↓
      NORMAL
```

### State Actions
| State | Action | driver_warning |
|---|---|---|
| NORMAL | `during: driver_warning = 0;` | 0 |
| WARNING | `during: driver_warning = 1;` | 1 |
| CRITICAL | `during: driver_warning = 2;` | 2 |

### All Transitions
| From | To | Condition | Priority |
|---|---|---|---|
| NORMAL | WARNING | `warn_flag == 1` | — |
| WARNING | NORMAL | `warn_flag == 0` | 2 |
| WARNING | CRITICAL | `critical_flag == 1` | 1 |
| CRITICAL | NORMAL | `critical_flag == 0 && warn_flag == 0` | 1 |
| CRITICAL | WARNING | `critical_flag == 0 && warn_flag == 1` | 2 |
| SLOW_LEAK | FAST_LEAK | `fast_leak_flag == 1` | — |
| FAST_LEAK | SLOW_LEAK | `fast_leak_flag == 0` | — |

### Why `during` Instead of `entry`?
- `entry` fires **once** when state is entered → output resets to 0 next step
- `during` fires **every time step** while in the state → output holds value
- Using `entry` caused driver_warning to appear as a spike, not a sustained level

---

### Time-to-Flat Prediction
#### Subsystem: `Time-to-Flat Prediction`

**Formula:**
$$time\_to\_flat = \frac{P_{sensor} - P_{critical}}{|dP/dt| + \epsilon}$$

| Block | Purpose |
|---|---|
| Sum (`+-`) | Computes P_sensor - P_critical |
| Abs | Computes absolute value of dP_dt |
| Sum (`++`) | Adds epsilon to prevent division by zero |
| Divide | Computes the ratio |
| Saturation | Clamps output to max 9999 seconds |

---

## 6. Part 4 — Dashboard and Code Generation

### Dashboard Blocks
| Block | Connected Signal | Purpose |
|---|---|---|
| Gauge (0-40 psi) | `P_sensor` | Live pressure display |
| Green Lamp | `driver_warning == 0` | NORMAL indicator |
| Yellow Lamp | `driver_warning == 1` | WARNING indicator |
| Red Lamp | `driver_warning == 2` | CRITICAL indicator |
| Display | Time-to-Flat output | Seconds until flat |

### Gauge Scale Colors
| Range | Color | Meaning |
|---|---|---|
| 0-20 psi | Red | Critical zone |
| 20-26 psi | Yellow | Warning zone |
| 26-40 psi | Green | Safe zone |

---

## 7. How to Run the Model

### Step 1 — Load Parameters
```matlab
P_initial  = 32;
P_atm      = 14.7;
C_leak     = 0.001;
P_warn     = 26;
P_critical = 20;
dPdt_slow  = -0.02;
dPdt_fast  = -0.05;
epsilon    = 0.0001;
```

### Step 2 — Quick Test (see all states in 100 seconds)
```matlab
C_leak     = 0.05;
P_warn     = 30;
P_critical = 25;
```
Set Stop Time = 100 in Simulink toolbar.

### Step 3 — Run
Press **Ctrl+T** or click the green Run button.

### Step 4 — Observe
- Gauge needle sweeps from 32 psi downward
- Green lamp → Yellow lamp → Red lamp transitions
- driver_warning scope shows 0 → 1 → 2 staircase
- Time-to-flat display counts down

---

## 8. Results Summary

### Verified Outputs

| Signal | Behavior | Verified |
|---|---|---|
| P_sensor | Exponential decay from 32 → 14.7 psi | ✅ |
| dP_dt | Negative signal proportional to C_leak | ✅ |
| warn_flag | Flips to 1 when P < P_warn | ✅ |
| critical_flag | Flips to 1 when P < P_critical | ✅ |
| driver_warning | Clean 0 → 1 → 2 staircase | ✅ |
| time_to_flat | Decreasing countdown in seconds | ✅ |

### Key MBD Concepts Demonstrated
- **Plant modeling** using differential equations in Simulink
- **Signal processing** with derivative + low-pass filter
- **Hierarchical FSM** in Stateflow with entry/during/transition actions
- **Predictive detection** via dP/dt rather than just threshold comparison
- **Subsystem organization** for clean, readable model architecture
- **C code generation** via Embedded Coder for embedded deployment

---

*Model: TPMS_PlantModel.slx | MATLAB R2025a | ECE Undergraduate Project*
