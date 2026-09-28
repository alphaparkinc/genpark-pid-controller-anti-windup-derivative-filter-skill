# Closed-Loop PID Controller Skill

Industrial-strength Proportional-Integral-Derivative (PID) controller featuring anti-windup clamping and low-pass derivative filtering.

```mermaid
flowchart LR
    Setpoint["Setpoint r(t)"] --> ErrorSum((+ -))
    Feedback["Measured y(t)"] --> ErrorSum
    ErrorSum --> Error["Error e(t)"]
    Error --> P["Kp * e(t)"]
    Error --> I["Ki * ∫ e dt (Anti-Windup)"]
    Error --> D["Kd * Filtered de/dt"]
    P --> OutSum((+))
    I --> OutSum
    D --> OutSum
    OutSum --> Clamp["Saturation Clamp"]
    Clamp --> Actuator["Control Output u(t)"]
```

## Features
- **100% Python Standard Library**: No external dependencies.
- **Anti-Windup Integration**: Halts integrator drift during actuator saturation.
- **First-Order Derivative Filtering**: Suppresses high-frequency sensor noise.
- **MCP Server Ready**: Direct stdio control loop invocation.
