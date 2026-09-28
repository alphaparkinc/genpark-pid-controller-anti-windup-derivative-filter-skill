"""Example simulation using PID Controller."""
from client import PIDController

def main():
    pid = PIDController(kp=2.5, ki=0.8, kd=0.15, output_limits=(-10.0, 10.0))
    target = 50.0
    state = 0.0
    dt = 0.05
    print(f"Simulating PID tracking target={target}:")
    for step in range(30):
        control = pid.compute(target, state, dt)
        state += control * dt * 2.0  # plant transfer
        if step % 5 == 0 or step == 29:
            print(f"  Step {step:02d}: State={state:.2f} | Error={target-state:.2f} | Control={control:.2f}")

if __name__ == "__main__":
    main()
