# Forward the standalone command to the arm-following motor controller.
from python.arm_motor_control import main


if __name__ == "__main__":
    # Start the controller when invoked from the repository root.
    main()
