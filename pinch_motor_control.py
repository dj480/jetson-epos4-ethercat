# Forward the standalone command to the pinch-to-motor controller.
from python.pinch_motor_control import main


if __name__ == "__main__":
    # Start the controller when invoked from the repository root.
    main()
