# Provide a short root-level entry point for the two-joint arm controller.
from python.two_joint_arm_control import main


if __name__ == "__main__":
    # Only start the controller when this file is executed as a script.
    main()
