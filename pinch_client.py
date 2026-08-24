# Forward the standalone command to the package implementation.
import argparse
from python.pinch_client import main


if __name__ == "__main__":
    # Start the camera-only pinch event client.
    main()
