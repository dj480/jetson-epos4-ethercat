# Keep the repository-root command compatible with the package implementation.
from python.cli import main


if __name__ == "__main__":
    # Run the interactive motor command-line interface when launched directly.
    main()