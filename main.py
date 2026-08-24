# Provide a short root-level entry point for the interactive motor CLI.
from python.cli import main


if __name__ == "__main__":
    # Only start the CLI when this file is executed as a script.
    main()