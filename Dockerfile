# Bifocal reproducibility image.
#
# Builds a small container that can run the host-buildable, verifiable parts of
# the project: the unit tests, the offline replay simulation, and the Python
# figure and statistics generation. It runs reproduce.sh.
#
# NOTE: the Arduino AVR toolchain is intentionally NOT included. Compiling and
# flashing the microcontroller firmware needs arduino-cli plus the arduino:avr
# core, which are large and hardware-oriented. reproduce.sh detects that
# arduino-cli is absent and SKIPS the firmware compile, so the image still
# reproduces steps 1 to 3 cleanly. To also build the firmware, install
# arduino-cli in a derived image or run that step on the host.
#
# Build:
#   docker build -t bifocal-repro .
#
# Run (regenerate everything and print the summary):
#   docker run --rm bifocal-repro
#
# Copy the regenerated figures back out to the host:
#   cid=$(docker create bifocal-repro)
#   docker cp "$cid":/app/gallery ./gallery-from-container
#   docker rm "$cid"

FROM python:3.12-slim

# g++ and make for the unit tests and the sim; git is handy for provenance.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        g++ \
        make \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install Python dependencies first so this layer is cached across code edits.
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the repository.
COPY . .

# The Python deps are already installed system-wide in the image, so tell the
# runner to use the system interpreter instead of building a fresh venv.
ENV BIFOCAL_NO_VENV=1

# Default command: run the full host-buildable reproduction.
CMD ["sh", "reproduce.sh"]
