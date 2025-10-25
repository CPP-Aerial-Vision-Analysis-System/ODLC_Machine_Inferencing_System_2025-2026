#!/bin/bash
set -e

OS=$(uname -s)
echo "Detected OS: $OS"

case "$OS" in
  Linux*)
    echo " Linux detected – enabling host networking & display support"
    # For Linux we just print hints or set environment vars
    echo "DISPLAY=:0" >> ~/.bashrc
    echo "export DISPLAY=:0" >> ~/.bashrc
    ;;
  Darwin*)
    echo " macOS detected – skipping network/display changes"
    ;;
  MINGW*|CYGWIN*|MSYS*|Windows*)
    echo " Windows/WSL detected – using defaults"
    ;;
  *)
    echo "Unknown OS – using safe defaults"
    ;;
esac

