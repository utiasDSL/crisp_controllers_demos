#!/bin/bash

# Script to setup ROS 2 middleware based on RMW environment variable
# Usage: source scripts/setup_middleware.sh

set -e

echo "Setting up ROS 2 middleware with '$RMW'..."

case "$RMW" in
    "cyclone")
        echo "Setting up CycloneDDS middleware..."
        source src/crisp_controllers_demos/scripts/set_cyclone_config.sh
        ;;
    "zenoh")
        echo "Setting up Zenoh middleware..."
        source src/crisp_controllers_demos/scripts/set_zenoh_config.sh
        ;;
    "fastdds"|"fast"|"")
        echo "Using default FastDDS middleware"
        ;;
    *)
        echo "Error: Unknown RMW option '$RMW'. Supported options: cyclone, zenoh, fastdds"
        exit 1
        ;;
esac
