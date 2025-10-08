#!/bin/bash

# UWB Localizer ROS2 Build and Run Script

set -e

echo "UWB Localizer ROS2 - Build and Run Script"
echo "=========================================="

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker Compose is installed
if ! command -v docker-compose &> /dev/null; then
    echo "Error: Docker Compose is not installed. Please install Docker Compose first."
    exit 1
fi

# Check if UWB device is connected
if [ ! -e "/dev/ttyUSB0" ]; then
    echo "Warning: UWB device not found at /dev/ttyUSB0"
    echo "Please ensure your UWB device is connected and update the device path if needed."
    echo "You can modify the device path in docker-compose.yml"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  build     Build the Docker image"
    echo "  run       Run the UWB localizer"
    echo "  gui       Run with RViz visualization"
    echo "  record    Run with data recording"
    echo "  stop      Stop all running containers"
    echo "  logs      Show logs from running containers"
    echo "  clean     Remove containers and images"
    echo "  help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 build                    # Build the image"
    echo "  $0 run                      # Run basic UWB localizer"
    echo "  $0 gui                      # Run with RViz"
    echo "  $0 record                   # Run with data recording"
}

# Parse command line arguments
case "${1:-run}" in
    "build")
        echo "Building UWB Localizer ROS2 Docker image..."
        docker-compose build uwb-localizer
        echo "Build completed successfully!"
        ;;
    "run")
        echo "Starting UWB Localizer ROS2..."
        docker-compose up uwb-localizer
        ;;
    "gui")
        echo "Starting UWB Localizer ROS2 with RViz..."
        docker-compose --profile gui up
        ;;
    "record")
        echo "Starting UWB Localizer ROS2 with data recording..."
        docker-compose --profile recording up
        ;;
    "stop")
        echo "Stopping all containers..."
        docker-compose down
        ;;
    "logs")
        echo "Showing logs from running containers..."
        docker-compose logs -f
        ;;
    "clean")
        echo "Cleaning up containers and images..."
        docker-compose down --rmi all --volumes --remove-orphans
        ;;
    "help"|"-h"|"--help")
        show_usage
        ;;
    *)
        echo "Unknown option: $1"
        show_usage
        exit 1
        ;;
esac
