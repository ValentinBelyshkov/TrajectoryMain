#!/bin/bash
# TerraSLAM Docker Build Script
# This script helps build the TerraSLAM Docker image for different platforms

set -e

# Default to current platform
PLATFORM=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --platform)
            PLATFORM="$2"
            shift 2
            ;;
        --arm64)
            PLATFORM="linux/arm64"
            shift
            ;;
        --amd64)
            PLATFORM="linux/amd64"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Build TerraSLAM Docker image"
            echo ""
            echo "Options:"
            echo "  --platform PLATFORM  Specify platform (e.g., linux/amd64, linux/arm64)"
            echo "  --arm64              Build for ARM64 (Jetson Orin)"
            echo "  --amd64              Build for AMD64 (x86_64)"
            echo "  -h, --help           Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                  Build for current platform"
            echo "  $0 --amd64           Build for AMD64/x86_64"
            echo "  $0 --arm64           Build for ARM64"
            echo "  $0 --platform linux/arm64  Build for specific platform"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Detect current platform if not specified
if [ -z "$PLATFORM" ]; then
    ARCH=$(uname -m)
    case $ARCH in
        x86_64)
            PLATFORM="linux/amd64"
            ;;
        aarch64|arm64)
            PLATFORM="linux/arm64"
            ;;
        *)
            echo "Unknown architecture: $ARCH"
            echo "Please specify platform explicitly using --platform"
            exit 1
            ;;
    esac
fi

echo "Building TerraSLAM Docker image for platform: $PLATFORM"

# Build arguments
BUILD_ARGS=""
if [ "$PLATFORM" = "linux/arm64" ]; then
    BUILD_ARGS="--build-arg BUILDPLATFORM=linux/arm64 --build-arg TARGETPLATFORM=linux/arm64"
else
    BUILD_ARGS="--build-arg BUILDPLATFORM=linux/amd64 --build-arg TARGETPLATFORM=linux/amd64"
fi

# Build the image
docker buildx build \
    --platform "$PLATFORM" \
    $BUILD_ARGS \
    -t orb-slam3-humble:22.04 \
    .

echo "Build completed successfully!"
echo "Image tag: orb-slam3-humble:22.04"
