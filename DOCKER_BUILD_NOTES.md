# Docker Build Platform Fix

## Problem
The Docker build was failing with the error:
```
ERROR: failed to build: failed to solve: osrf/ros:humble-desktop-full-jammy: failed to resolve source metadata for docker.io/osrf/ros:humble-desktop-full-jammy: no match for platform in manifest: not found
```

This error occurs when Docker cannot resolve the correct platform manifest for the base image, typically due to:
- Platform mismatch between build host and target architecture
- Docker buildx not properly configured for multi-platform builds
- Missing platform specification in the Dockerfile

## Solution

### 1. Updated Dockerfile
The Dockerfile now includes explicit platform arguments:

```dockerfile
ARG BUILDPLATFORM=linux/amd64
ARG TARGETPLATFORM=linux/amd64
FROM --platform=$BUILDPLATFORM osrf/ros:humble-desktop-full-jammy
```

This allows Docker to pull the correct image manifest for the target platform.

### 2. Build Script (`build_docker.sh`)
A new convenience script has been added to simplify platform-specific builds:

```bash
./build_docker.sh              # Auto-detect platform
./build_docker.sh --amd64      # Build for x86_64
./build_docker.sh --arm64      # Build for ARM64 (Jetson Orin)
./build_docker.sh --help       # Show all options
```

### 3. Documentation Updates
- README.md: Updated with build script usage and platform-specific instructions
- Troubleshooting section: Added common Docker build issues and solutions
- .dockerignore: Added to optimize build context

## Usage

### Quick Start
For most users, simply run:
```bash
./build_docker.sh
```

The script will auto-detect your architecture and build accordingly.

### Manual Build with Platform
If you need to specify a platform manually:
```bash
# For x86_64 (Intel/AMD)
docker buildx build --platform linux/amd64 -t orb-slam3-humble:22.04 .

# For ARM64 (Jetson Orin)
docker buildx build --platform linux/arm64 -t orb-slam3-humble:22.04 .
```

### Cross-Compilation
To build for ARM64 on an x86_64 machine:
```bash
# Install QEMU for cross-platform emulation
docker run --privileged --rm tonistiigi/binfmt --install all

# Build for ARM64
./build_docker.sh --arm64
```

## Platform Support

| Platform | Architecture | Use Case |
|----------|-------------|----------|
| linux/amd64 | x86_64 | Intel/AMD desktops and servers |
| linux/arm64 | ARM64 | NVIDIA Jetson Orin, Raspberry Pi 4+ |

## Troubleshooting

### "no match for platform in manifest" error
This indicates a platform mismatch. Solution:
```bash
./build_docker.sh --amd64   # or --arm64
```

### Docker buildx not available
Enable Docker buildx:
```bash
docker buildx create --use
```

### Cross-compilation fails
Install QEMU support:
```bash
docker run --privileged --rm tonistiigi/binfmt --install all
```

## Files Changed
1. **Dockerfile** - Added platform arguments and improved architecture detection
2. **build_docker.sh** - New build script for platform-aware builds
3. **README.md** - Updated build instructions and added troubleshooting section
4. **.dockerignore** - New file to optimize Docker build context
