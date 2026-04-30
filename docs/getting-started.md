# Getting Started

## Installation

### From release binaries

Download the latest binary for your platform from the
[GitHub Releases](https://github.com/mgerhardy/q3modelconverter/releases)
page. Place it somewhere in your `PATH`.

### From .deb / .rpm packages

Debian/Ubuntu and RPM packages are attached to each tagged release.

```bash
# Debian/Ubuntu
sudo dpkg -i modelconverter-*.deb

# Fedora/RocketLinux
sudo rpm -i modelconverter-*.rpm
```

### Building from source

```bash
git clone https://github.com/mgerhardy/q3modelconverter.git
cd q3modelconverter
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
# Optional: install system-wide
sudo cmake --install build
```

The resulting binary is at `build/modelconverter`. No external
dependencies are required beyond a C11 compiler and CMake 3.16+.

## First conversion

```bash
# Convert a Blender export to MD3
modelconverter -i mymodel.glb -o mymodel.md3

# Also emit a .skin and .shader sidecar
modelconverter -i mymodel.glb -o mymodel.md3 --skin --shader
```

## Verifying output

```bash
# Print model info
modelconverter --info -i mymodel.md3

# Run format-limit checks
modelconverter --validate -i mymodel.md3
```

Drop the `.md3` plus its textures and `.skin` file into your game's
`.pk3dir` and verify in-engine with `r_showtris 1`.
