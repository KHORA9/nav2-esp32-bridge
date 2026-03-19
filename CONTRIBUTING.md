# Contributing to nav2-esp32-bridge

Thank you for your interest in contributing!

## How to Contribute

### Reporting Bugs

Open a GitHub Issue with:
- Clear title describing the problem
- Steps to reproduce
- Expected vs actual behavior
- ROS2 version, ESP32 board, and firmware version

### Submitting Changes

1. **Fork the repository**
2. **Create a feature branch:** `git checkout -b feature/my-feature`
3. **Make your changes** — follow the code style below
4. **Write tests** if adding functionality
5. **Commit** with a clear message: `git commit -m "Add X for Y"`
6. **Push:** `git push origin feature/my-feature`
7. **Open a Pull Request**

### Code Style

**C++ (firmware):**
- 4-space indentation, no tabs
- `static const` for compile-time constants
- `// Section comments` for file organization
- Prefix: `g_` for globals, `m_` for class members
- Braces on same line: `if (x) { ... }`

**Python (ROS2 launch files):**
- PEP 8 compliant
- 4-space indentation
- `LaunchConfiguration` for launch args
- Use `DeclareLaunchArgument` for all parameters

**ROS2 packages:**
- Follow ROS2 style guide
- Use `ament_cmake` for C++ packages
- Always define `package.xml` with all dependencies

### Commit Message Format

```
type: short description

Detailed explanation (if needed).

Fixes: #issue-number
```

Types: `feat`, `fix`, `docs`, `test`, `refactor`, `chore`

### Testing

```bash
# Build firmware
cd firmware && pio run -e esp32-s3

# Build ROS2 packages
cd ros2
colcon build --packages-select hanatra_msgs hanatra_control
colcon test --packages-select hanatra_control
```

## Code of Conduct

Be respectful. Harassment will not be tolerated.

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
