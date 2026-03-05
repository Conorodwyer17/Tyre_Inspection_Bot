# Contributing

Guidelines for contributing to the UGV tyre inspection robot project.

## Development Setup

1. Clone the repository
2. Install dependencies: `pip3 install -r requirements.txt`
3. Install Aurora ROS 2 SDK into `src/`
4. Build: `colcon build --symlink-install`
5. Source: `source install/setup.bash`

See [SETUP.md](SETUP.md) for detailed installation.

## Code Style

- **Python:** 4 spaces, max line length 100, `snake_case` for functions/variables
- **Comments:** British English; explain *why* for non-obvious logic
- **Docstrings:** All public functions and classes
- **Logging:** Use ROS2 logging (`self.get_logger().info/warn/error`), not `print`

## Testing

- Run unit tests: `colcon test --packages-select inspection_manager`
- Run simulation: `ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true`

## Pull Requests

- Describe changes clearly
- Ensure build and simulation pass
- Update documentation if behaviour changes
