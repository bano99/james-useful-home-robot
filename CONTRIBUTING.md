# Contributing to James Home Robot

Thank you for your interest in contributing to James! This document provides guidelines and instructions for contributing to the project.

## Code of Conduct

This project adheres to the Contributor Covenant [Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code.

## How to Contribute

### Reporting Bugs

Before creating bug reports, please check the existing issues to avoid duplicates. When creating a bug report, include:

- **Clear title and description**
- **Steps to reproduce** the issue
- **Expected behavior** vs actual behavior
- **Environment details** (OS, ROS2 version, hardware)
- **Logs and error messages**
- **Screenshots or videos** if applicable

### Suggesting Enhancements

Enhancement suggestions are tracked as GitHub issues. When creating an enhancement suggestion, include:

- **Clear title and description**
- **Use case** and motivation
- **Proposed solution** or implementation approach
- **Alternative solutions** considered
- **Impact** on existing functionality

### Pull Requests

1. **Fork the repository** and create your branch from `develop`
2. **Follow the coding standards** (see below)
3. **Write clear commit messages** following conventional commits
4. **Add tests** for new functionality
5. **Update documentation** as needed
6. **Ensure CI passes** before requesting review
7. **Request review** from maintainers

#### Branch Naming Convention

- `feature/description` - New features
- `fix/description` - Bug fixes
- `docs/description` - Documentation updates
- `refactor/description` - Code refactoring
- `test/description` - Test additions or updates

#### Commit Message Format

```
<type>(<scope>): <subject>

<body>

<footer>
```

Types: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

Example:
```
feat(navigation): add dynamic obstacle avoidance

Implement real-time obstacle detection and path replanning
using depth camera data from RealSense D435.

Closes #123
```

## Development Setup

### Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.10+
- Git

### Local Development

1. Clone your fork:
```bash
git clone https://github.com/yourusername/james-useful-home-robot.git
cd james-useful-home-robot
```

2. Create a development branch:
```bash
git checkout -b feature/your-feature-name
```

3. Set up the development environment:
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

4. Make your changes and test:
```bash
colcon test
pytest tests/
```

5. Commit and push:
```bash
git add .
git commit -m "feat(scope): description"
git push origin feature/your-feature-name
```

6. Create a pull request on GitHub

## Coding Standards

### Python

- Follow [PEP 8](https://pep8.org/) style guide
- Use [Black](https://black.readthedocs.io/) for formatting (line length: 100)
- Use type hints for function signatures
- Write docstrings for all public functions and classes
- Maximum function length: 50 lines
- Maximum file length: 500 lines

Example:
```python
def calculate_distance(point_a: Point3D, point_b: Point3D) -> float:
    """
    Calculate Euclidean distance between two 3D points.
    
    Args:
        point_a: First point
        point_b: Second point
        
    Returns:
        Distance in meters
    """
    dx = point_b.x - point_a.x
    dy = point_b.y - point_a.y
    dz = point_b.z - point_a.z
    return math.sqrt(dx**2 + dy**2 + dz**2)
```

### C++ (ROS2 Nodes)

- Follow [ROS2 C++ Style Guide](https://docs.ros.org/en/jazzy/Contributing/Code-Style-Language-Versions.html)
- Use `clang-format` with ROS2 configuration
- Use smart pointers (avoid raw pointers)
- RAII for resource management
- Const correctness

### Arduino/ESP32

- Follow [Arduino Style Guide](https://www.arduino.cc/en/Reference/StyleGuide)
- Use meaningful variable names
- Comment complex logic
- Keep functions focused and small
- Use `const` for constants

## Testing Guidelines

### Unit Tests

- Write unit tests for all new functions
- Aim for >80% code coverage
- Use pytest for Python tests
- Use gtest for C++ tests
- Mock external dependencies

### Integration Tests

- Test interactions between components
- Use ROS2 bag files for reproducible tests
- Test error handling and edge cases

### Hardware Tests

- Document hardware test procedures
- Include expected results
- Test on actual hardware before merging

## Documentation

### Code Documentation

- Document all public APIs
- Include usage examples
- Document assumptions and limitations
- Keep documentation up-to-date with code changes

### User Documentation

- Update README.md for user-facing changes
- Add tutorials for new features
- Include troubleshooting tips
- Provide configuration examples

### Architecture Documentation

- Update architecture diagrams for structural changes
- Document design decisions
- Explain trade-offs and alternatives considered

## Review Process

1. **Automated checks** must pass (CI/CD pipeline)
2. **Code review** by at least one maintainer
3. **Testing** on hardware (for hardware-related changes)
4. **Documentation review** for completeness
5. **Approval** and merge by maintainer

### Review Checklist

- [ ] Code follows style guidelines
- [ ] Tests are included and passing
- [ ] Documentation is updated
- [ ] No breaking changes (or properly documented)
- [ ] Commit messages are clear
- [ ] Branch is up-to-date with develop

## Release Process

1. Version bump following [Semantic Versioning](https://semver.org/)
2. Update CHANGELOG.md
3. Create release branch
4. Final testing and validation
5. Merge to main
6. Tag release
7. Deploy documentation

## Getting Help

- **GitHub Discussions**: For questions and general discussion
- **GitHub Issues**: For bug reports and feature requests
- **Discord**: [Join our community](https://discord.gg/james-robot) (if available)

## Recognition

Contributors will be recognized in:
- CONTRIBUTORS.md file
- Release notes
- Project website (if applicable)

Thank you for contributing to James! 🤖
