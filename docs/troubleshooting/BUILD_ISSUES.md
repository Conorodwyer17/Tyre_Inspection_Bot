# Build Issues Troubleshooting

## Common Build Errors

### Symbolic Link Error

**Error:**
```
failed to create symbolic link '.../segmentation_msgs/segmentation_msgs' because existing path cannot be removed: Is a directory
```

**Cause:** Build directory contains a directory where a symlink should be created.

**Fix:**
```bash
# Remove build and install directories for the package
rm -rf build/segmentation_msgs install/segmentation_msgs

# Rebuild the package
colcon build --packages-select segmentation_msgs
```

**Prevention:** Always clean build directories when switching branches or after major changes.

---

### Package Override Warning

**Warning:**
```
Some selected packages are already built in one or more underlay workspaces:
        'cartographer' is in: /home/jetson/ugv_ws/install/cartographer
```

**Fix:**
```bash
# Add --allow-overriding flag
colcon build --allow-overriding cartographer

# Or clean and rebuild
rm -rf build/cartographer install/cartographer
colcon build
```

---

### Missing Dependencies

**Error:** Package not found or missing dependencies

**Fix:**
```bash
# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build
```

---

### Python Import Errors

**Error:** Module not found or import errors

**Fix:**
```bash
# Source workspace
source install/setup.bash

# Rebuild package
colcon build --packages-select <package_name>

# Verify Python path
echo $PYTHONPATH
```

---

## Clean Build

### Complete Clean
```bash
# Remove all build artifacts
rm -rf build/ install/ log/

# Rebuild everything
colcon build
```

### Package-Specific Clean
```bash
# Remove specific package
rm -rf build/<package_name> install/<package_name>

# Rebuild
colcon build --packages-select <package_name>
```

---

## Build Best Practices

1. **Clean Before Major Changes** - Remove build/install when switching branches
2. **Incremental Builds** - Use `--packages-select` for single packages
3. **Check Dependencies** - Run `rosdep install` after adding packages
4. **Source Workspace** - Always source `install/setup.bash` after building

---

**Last Updated:** Current Session  
**Version:** 1.0
