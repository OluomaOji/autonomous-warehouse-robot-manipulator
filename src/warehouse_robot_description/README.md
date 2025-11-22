# warehouse_robot_description

Robot description package containing URDF models, meshes, and visualization configurations for the autonomous warehouse robot.

## Package Contents

- **urdf/**: Robot description files (URDF/Xacro format)
  - `robot.urdf.xacro`: Main robot assembly file
  - `mobile_base.urdf.xacro`: Differential drive mobile base
  
- **launch/**: Launch files for visualization
  - `display.launch.py`: Launch RViz with robot model
  - `description.launch.py`: Publish robot description
  
- **rviz/**: RViz configuration files
  - `robot_view.rviz`: Default visualization configuration

- **config/**: Configuration files (joint limits, etc.)

- **meshes/**: 3D mesh files for realistic visualization

## Usage

### View robot in RViz:
```bash
ros2 launch warehouse_robot_description display.launch.py
```

## Robot Specifications

### Mobile Base
- **Type:** Differential drive
- **Dimensions:** 0.6m (L) x 0.4m (W) x 0.2m (H)
- **Wheel Radius:** 0.1m
- **Mass:** 5kg

### Wheels
- **Type:** Two powered wheels + one caster
- **Powered Wheel Diameter:** 0.2m (0.1m radius)
- **Caster Diameter:** 0.1m (0.05m radius)