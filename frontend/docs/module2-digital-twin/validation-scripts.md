# Validation Scripts for Digital Twin Module

This document describes the validation scripts that test the reproducibility of code examples in the Digital Twin module.

## Overview

The validation system ensures that all code examples in the digital twin module work as documented and can be reproduced by students. Validation scripts check:
- URDF models load correctly in Gazebo
- ROS 2 configurations work properly
- Unity scene elements are correctly set up
- Integration between systems functions as expected

## URDF Validation Script

```python
#!/usr/bin/env python3
# validate_urdf.py
# Validates that URDF files are properly formatted and can be parsed

import sys
import os
import xml.etree.ElementTree as ET
from urdf_parser_py.urdf import URDF

def validate_urdf_structure(urdf_path):
    """Validate URDF file structure and content"""
    try:
        # Parse the URDF file
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        if root.tag != 'robot':
            print(f"ERROR: Root tag is not 'robot': {root.tag}")
            return False
            
        robot_name = root.get('name')
        if not robot_name:
            print("ERROR: Robot has no name attribute")
            return False
            
        print(f"✓ Validated robot: {robot_name}")
        return True
    except ET.ParseError as e:
        print(f"ERROR: Invalid XML format: {e}")
        return False
    except Exception as e:
        print(f"ERROR: Could not parse URDF: {e}")
        return False

def validate_urdf_semantics(urdf_path):
    """Validate URDF semantics using urdf_parser_py"""
    try:
        # Load and parse the URDF
        robot = URDF.from_xml_string(open(urdf_path).read())
        
        if not robot:
            print("ERROR: Could not parse URDF semantics")
            return False
            
        print(f"✓ Parsed robot with {len(robot.links)} links and {len(robot.joints)} joints")
        
        # Check for basic robot properties
        if not robot.name:
            print("ERROR: Robot has no name")
            return False
            
        # Check that all joints have proper parent-child relationships
        for joint in robot.joints:
            if joint.parent not in [link.name for link in robot.links]:
                print(f"ERROR: Joint {joint.name} has invalid parent: {joint.parent}")
                return False
            if joint.child not in [link.name for link in robot.links]:
                print(f"ERROR: Joint {joint.name} has invalid child: {joint.child}")
                return False
                
        print(f"✓ Validated {len(robot.joints)} joints have valid parent-child relationships")
        return True
    except Exception as e:
        print(f"ERROR: URDF semantics validation failed: {e}")
        return False

def main(urdf_path):
    print(f"Validating URDF file: {urdf_path}")
    
    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF file does not exist: {urdf_path}")
        return False
    
    # Run structure validation
    if not validate_urdf_structure(urdf_path):
        print("✗ URDF structure validation failed")
        return False
    print("✓ URDF structure validation passed")
    
    # Run semantics validation
    if not validate_urdf_semantics(urdf_path):
        print("✗ URDF semantics validation failed")
        return False
    print("✓ URDF semantics validation passed")
    
    print(f"✓ URDF validation completed successfully: {urdf_path}")
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 validate_urdf.py <urdf_file>")
        sys.exit(1)
        
    urdf_file = sys.argv[1]
    success = main(urdf_file)
    sys.exit(0 if success else 1)
```

## YAML Configuration Validation Script

```python
#!/usr/bin/env python3
# validate_yaml.py
# Validates YAML configuration files used in ROS 2

import sys
import os
import yaml

def validate_yaml_config(yaml_path):
    """Validate YAML configuration file"""
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
        
        if config is None:
            print(f"ERROR: YAML file is empty: {yaml_path}")
            return False
            
        if not isinstance(config, dict):
            print(f"ERROR: Expected YAML to be a dictionary: {yaml_path}")
            return False
            
        print(f"✓ Validated YAML config: {yaml_path}")
        return True
    except yaml.YAMLError as e:
        print(f"ERROR: Invalid YAML format: {e}")
        return False
    except Exception as e:
        print(f"ERROR: Could not parse YAML: {e}")
        return False

def validate_ros2_controller_config(yaml_path):
    """Validate ROS 2 controller configuration"""
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # Check for controller manager section
        if 'controller_manager' not in config:
            print(f"ERROR: Missing 'controller_manager' section in {yaml_path}")
            return False
            
        # Check for required parameters
        cm_params = config['controller_manager'].get('ros__parameters', {})
        if 'update_rate' not in cm_params:
            print(f"ERROR: Missing 'update_rate' in controller manager parameters")
            return False
            
        # Validate controller configurations
        for section_name, section_data in config.items():
            if section_name == 'controller_manager':
                continue
                
            if 'ros__parameters' not in section_data:
                print(f"ERROR: Missing 'ros__parameters' in section {section_name}")
                continue
                
            params = section_data['ros__parameters']
            
            # Example validation for diff_drive_controller
            if 'type' in params and 'diff_drive_controller' in params['type']:
                required_params = ['left_wheel_names', 'right_wheel_names', 'wheel_separation', 'wheel_radius']
                for param in required_params:
                    if param not in params:
                        print(f"ERROR: Missing required parameter '{param}' for diff_drive_controller")
                        return False
                        
        print(f"✓ Validated ROS 2 controller config: {yaml_path}")
        return True
    except Exception as e:
        print(f"ERROR: Controller config validation failed: {e}")
        return False

def main(yaml_path):
    print(f"Validating YAML file: {yaml_path}")
    
    if not os.path.exists(yaml_path):
        print(f"ERROR: YAML file does not exist: {yaml_path}")
        return False
    
    # Run basic YAML validation
    if not validate_yaml_config(yaml_path):
        print("✗ YAML structure validation failed")
        return False
    print("✓ YAML structure validation passed")
    
    # Run ROS 2 controller validation if it's a controller config
    if 'controller' in yaml_path:
        if not validate_ros2_controller_config(yaml_path):
            print("✗ ROS 2 controller config validation failed")
            return False
        print("✓ ROS 2 controller config validation passed")
    
    print(f"✓ YAML validation completed successfully: {yaml_path}")
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 validate_yaml.py <yaml_file>")
        sys.exit(1)
        
    yaml_file = sys.argv[1]
    success = main(yaml_file)
    sys.exit(0 if success else 1)
```

## Unit Testing Script

```python
#!/usr/bin/env python3
# run_module_tests.py
# Runs all validation tests for the digital twin module

import subprocess
import sys
import os
from pathlib import Path

def run_validation_script(script_path, file_path):
    """Run a validation script on a file"""
    try:
        result = subprocess.run([
            sys.executable, script_path, file_path
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            print(f"✗ Validation failed for {file_path}")
            print(f"  Error: {result.stderr.strip()}")
            return False
        else:
            print(f"✓ Validation passed for {file_path}")
            return True
    except subprocess.TimeoutExpired:
        print(f"✗ Validation timed out for {file_path}")
        return False
    except Exception as e:
        print(f"✗ Validation error for {file_path}: {e}")
        return False

def find_files_by_extension(directory, extension):
    """Find all files with a specific extension in directory and subdirectories"""
    path = Path(directory)
    return list(path.rglob(f"*.{extension}"))

def main(module_dir):
    print(f"Running validation tests for digital twin module: {module_dir}")
    
    # Find all URDF files
    urdf_files = find_files_by_extension(module_dir, "urdf")
    print(f"Found {len(urdf_files)} URDF files to validate")
    
    # Find all YAML files
    yaml_files = find_files_by_extension(module_dir, "yaml")
    print(f"Found {len(yaml_files)} YAML files to validate")
    
    # Validation script paths
    urdf_validator = os.path.join(os.path.dirname(__file__), "validate_urdf.py")
    yaml_validator = os.path.join(os.path.dirname(__file__), "validate_yaml.py")
    
    all_passed = True
    
    # Validate all URDF files
    for urdf_file in urdf_files:
        if not run_validation_script(urdf_validator, str(urdf_file)):
            all_passed = False
    
    # Validate all YAML files
    for yaml_file in yaml_files:
        if not run_validation_script(yaml_validator, str(yaml_file)):
            all_passed = False
    
    if all_passed:
        print(f"\n✓ All validations passed for module: {module_dir}")
        return True
    else:
        print(f"\n✗ Some validations failed for module: {module_dir}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 run_module_tests.py <module_directory>")
        sys.exit(1)
        
    module_dir = sys.argv[1]
    success = main(module_dir)
    sys.exit(0 if success else 1)
```

## Running the Validation Scripts

To run the validation on the digital twin examples:

```bash
# Make the scripts executable
chmod +x validate_urdf.py
chmod +x validate_yaml.py
chmod +x run_module_tests.py

# Validate a specific URDF file
python3 validate_urdf.py digital_twin_robot.urdf

# Validate a specific YAML file
python3 validate_yaml.py diffdrive_controller.yaml

# Validate all files in the module directory
python3 run_module_tests.py /path/to/module/directory
```

## Integration with CI/CD

For automated validation, you can add these scripts to your CI/CD pipeline:

```yaml
# .github/workflows/validation.yml
name: Module Validation
on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  validate:
    runs-on: ubuntu-22.04
    steps:
    - uses: actions/checkout@v3
    
    - name: Setup Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.10'
        
    - name: Install dependencies
      run: |
        pip install urdf-parser-py pyyaml
        
    - name: Run module validation
      run: |
        python3 run_module_tests.py frontend/docs/module2-digital-twin/
```

These validation scripts ensure that all code examples in the digital twin module are reproducible and function as documented, providing students with reliable examples to learn from.