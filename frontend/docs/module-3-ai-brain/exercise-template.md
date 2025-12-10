# Hands-On Exercise Template

This document provides a template for creating hands-on exercises that follow the educational interface contract as defined in the module's contract documentation.

## Exercise Structure

Each hands-on exercise in the module should follow this template:

### Exercise Title
**Estimated Completion Time**: [e.g., 30-60 minutes]

**Learning Objectives**:
- [List 2-3 specific learning objectives the user will achieve]

**Prerequisites**:
- [List any prior knowledge, installed software, or completed sections required]

**Environment Requirements**:
- [Specify hardware and software requirements]

## Exercise Content

### Introduction
Provide context for the exercise and explain what the user will accomplish.

### Step-by-Step Instructions
1. **Step 1**: [Detailed instruction with expected outcome]
   - Code example or command if applicable:
   ```bash
   # Example command
   command --option value
   ```

2. **Step 2**: [Detailed instruction with expected outcome]
   - Code example or command if applicable:
   ```python
   # Example code
   result = function(parameters)
   ```

3. **Continue as needed**...

### Validation Checkpoints
At key points in the exercise, include validation steps:

- **Checkpoint 1**: [How to verify Step 1 was successful]
  - Expected output: [What the user should see]
  - Troubleshooting: [What to do if results don't match]

- **Checkpoint 2**: [How to verify Step 2 was successful]
  - Expected output: [What the user should see]
  - Troubleshooting: [What to do if results don't match]

### Expected Outcome
Describe the final state the user should achieve after completing the exercise.

## Educational Interface Contract Compliance

### 1. Code Example Interface
Each code example in the exercise must specify:
- Environment: `isaac_sim | isaac_ros | ros2 | nav2`
- Dependencies: List any packages or setup required
- Expected execution result: Success or specific output

### 2. Exercise Interface
The exercise must define:
- Exercise ID: Unique identifier for the exercise
- Validation criteria: How to verify completion
- Feedback mechanism: How users know if they succeeded

### 3. Simulation Environment Interface
If the exercise uses simulation:
- Specify the scenario type: `photorealistic | vslam | nav2 | humanoid_navigation`
- Define robot model requirements
- Specify environment parameters
- List required sensors

## Assessment Criteria

Each exercise should include clear assessment criteria:

- **Success**: [What constitutes successful completion]
- **Partial Success**: [What constitutes partial completion]
- **Failure**: [What indicates failure to complete objectives]

## Troubleshooting Section

Include common issues and solutions:

- **Issue 1**: [Description of common problem]
  - **Solution**: [How to fix it]
  - **Prevention**: [How to avoid it in the future]

- **Issue 2**: [Description of common problem]
  - **Solution**: [How to fix it]
  - **Prevention**: [How to avoid it in the future]

## Extension Activities (Optional)

For advanced users, include optional extension activities that build on the main exercise concepts.

---

## Example Exercise Template

### Exercise: Basic Navigation with Isaac Sim and Nav2

**Estimated Completion Time**: 45 minutes

**Learning Objectives**:
- Set up a basic navigation environment in Isaac Sim
- Configure Nav2 for simple goal-based navigation
- Execute a navigation task and observe results

**Prerequisites**:
- Completion of Introduction to Isaac ROS section
- Isaac Sim properly installed and configured
- Basic understanding of ROS 2 concepts

**Environment Requirements**:
- NVIDIA GPU with CUDA support
- Isaac Sim 2023.1 or later
- ROS 2 Humble Hawksbill
- Isaac ROS navigation packages

### Introduction
In this exercise, you'll set up a simple navigation task using Isaac Sim for simulation and Nav2 for path planning and execution.

### Step-by-Step Instructions
1. **Launch Isaac Sim**:
   ```bash
   cd ~/isaac-sim
   ./isaac-sim.py --exec "from omni.isaac.kit import SimulationApp; config = {'headless': False}; simulation_app = SimulationApp(config); import carb; simulation_app.update(); simulation_app.close()"
   ```
   - Expected: Isaac Sim GUI opens and loads default environment

2. **Set up navigation scenario**:
   [Continue with detailed steps...]

### Validation Checkpoints
- **Checkpoint 1**: After launching Isaac Sim
  - Expected output: Simulation environment loads without errors
  - Troubleshooting: If errors occur, verify Isaac Sim installation

### Expected Outcome
By the end of this exercise, you should have successfully navigated a robot from a start position to a goal position using Isaac Sim and Nav2.

### Assessment Criteria
- **Success**: Robot reaches goal position within 10% tolerance
- **Partial Success**: Path planning works, but execution has minor issues
- **Failure**: Robot fails to plan a path or execute navigation

### Troubleshooting Section
- **Issue 1**: Isaac Sim fails to launch
  - **Solution**: Verify GPU drivers and CUDA installation
  - **Prevention**: Check system requirements before installation

[Template continues as needed for each specific exercise]