# SPG Documentation Index

Welcome to the **SPG (Setpoint Generator)** framework documentation! This system provides real-time path planning and control for soccer robots with two control modes: traditional MSL and advanced HumanoidMPC.

---

## 📚 Documentation Files

### 🎯 Start Here

**[ARCHITECTURE.md](ARCHITECTURE.md)** - **Comprehensive System Guide**
- Complete system overview
- Control flow explanation
- Coordinate frame transformations
- MSL vs HumanoidMPC comparison
- MPC algorithm mathematics
- Code structure and integration

👉 **Start here if you want to understand how everything works together**

---

### ⚡ Quick Access

**[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - **Practical How-To Guide**
- Switching between MSL and HumanoidMPC modes
- Tuning MPC parameters
- Frame conversion functions
- Common tasks (adding targets, obstacles, etc.)
- Debugging tips
- Performance benchmarks

👉 **Start here if you need to make specific changes quickly**

---

### 📊 Visual Learning

**[VISUAL_GUIDE.md](VISUAL_GUIDE.md)** - **Diagrams & Flowcharts**
- System architecture diagrams
- Control loop visualization
- Coordinate frame illustrations
- MPC horizon visualization
- Cost function breakdown
- Field layout
- Data flow diagrams
- Timing diagrams

👉 **Start here if you learn best from visual representations**

---

### 📝 Historical Context

**[MPC_EXPLANATION.md](MPC_EXPLANATION.md)** - **Original MPC Documentation**
- Initial MPC implementation
- Control parameterization
- Cost function design

**[MPC_TRAJECTORY_IMPROVEMENT.md](MPC_TRAJECTORY_IMPROVEMENT.md)** - **Trajectory Prediction Update**
- Why we moved from constant-velocity extrapolation to full MPC trajectory
- Performance comparison

**[MPC_VELOCITY_BUG.md](MPC_VELOCITY_BUG.md)** - **Critical Bug Fix**
- The velocity continuity bug and its fix
- Importance of measured velocity

**[FRAME_TRANSFORMATION_FIX.md](FRAME_TRANSFORMATION_FIX.md)** - **Frame Handling Update**
- Switch to proper local frame formulation
- Transformation mathematics

**[INTEGRATION_SUMMARY.md](INTEGRATION_SUMMARY.md)** - **Latest Integration**
- Summary of all recent updates
- Current status

---

## 🚀 Quick Start Paths

### Path 1: "I want to understand the big picture"
1. Read [ARCHITECTURE.md](ARCHITECTURE.md) - System Overview section
2. Look at diagrams in [VISUAL_GUIDE.md](VISUAL_GUIDE.md) - System Architecture
3. Read [ARCHITECTURE.md](ARCHITECTURE.md) - Control Flow section

### Path 2: "I need to change something right now"
1. Scan [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Table of contents
2. Jump to the relevant section
3. Make your change

### Path 3: "I need to understand the MPC algorithm"
1. Read [ARCHITECTURE.md](ARCHITECTURE.md) - HumanoidMPC Algorithm Details
2. Look at [VISUAL_GUIDE.md](VISUAL_GUIDE.md) - MPC diagrams
3. Study [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Tuning section
4. Check the code: `src/spg/setpoint/HumanoidMPC.cpp`

### Path 4: "I'm debugging a problem"
1. Check [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Common Errors section
2. Enable debug output (instructions in Quick Reference)
3. Review [ARCHITECTURE.md](ARCHITECTURE.md) - Coordinate Frames section
4. Check historical bug fixes if relevant

---

## 📖 Document Summaries

| Document | Purpose | Audience | Reading Time |
|----------|---------|----------|--------------|
| **ARCHITECTURE.md** | Complete technical documentation | Engineers, developers | 30-45 min |
| **QUICK_REFERENCE.md** | Practical how-to guide | Users, developers | 10-15 min |
| **VISUAL_GUIDE.md** | Visual diagrams and flowcharts | Visual learners | 15-20 min |
| MPC_EXPLANATION.md | Original MPC design | Advanced users | 10 min |
| MPC_TRAJECTORY_IMPROVEMENT.md | Trajectory update | Context only | 5 min |
| MPC_VELOCITY_BUG.md | Velocity bug fix | Context only | 5 min |
| FRAME_TRANSFORMATION_FIX.md | Frame handling fix | Context only | 5 min |
| INTEGRATION_SUMMARY.md | Latest status | Quick overview | 5 min |

---

## 🎓 Learning Objectives

After reading these docs, you should be able to:

✅ **Understand** how SPG generates control commands from high-level goals

✅ **Explain** the difference between MSL mode and HumanoidMPC mode

✅ **Convert** between global and robot local coordinate frames

✅ **Tune** MPC parameters to change robot behavior

✅ **Debug** common issues with control and trajectory generation

✅ **Modify** the code to add new features or change behavior

✅ **Optimize** performance for your specific robot platform

---

## 🔧 Key Concepts to Master

### 1. Coordinate Frames ⭐⭐⭐
- Global (world) frame vs Robot local (body) frame
- When and how to transform between them
- **Docs**: ARCHITECTURE.md → Coordinate Frames, VISUAL_GUIDE.md → Frame diagrams

### 2. Control Modes ⭐⭐
- MSL: Segment-based, fast, proven
- HumanoidMPC: Optimal, smooth, tunable
- **Docs**: ARCHITECTURE.md → MSL vs HumanoidMPC, QUICK_REFERENCE.md → Mode switching

### 3. MPC Algorithm ⭐⭐⭐
- Cost function components (tracking, effort, smoothness)
- QP formulation and solving
- Receding horizon principle
- **Docs**: ARCHITECTURE.md → MPC Algorithm Details, VISUAL_GUIDE.md → MPC diagrams

### 4. Data Flow ⭐⭐
- SPG state structure
- Target → Subtarget → Setpoint pipeline
- Output: position, velocity, acceleration
- **Docs**: ARCHITECTURE.md → Control Flow, VISUAL_GUIDE.md → Data Flow Diagram

### 5. Parameter Tuning ⭐
- 10 BO-ready weight parameters
- How each parameter affects behavior
- Tuning guidelines for different scenarios
- **Docs**: QUICK_REFERENCE.md → Tuning section, ARCHITECTURE.md → Cost Function

---

## 📞 Common Questions

### Q: Which mode should I use, MSL or HumanoidMPC?

**A:** 
- **MSL** if you need: Fast computation, proven reliability, wheeled robots
- **HumanoidMPC** if you need: Optimal control, smooth motion, easy tuning

See: ARCHITECTURE.md → MSL Mode vs HumanoidMPC Mode

---

### Q: How do I switch between modes?

**A:** Set `d.par.use_humanoid_mpc = true` (MPC) or `false` (MSL)

See: QUICK_REFERENCE.md → Switching Between Modes

---

### Q: Why does my robot behave differently than expected?

**A:** Check:
1. Are coordinates in the correct frame? (global vs local)
2. Are MPC weights tuned properly?
3. Are physical limits set correctly?

See: QUICK_REFERENCE.md → Debugging Tips

---

### Q: How can I make the robot move faster/smoother/more aggressive?

**A:** Adjust MPC cost weights:
- Faster: decrease `r_vf`, increase `qf_pos`
- Smoother: increase `s_vf`, `s_vs`, `s_omega`
- More aggressive: decrease smoothness weights

See: QUICK_REFERENCE.md → Tuning Guidelines

---

### Q: What does "robot local frame" mean?

**A:** A coordinate system attached to the robot where:
- Origin = robot's current position
- X-axis = robot's forward direction
- Y-axis = robot's left direction

The MPC solver works in this frame, then transforms back to global frame.

See: ARCHITECTURE.md → Coordinate Frames, VISUAL_GUIDE.md → Frame Transformation Visual

---

### Q: Why is MPC slower than MSL mode?

**A:** MPC solves an optimization problem (QP) with 30 variables, which takes ~1-3ms. MSL just evaluates analytic formulas (~200μs). Both are fast enough for 50 Hz control (20ms budget).

See: QUICK_REFERENCE.md → Performance Benchmarks

---

### Q: Can I visualize what the controller is doing?

**A:** Yes! Run `./demo_humanoid_mpc` to see:
- Robot position and heading
- Target and subtarget
- Predicted trajectory (cyan line)
- Obstacles
- Real-time control updates

See: ARCHITECTURE.md → Visualization System

---

### Q: What's the "measured velocity" and why does it matter?

**A:** The robot's current velocity (from odometry/IMU). MPC uses this to:
1. Ensure smooth transitions (no sudden velocity changes)
2. Calculate correct Δu penalty (control rate of change)

Without it, MPC would assume the robot starts from rest every timestep!

See: MPC_VELOCITY_BUG.md, ARCHITECTURE.md → Cost Function

---

## 🛠️ Code Locations

Quick reference for key code files:

```
src/spg/setpoint/Set.cpp              ← Mode selection & integration
src/spg/setpoint/HumanoidMPC.cpp      ← MPC algorithm implementation
include/spg/setpoint/HumanoidMPC.hpp  ← MPC interface
src/spg/target/Target.cpp             ← Target adjustment logic
src/spg/subtarget/Subtarget.cpp       ← Subtarget calculation
demo/main.cpp                         ← Demo with visualization
test/test_humanoid_mpc.cpp            ← Unit tests
```

---

## 🎯 Next Steps

1. **Choose your learning path** above
2. **Read the recommended docs** for your path
3. **Run the demo**: `./demo_humanoid_mpc`
4. **Try modifying** parameters in `Set.cpp`
5. **Read the code** with docs as reference

---

## 📧 Need Help?

If these docs don't answer your question:
1. Check the code comments in the relevant files
2. Look at the unit tests for examples
3. Review git history for context on specific changes

---

**Version**: December 9, 2025  
**Framework**: SPG with HumanoidMPC (Local Frame Implementation)  
**Documentation Status**: ✅ Complete

---

## 📋 Quick Navigation

| I want to... | Read this... |
|--------------|-------------|
| Understand the system | [ARCHITECTURE.md](ARCHITECTURE.md) |
| Make a quick change | [QUICK_REFERENCE.md](QUICK_REFERENCE.md) |
| See diagrams | [VISUAL_GUIDE.md](VISUAL_GUIDE.md) |
| Learn about MPC | [ARCHITECTURE.md](ARCHITECTURE.md) → MPC section |
| Fix a bug | [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → Debugging |
| Tune parameters | [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → Tuning |
| Understand frames | [ARCHITECTURE.md](ARCHITECTURE.md) → Coordinate Frames |
| See examples | demo/main.cpp, test/test_humanoid_mpc.cpp |

---

Happy coding! 🤖⚽
