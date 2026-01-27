# SPG HumanoidMPC - Documentation Index

Welcome to the complete documentation for the HumanoidMPC integration with the SPG (Soccer Playing Goalkeeper) framework.

---

## 📚 Documentation Structure

### **Start Here**
- **[INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)** ⭐
  - Complete summary of what was done
  - Quick start guide
  - Test results and performance metrics
  - Troubleshooting tips

### **Understanding the System**
1. **[ARCHITECTURE.md](ARCHITECTURE.md)** - Deep dive into system architecture
   - Complete control flow
   - Component interactions
   - Frame transformations
   - MPC algorithm details
   
2. **[VISUAL_GUIDE.md](VISUAL_GUIDE.md)** - Flow diagrams and visualizations
   - Visual control flow
   - Frame transformation diagrams
   - Data flow illustrations
   - Mode switching logic

3. **[README.md](README.md)** - General project README
   - Project overview
   - Build instructions
   - Usage examples

### **Quick Reference**
- **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - API quick reference
  - Code snippets
  - Common patterns
  - Mode switching
  - Parameter settings

### **Benchmarking & Scripts**
- **[../scripts/README.md](../scripts/README.md)** - Benchmarking guide
  - How to run benchmarks
  - Script usage
  - Configuration options
  - Result analysis

---

## 🎯 Common Tasks

### I want to...

#### **...understand how everything works**
→ Start with [VISUAL_GUIDE.md](VISUAL_GUIDE.md), then read [ARCHITECTURE.md](ARCHITECTURE.md)

#### **...use HumanoidMPC in my code**
→ Check [QUICK_REFERENCE.md](QUICK_REFERENCE.md) for code examples

#### **...run benchmarks**
→ See [../scripts/README.md](../scripts/README.md)

#### **...see test results and performance**
→ Check [INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)

#### **...modify MPC parameters**
→ Edit `src/spg/setpoint/Set.cpp` (weights section) or use benchmark scripts

#### **...debug issues**
→ See troubleshooting in [INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)

---

## 📁 File Organization

```
spg_cmake/
├── docs/                           # Documentation
│   ├── INDEX.md                   # This file
│   ├── INTEGRATION_COMPLETE.md    # ⭐ Start here
│   ├── ARCHITECTURE.md            # System architecture
│   ├── VISUAL_GUIDE.md            # Diagrams and flows
│   ├── QUICK_REFERENCE.md         # API reference
│   └── README.md                  # Project README
│
├── scripts/                        # Benchmarking scripts
│   ├── README.md                  # Benchmarking guide
│   ├── run_mpc_benchmarks.py      # Simple runner
│   ├── batch_mpc_benchmark.py     # Advanced runner
│   └── default_mpc_weights.json   # Default config
│
├── include/spg/setpoint/
│   └── HumanoidMPC.hpp            # MPC interface
│
├── src/spg/setpoint/
│   ├── HumanoidMPC.cpp            # MPC implementation
│   └── Set.cpp                    # Integration point
│
├── test/
│   └── test_humanoid_mpc.cpp      # Unit tests
│
└── demo/
    └── demo_humanoid_mpc.cpp      # Interactive demo
```

---

## 🚀 Quick Start Guide

### 1. Build the Project
```bash
cd /home/robocup/Downloads/spg_cmake/build
cmake ..
make -j4
```

### 2. Run Tests
```bash
./test/test_humanoid_mpc
```
**Expected**: All 6 tests pass ✅

### 3. Run Interactive Demo
```bash
./demo_humanoid_mpc
```
**Controls**:
- `M` - Toggle between MSL and HumanoidMPC modes
- `R` - Reset simulation
- `Click` - Set new target
- `ESC` - Exit

### 4. Run Benchmarks
```bash
cd ../scripts
python3 run_mpc_benchmarks.py --runs 10
```

---

## 📖 Documentation Features

### For Different Audiences

#### **Researchers/Scientists**
- Read [ARCHITECTURE.md](ARCHITECTURE.md) for algorithm details
- Check [INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md) for performance metrics
- Use benchmark scripts for experiments

#### **Developers/Engineers**
- Use [QUICK_REFERENCE.md](QUICK_REFERENCE.md) for code examples
- Check [ARCHITECTURE.md](ARCHITECTURE.md) for component details
- Review test cases in `test/test_humanoid_mpc.cpp`

#### **Students/Learners**
- Start with [VISUAL_GUIDE.md](VISUAL_GUIDE.md)
- Run the interactive demo: `./demo_humanoid_mpc`
- Read [ARCHITECTURE.md](ARCHITECTURE.md) for theory

#### **Users/Operators**
- Quick start in [INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)
- Benchmarking guide in [../scripts/README.md](../scripts/README.md)
- Troubleshooting tips in docs

---

## 🎓 Learning Path

### Beginner Path
1. **[INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)** - Overview
2. **[VISUAL_GUIDE.md](VISUAL_GUIDE.md)** - Visual understanding
3. Run the demo: `./demo_humanoid_mpc`
4. **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Try code examples

### Intermediate Path
1. **[ARCHITECTURE.md](ARCHITECTURE.md)** - System details
2. Review `src/spg/setpoint/Set.cpp` - Integration code
3. **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - API usage
4. Run benchmarks: `python3 run_mpc_benchmarks.py`

### Advanced Path
1. **[ARCHITECTURE.md](ARCHITECTURE.md)** - Full algorithm
2. Study `src/spg/setpoint/HumanoidMPC.cpp` - Implementation
3. Review `test/test_humanoid_mpc.cpp` - Test cases
4. Modify weights and run experiments

---

## 🔍 Key Concepts

### Core Components
- **SPG Framework**: Overall path planning system
- **HumanoidMPC**: Model Predictive Controller for humanoid robots
- **MSL Mode**: Traditional omnidirectional control
- **Frame Transformations**: Global ↔ Local coordinate conversion

### Important Topics
- **Robot Local Frame**: Why MPC solves in robot-centric coordinates
- **Measured Velocity**: How current velocity ensures smooth control
- **BO-Ready Parameters**: 10 tunable weights for optimization
- **QP Solving**: How qpOASES finds optimal control

### Performance Metrics
- **Computation Time**: ~2-3ms per timestep
- **Convergence**: ~30 QP iterations
- **Success Rate**: 100% in tests
- **Real-time**: Capable of 50Hz control

---

## 📊 Test Results

All tests passing ✅:
- ✅ Construction
- ✅ NullSolverThrows
- ✅ StraightLineForward
- ✅ SmoothTransition
- ✅ RotationRequired
- ✅ TrajectoryPrediction

---

## 🔗 External Resources

### Dependencies
- **Eigen**: Linear algebra library
- **qpOASES**: QP solver for MPC
- **ImGui/ImPlot**: Visualization (demo only)
- **Google Test**: Unit testing

### Related Papers/Theory
- Model Predictive Control (MPC)
- Robot kinematics in SE(2)
- Quadratic Programming
- Humanoid robot locomotion

---

## 🎯 Next Steps

### For Users
1. Run the demo to see it in action
2. Try different scenarios in benchmarks
3. Experiment with parameter tuning

### For Developers
1. Review the architecture documentation
2. Study the test cases
3. Modify and extend the MPC

### For Researchers
1. Run comprehensive benchmarks
2. Compare with MSL mode
3. Optimize parameters via Bayesian Optimization

---

## 📝 Version History

- **v1.0** (Dec 18, 2025) - Initial integration complete
  - Local frame MPC formulation
  - Measured velocity integration
  - Complete documentation
  - Benchmarking infrastructure

---

## 🙏 Acknowledgments

This integration represents the successful merger of:
- SPG framework (soccer robot path planning)
- HumanoidMPC (model predictive control)
- qpOASES (quadratic programming)

All components working together seamlessly!

---

## 📞 Getting Help

1. **Check documentation**: Start with [INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)
2. **Run tests**: `./test/test_humanoid_mpc`
3. **Try the demo**: `./demo_humanoid_mpc`
4. **Review examples**: See [QUICK_REFERENCE.md](QUICK_REFERENCE.md)

---

**Ready to dive in?** Start with [INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)! 🚀

---

*Documentation Index | Last updated: December 18, 2025*
