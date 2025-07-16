# HSA Project - E-Skin Controlled Vehicle with IMU Trajectory Recording

## Project Overview
This project implements a vehicle control system using TUM ICS electronic skin (e-skin) sensors for directional control, combined with IMU-based trajectory recording. The e-skin sensors detect force inputs that are translated into movement commands for the vehicle.

## Repository Structure
```
HSA_Project/
├── docs/               # Collected documentation and research materials
├── skin_tutorial/      # TUM ICS skin tutorial examples (reference only)
└── test_skin_ws/       # Main development workspace - START HERE
```

## Important Development Guidelines

### ⚠️ BEFORE YOU START
1. **Never push directly to main/master branch**
2. **Always create your own feature branch for development**
3. **Submit pull requests for code review before merging**

### Getting Started
```bash
# Clone the repository
git clone [repository-url]

# Create your feature branch
git checkout -b feature/your-feature-name

# Navigate to the development workspace
cd test_skin_ws

# Open VS Code from here (not from the top level)
code .
```

## Project Components
- **E-Skin Interface** (Lukas): Force sensor data acquisition and processing
- **Vehicle Control** (TBD): Translates sensor inputs to movement commands  
- **IMU Integration** (TBD): Records and processes trajectory data
- **System Integration** (TBD): Combines all components into working system

## Next Steps
For development instructions, see `test_skin_ws/README.md`
