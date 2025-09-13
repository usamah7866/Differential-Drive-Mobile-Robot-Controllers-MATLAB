# Mobile Robot Control Algorithms (MATLAB)

This repository contains MATLAB implementations of various control strategies for a **Differential Drive Mobile Robot (DDMR)**.  
The aim is to design, simulate, and compare multiple controllers for trajectory tracking and disturbance rejection.

---

## 📂 Repository Contents

The repo includes **five MATLAB script files**, each implementing a different controller:

1. **`kinematic_controller.m`**  
   - Implements a kinematic controller for differential drive mobile robots.  
   - Uses robot kinematic equations to compute linear (`v`) and angular (`w`) velocity commands.  
   - Provides a baseline for trajectory tracking without advanced disturbance handling.  

2. **`adrc_controller.m`**  
   - Implements **Active Disturbance Rejection Control (ADRC)**.  
   - Employs an Extended State Observer (ESO) to estimate and compensate disturbances in real time.  
   - Suitable for robust control under uncertain environments.  

3. **`fuzzy_type1_controller.m`**  
   - Implements a **Type-1 Fuzzy Logic Controller (FLC)**.  
   - Uses predefined membership functions and rule bases.  
   - Provides adaptive nonlinear control with linguistic rules.  

4. **`fuzzy_type2_controller.m`**  
   - Implements a **Type-2 Fuzzy Logic Controller (IT2-FLC)**.  
   - Incorporates uncertainty handling in membership functions.  
   - More robust compared to Type-1 FLC under noisy/disturbed environments.  

5. **`fuzzy_tsk_controller.m`**  
   - Implements a **Takagi–Sugeno–Kang (TSK) Fuzzy Controller**.  
   - Uses rule-based modeling with linear consequents.  
   - Provides precise control with better approximation of nonlinear dynamics.  

---

## ⚙️ Requirements

- **MATLAB R2018a or later**  
- Toolboxes:  
  - Fuzzy Logic Toolbox (for Fuzzy Type-1, Type-2, TSK)  
  - Control System Toolbox (optional, for ADRC tuning)  

---

## 🚀 How to Run

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/mobile-robot-controllers.git
   cd mobile-robot-controllers
2. Open MATLAB and navigate to the cloned folder.

3. Run any script file:

4. run('kinematic_controller.m')


Modify the desired trajectory or disturbances inside the script for testing.
