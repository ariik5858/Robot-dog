# Robot Dog
A robot dog that I might finish
<img width="985" height="785" alt="image" src="https://github.com/user-attachments/assets/c3ec39c2-5ee8-447b-877c-b2ec503ed40d" />

## Background
I wanted to make a robot dog for a very long time, so I am starting now.

## Project Structure
This project has been divided into two parts:
### 🔧 Mechanical  
- CAD models of individual parts and assemblies  
- Bill of materials (list of external components purchased)  
- Notes on fabrication and assembly  

### 💻 Software  
- Low-level driver code for actuators and sensors  
- Middleware for communication between hardware and higher-level logic  
- Higher-level control and autonomy (e.g., gait generation, stability, navigation) 

## Contributors
Kohei Ariizumi
Jacko Chan (github)

## Roadmap
- [ ] Mechanical
  - [x] Mechanical Design
  - [ ] Assembly
- [ ] Software
  - [ ] Driver code
    - [ ] IK
    - [ ] Simulations
    - [ ] Determine if code will be on Arduino or STM32
  - [ ] Implement Basic Locomotion code
  - [ ] Autonomy
    - [ ] ROS2 (will need to buy Pi and cameras)