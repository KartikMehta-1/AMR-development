Phase 1 — Embedded Foundations (Weeks 1 – 6)
Week	Focus	Goal & Highlights	Est. Time
1 — Safety & Tools Setup	✅ (Completed)	Bench wiring, E-stop, fused path. Blink LED verified.	5–8 h
2 — UART + ADC + Debug (Ongoing)	Serial comms via USB-CDC/UART, read ADC. Basic command parser.	5–8 h
3 — PWM → Motor Driver	Generate PWM (~20 kHz), implement ramp & E-stop. Validate spin @ low duty.	6–10 h	
4 — Encoder Hookup & Counting	Configure hardware encoder mode. Verify counts ↑/↓ and speed.	6–10 h	
5 — RPM & Telemetry Basics	Compute RPM, print telemetry to serial, log CSV data.	5–8 h	
6 — Closed-Loop Speed Control (PID)	Implement PID with anti-windup & rate limit; step-test tuning.	6–10 h	

Deliverable after Phase 1:

STM32 controls motor + encoder + PID at bench.

Stable serial telemetry stream ready for Jetson/ROS consumption.

Phase 2 — Sensor & Safety Expansion (Weeks 7 – 10)
Week	Focus	Goal & Highlights	Est. Time
7 — Current Sensor Integration	Add ACS758 or INA226; measure current, implement over-current limit.	6–10 h	
8 — Proximity Sensors & Local Safety Logic	Add bump/ToF sensors; debounce + safety stop behavior.	6–10 h	
9 — BMS & Power Supervision	Integrate BMS telemetry and fault handling; pre-charge sequence.	6–10 h	
10 — System Validation & Refactor	Test all sensor paths together; refactor code into modules (motor.c, sensors.c, safety.c).	6–10 h	

Deliverable after Phase 2:

STM32 manages motor + sensors + BMS with independent safety stop.

Clean code architecture ready for Jetson link.

Phase 3 — ROS Integration & Simulation (Weeks 11 – 14)
Week	Focus	Goal & Highlights	Est. Time
11 — ROS Setup & Simulation Basics	Install ROS 2 (Humble/Iron), Gazebo Harmonic + RViz. Build robot URDF model + simple plugin for wheel drive.	10–14 h	
12 — Jetson ↔ STM32 Communication	Implement rosserial or micro-ROS link for setpoints + telemetry. Test Teleop via RViz/Gazebo.	8–12 h	
13 — SLAM + Localization	Bring-up LIDAR or Depth Camera; run Cartographer/RTAB-MAP; visualize maps in RViz.	10–14 h	
14 — Navigation & Path Following	Integrate nav2 stack or simple custom planner. Test cmd_vel → motor loop via STM32.	10–14 h	

Deliverable after Phase 3:

Full simulation pipeline works in Gazebo/RViz.

Real Jetson controls motor through STM32 using ROS topics.

Phase 4 — System Hardening & Demo (Weeks 15 – 16)
Week	Focus	Goal & Highlights	Est. Time
15 — PID Tuning & Safety Validation	Tune for payload; test fail-safes (lost encoder, Jetson fault). Thermal check driver & motor.	8–12 h	
16 — Packaging & Documentation	Build harnesses, label connectors, create pre-flight + flashing guides, run 30–60 min demo.	8–12 h	

Final Deliverables:

AMR ready for live demo with ROS navigation + safety stack.

Complete documentation and next-rev PCB plan.