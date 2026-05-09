# AMR Hardware Block Diagram

```mermaid
graph TD
  %% Battery and protections
  subgraph Battery_BMS
    BATT[12.8V LiFePO4 4S 18Ah + BMS<br/>T-connector]
    MSW[Main Power Switch]
    FUSE[Main Fuse]
    ESTOP[E-Stop]
    BATT --> MSW
    MSW --> FUSE
    FUSE --> ESTOP
  end

  %% DC-DC supplies and rails
  subgraph Power_Supplies
    MPBUS[Motor Bus 12-14.6V]
    BUCK_JET[5V Buck Jetson/Hub ~6A XH-M401]
    BUCK_LOGIC[5V Buck Logic/Enc/Prox ~2A LM2596]
    BUCK_12V[12V Buck Spare/Opt Sensors]
    ESTOP --> MPBUS
    MSW --> BUCK_JET
    MSW --> BUCK_LOGIC
    MSW --> BUCK_12V
    MSW --> DVM[DSN-DVM/DUM-368 Volt Display]
  end

  %% Drive system
  subgraph Drive
    MDD[Cytron MDD20A]
    CS_L[ACS758 Left 5V]
    CS_R[ACS758 Right 5V]
    M_L[Left Motor]
    M_R[Right Motor]
    MPBUS --> MDD
    MDD --> CS_L
    CS_L --> M_L
    MDD --> CS_R
    CS_R --> M_R
  end

  %% Control and compute
  subgraph Control
    STM[STM32 Nucleo F401RE]
    JET[Jetson Nano Dev Kit]
    USBHUB[Powered USB Hub]
    PWRBTN[Jetson Soft Power Button<br/>momentary to PWR_BTN]
  end

  %% Sensors
  subgraph Sensors
    ENC_L[Left Encoder 600 PPR<br/>5V NPN OC]
    ENC_R[Right Encoder 600 PPR<br/>5V NPN OC]
    LIDAR[YDLidar G4<br/>USB, 5V from hub]
    DEPTH[RealSense D455<br/>USB 3, 5V from hub]
    IMU[BNO080 IMU<br/>I2C, 3.3V]
    PROX[Proximity x4<br/>HC-SR04 ultrasonic 5V trig/echo]
  end

  %% Power distribution thick orange
  BUCK_JET --> JET
  BUCK_JET --> USBHUB
  USBHUB --> LIDAR
  USBHUB --> DEPTH
  BUCK_LOGIC --> STM
  BUCK_LOGIC --> ENC_L
  BUCK_LOGIC --> ENC_R
  BUCK_LOGIC --> PROX
  BUCK_LOGIC --> CS_L
  BUCK_LOGIC --> CS_R

  %% Control and signals thinner blue
  STM --> MDD
  ENC_L -.-> STM
  ENC_R -.-> STM
  CS_L -.-> STM
  CS_R -.-> STM
  ESTOP -.-> STM
  PROX -.-> STM
  LIDAR --> USBHUB
  DEPTH --> USBHUB
  USBHUB --> JET
  STM <-.-> JET
  IMU -.-> JET
  PWRBTN -.-> JET

  %% Styles: power vs data
  linkStyle 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22 stroke:#e67e22,stroke-width:3px;
  linkStyle 23,24,25,26,27,28,29,30,31,32,33,34,35 stroke:#1f78b4,stroke-width:1.5px;
```

- Encoders are powered from the 5 V logic rail and feed open-collector signals to the STM32 with pull-ups.
- Powered USB hub arrows are correct: LiDAR and RealSense data go to the hub, hub data to Jetson; hub + Jetson 5 V both come from the Jetson/USB buck.
- Power links are thick/orange; data/sense links are thinner/blue for quick visual separation.
- Main power switch sits at pack output ahead of fuse and E-Stop for full isolation during service/storage.
- Battery voltage display (DSN-DVM/DUM-368) taps the pack after the main switch so it is off when the robot is off.
- Proximity sensors: 4x HC-SR04 ultrasonic modules (5 V, trig/echo) wired to STM32 GPIO; stagger triggers to avoid crosstalk.
- IMU: BNO080 on Jetson I2C (3.3 V). Power from Jetson 3.3 V rail; keep cable short and avoid vibration.
- Jetson soft power button: momentary N.O. switch across JET PWR_BTN to GND for graceful shutdown/start (no power cut).
