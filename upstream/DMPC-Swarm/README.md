This is the accompanying repository for our paper: DMPC-Swarm: Distributed Model Predictive Control on Nano UAV Swarms.

# Overview
Firmwares: Contains the firmware for the Crazyflie and of the nrf52840.

cps_testbed_python: Contains the code for the CUs and a Software-in-the-Loop simulation. Entrance point: simulation/dmpc_simulation_caller.py

# Installation
## Simulation
```Console
cd cps_testbed_python
pip install -r requirements.txt
python dmpc_simulation_caller.py
```

## Hardware Experiments
Install crazyswarm: https://crazyswarm.readthedocs.io/en/latest/

Install Segger Embedded Studio V5.44 for ARM: https://www.segger.com/downloads/embedded-studio/ .
You can open the firmware (e.g., to build and flash it) by opening firmwares/cp_firmware/app/cp_firmware.emProject.

Install the Crazyflie development dependencies: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#dependencies . Build and flash the firmware as described in the Crazyflie docs.

To start the swarm, flash the Crazyflies, configure Crazyswarm according to your setup, power on the Crazyflies and turn on Crazyswarm.

On the CUs run
```Console
cd cps_testbed_python
sudo python spawn_CUs_initiator.py/sudo python spawn_CUs2.py
```


## Hardware Files
For gerber files regarding the communication PCBs (for Crazyflie and CUs), please contact: alexander.graefe@dsme.rwth-aachen.de

# Citation
```
@article{Graefe2025DMPCSwarm,
	author    = {Gr{\"a}fe, Alexander and Eickhoff, Joram and Zimmerling, Marco and Trimpe, Sebastian},
	title     = {{DMPC-Swarm: Distributed Model Predictive Control on Nano UAV Swarms}},
	journal   = {Autonomous Robots},
	year      = {2025},
	volume    = {49},
	pages     = {28},
	doi       = {10.1007/s10514-025-10211-w},
	url       = {https://doi.org/10.1007/s10514-025-10211-w}
}
``` 
