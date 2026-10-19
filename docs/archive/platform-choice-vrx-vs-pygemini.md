# Choice of simulator platform for Njord NTNU

**Decision basis: PyGemini compared with VRX, Gazebo and ROS 2**

**Date:** 14 September 2026 (archived; translated from Norwegian)

## 1. Recommendation

**Njord should choose VRX + Gazebo + ROS 2 as the main platform for joint testing of perception, control and autonomy.** In my assessment this combination gives the most direct, documented path to the simulator the team needs: a virtual boat that receives control commands, moves physically and delivers new sensor measurements back to the team's software.

The recommendation concerns the **technology foundation for further development**. The codebase in this repository is a demo that explores feasibility. It should not be judged as a finished alternative that already covers all the team's needs.

The most important decision criterion is therefore:

> Which platform lets the simulation group spend as much of its work as possible on Njord-specific models, scenarios and tests, and as little as possible on developing basic simulator infrastructure?

VRX/Gazebo offers documented building blocks for vessel models, sensors, actuators, environment and ROS integration. This is the basis for the recommendation. See [the VRX project][vrx], [ROS 2 integration][ros] and [Gazebo's marine models][marine].

## 2. Purpose of the simulator

The simulation group's main task is to make it possible to test the team's software digitally before testing on the water. The simulator should support both individual groups and the whole system:

| Group | What the simulator should make possible |
|---|---|
| Perception | Test detection, tracking, sensor fusion and mapping against simulated measurements. |
| Control | Test controllers against force-based motion, inertia, resistance and environmental effects. |
| Autonomy | Test planning, decisions, missions and obstacle handling. |
| Whole team | Test how the algorithms work together, including delays, faults and dropouts. |

The central test loop is:

```text
Simulated sensors
       ↓
The team's perception and state estimation
       ↓
The team's autonomy and control
       ↓
Actuators and physical boat response
       ↓
New sensor measurements → back to the team's software
```

The simulator should adapt to the team's ROS interfaces. The reference algorithms in the demo are replaceable aids for trying this loop.

## 3. Scope of the comparison

We compare two **development platforms**, including realistic extension options. Missing features in the demo are not automatically limitations of VRX/Gazebo. Likewise, possible extensions of PyGemini must be taken into account.

| Term | Meaning |
|---|---|
| **Available** | Documented building blocks exist. Configuration and integration remain. |
| **Development work** | A concrete implementation path exists, but the team must build or connect functionality. |
| **Unclear** | The sources are insufficient to establish support or workload. |

The assessment of PyGemini is based on [the authors' paper from June 2025][pygemini]. An updated maritime code repository and the Njord group's implementation have not been checked. Newer additions may therefore change the comparison.

The assessment of VRX/Gazebo is based on official documentation and the earlier review of the demo. Documented platform capabilities do not confirm that all functions work together in the current container. Chosen versions and plugins must be integrated and tested.

## 4. Capabilities of VRX + Gazebo + ROS 2

| Need | Platform capabilities | Remaining work for Njord |
|---|---|---|
| **Test ROS 2 nodes** | Available communication between ROS 2 and Gazebo through `ros_gz`. | Adapt topics, message types, QoS, TF and simulation time. Some messages need adapters. |
| **Test the whole algorithm chain** | Sensors, physics and actuators can form one closed test loop. | Integrate the team's nodes and define test criteria. |
| **Model the Njord boat** | Own vessel models with geometry, mass, inertia, joints and sensors. No tie to the WAM-V. | Build and calibrate a model of the hull, propulsion and sensor setup. |
| **Acceleration, braking and turning** | Force-based actuators and physical motion. | Adapt motor curves, limits and response. |
| **Hydrodynamics** | Models for buoyancy, damping, added mass and Coriolis terms exist. | Choose a consistent model and determine parameters. |
| **Wind and waves** | VRX has configurable models. | Define relevant environmental conditions and check the vessel's response. |
| **Current** | Gazebo's hydrodynamics supports water current. | Integrate it with the chosen vessel and wave model. |
| **Camera, lidar, GPS and IMU** | Ready sensor models exist. | Adapt placement, rate, field of view, resolution and noise. |
| **More camera types** | Depth, RGB-D, wide-angle and thermal cameras exist. | Configure sensor and ROS interface as needed. |
| **Labelled training images** | Cameras for segmentation and bounding boxes exist. | Build a dataset generator, scene variation and export. |
| **Other vessels and obstacles** | Several models can be driven by their own controllers or plugins. | Develop traffic behaviour, encounter scenarios and any COLREGs logic. |
| **Sensor and actuator faults** | Can be implemented with adapters and plugins. | Build models of dropout, delay, bias and reduced motor force. |
| **Automatic test series** | Running without GUI and with sensor rendering is supported. | Build test orchestration, scoring and result archiving. |
| **Specialized radar and water optics** | The platform can be extended, but a suitable ready solution has not been verified here. | Investigate compatible add-ons or develop own models. Potentially extensive work. |

The table is based on [the ROS integration][ros], [the sensor library][sensors], [marine models and actuators][marine], [hydrodynamics and current][hydro], [own plugins][plugins], [running without GUI][headless] and [VRX's customization tutorials][tutorials].

### Model choices are not platform limits

That the demo uses the WAM-V, a particular sensor resolution or zero added mass in some directions does not mean the platform is limited to these choices. They can be changed or replaced.

Gazebo documents both hydrodynamic parameters and extension through own system plugins. That gives a concrete path to more advanced models. Different physics engines and versions have different properties, and the same force contribution must not be computed by several models at once. See [the hydrodynamics documentation][hydro].

## 5. Capabilities of PyGemini

| Area | Documented basis and extension options |
|---|---|
| **ROS 2** | Native node through `rclpy`. |
| **Camera and lidar** | Sensor functions are documented. |
| **Reconstruction and training data** | Gaussian Splatting, annotation and image augmentation. |
| **Motion** | Trajectories and SeaState disturbances. |
| **Force-based vessel dynamics** | Missing in the 2025 description; integrating external physics libraries is proposed. |
| **Complete physical autonomy loop** | Described as future work. |
| **Own extensions** | Modular Python architecture. |
| **Radar** | Planned in the 2025 description. |

Source: [the PyGemini paper, sections IV and V][pygemini]. Functions that require further development should not be described as impossible. PyGemini has a relevant starting point for sensor and data work.

## 6. Which platform fits the task best?

This is a technical assessment of expected development work, not a measurement of development time or a direct performance benchmark.

| Priority | Assessment |
|---|---|
| **Joint testing of perception, control and autonomy** | VRX/Gazebo recommended: documented building blocks cover the whole physical test loop. |
| **Njord-specific vessel model** | VRX/Gazebo gives a documented path through own models and marine physics systems. Parameters must be obtained regardless of platform. |
| **Specialized training-data generation** | PyGemini should be considered more closely if this becomes a main deliverable. |
| **Low threshold for the team's ROS users** | Prioritize a stable ROS interface. Avoid the algorithm groups needing to know the simulator's internal architecture. |
| **Handover between student cohorts** | Emphasize documentation, maintainers and available help. A larger upstream project does not relieve the team of documenting its own additions. |

VRX has public source code, tutorials, an issue tracker and documented releases, including support for ROS 2 Jazzy and Gazebo Harmonic. This gives a verifiable basis for assessing maintenance and support. See [the project description][vrx] and [the releases][releases].

## 7. Arguments for choosing VRX/Gazebo

### Work can concentrate on Njord's needs

With sensors, actuators, physics and ROS connection available, the group can prioritize the boat model, scenarios and integration. This is my main reason for the choice. The expected gain is less development of basic infrastructure.

### Control needs a physical response model

The controller must be testable against acceleration, inertia, resistance and turning response. A wrong command should have consequences for the subsequent boat state and thereby for upcoming sensor measurements. This makes it possible to study the interplay between control, planning and perception.

### The simulator can be developed gradually

A first model can give value through testing message flow, coordinate frames, algorithm interplay and fault states. The model can then be improved to answer questions about manoeuvring more precisely. Not all calibration needs to be finished before the simulator is used.

### The demo explores a relevant technical direction

The demo's role is to show that the chosen building blocks can be connected in a concrete project. How good the demonstration controller is, or which scenarios it manages, should not drive the platform choice. The team's own algorithms must be able to take over.

## 8. Uncertainties that actually affect the decision

| Uncertainty | Significance |
|---|---|
| **Newer PyGemini code and the group's additions** | May reduce the difference in necessary groundwork. Should be reviewed before a final platform decision. |
| **Available NTNU expertise** | Good local guidance can affect development cost significantly. |
| **Need for visual realism** | Clarify which sensor errors perception actually needs to test. Nice images alone do not document sensor realism. |
| **Data about the Njord boat** | Precise predictions need relevant model parameters and comparison with measurements. |
| **Hardware and performance** | Measure performance with the desired sensor setup and the team's nodes. No general guarantee of real time or bit-identical results is assumed. |
| **Dependence on individuals** | Several members must be able to build, configure and develop the simulator. |

The claims about few PyGemini users, little development activity and the group's progress are not sufficiently verified. They should not be used as main arguments. Nor has the claim of a lower learning curve been measured; this should be assessed through a concrete integration task.

## 9. Proposed decision text

> Njord chooses VRX/Gazebo as the main direction for further development of the simulator. The platform offers documented building blocks for ROS 2 integration, simulated sensors, physical motion and own vessel models. This gives the simulation group a suitable foundation for testing perception, control and autonomy together before trials on the water.
>
> The existing demo is a feasibility trial. Further work is prioritized towards the team's interfaces, relevant test scenarios and gradual adaptation of the boat model. PyGemini is considered a possible supplement for specialized sensor and data work, and newer features are reviewed before the final platform decision.

[vrx]: https://github.com/osrf/vrx
[releases]: https://github.com/osrf/vrx/releases
[tutorials]: https://github.com/osrf/vrx/wiki/tutorials
[ros]: https://gazebosim.org/docs/harmonic/ros2_integration/
[sensors]: https://gazebosim.org/api/sensors/8/namespacegz_1_1sensors.html
[marine]: https://gazebosim.org/api/sim/8/underwater_vehicles.html
[hydro]: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Hydrodynamics.html
[plugins]: https://gazebosim.org/api/sim/8/createsystemplugins.html
[headless]: https://gazebosim.org/api/sim/8/headless_rendering.html
[pygemini]: https://arxiv.org/html/2506.06262v1
