# Real-Time Vehicle Detection and Alert System for Road Safety on Narrow, Curved, and Inclined Roads

---

# 1. Introduction

Narrow, curved, and inclined road segments present significant safety challenges for both drivers and road authorities. The restricted sightlines inherent to these environments prevent drivers from observing oncoming vehicles in time to make safe and timely decisions, particularly at blind spots where collision risks are substantially elevated. Traditional road signage and static warning systems are insufficient for dynamic traffic conditions, as they lack the ability to respond to real-time vehicle presence, type, or speed.

This research addresses these limitations by proposing an intelligent, sensor-based roadside alert system capable of detecting approaching vehicles in real time, classifying them by type and size, estimating their speed, and communicating this information wirelessly between two roadside posts positioned at opposite ends of a constrained road segment. The system translates this information into actionable driver signals delivered through a visual LED display and an audio warning module, thereby enabling safer passage decisions for drivers approaching blind-spot zones.

---

# 2. Research Objectives

1. To design a driver-oriented visual and audio alert system that can be easily understood with minimal distraction to the driver.
2. To develop a roadside prototype with a depth camera and a vision processor capable of detecting vehicle speed and classification in real-time.
3. To integrate a speed estimation algorithm that computes vehicle speed and evaluates the accuracy of the estimation with respect to actual speed values.
4. To evaluate wireless communication between two roadside stations for real-time vehicle data exchange.
5. To assess the overall performance and usability of the system in reducing collision risks in blind-spot areas, specifically on curved, inclined, and narrow roads.

---

# 3. System Overview

The proposed system consists of two independent but interdependent roadside posts, designated as Post A and Post B, installed at each end of a narrow, curved, or inclined road segment. Each post operates as both a sensing node and a communication terminal, continuously monitoring approaching vehicles and sharing real-time data with the opposing post.

Each post is equipped with a depth camera for vehicle detection, classification, and speed estimation; a single-board computer for local processing and decision logic; and a wireless transceiver for inter-post data exchange. The receiving post processes the transmitted data and generates appropriate driver alerts via an LED display panel and an audio output module. The system is self-powered through a solar energy setup, making it suitable for remote or off-grid road segments where grid electricity is unavailable.

The overarching goal of the system is to give drivers on one side of the road sufficient advance warning about vehicles approaching from the other side, enabling them to stop, slow down, or proceed safely based on the current traffic state.

---

# 4. System Architecture

The system is structured around a two-node, bidirectional communication architecture. Each node operates identically in terms of hardware configuration but assumes complementary roles at any given moment depending on the direction of detected traffic.

**Post A** captures and processes vehicle data from vehicles approaching its side of the road. It transmits detected vehicle information — including vehicle class, estimated distance, and speed — to Post B via wireless communication.

**Post B** receives this data, applies the system's decision-making logic, and drives the output systems to display an appropriate signal to drivers on its side of the road. The same process occurs in reverse: Post B transmits its locally detected data to Post A, which then generates corresponding outputs for drivers on its side.

This bidirectional design ensures that both ends of the road are continuously informed of traffic conditions on the opposing end. The decision-making logic resides primarily on the receiving post's single-board computer, which evaluates detected vehicle attributes from both its local sensor and the transmitted data from the opposing post before determining the appropriate driver signal.

The communication flow follows this pattern:

```
[Post A: OAK-D Lite Detection] --> [Post A: Raspberry Pi 5 Processing]
    --> [HC-12 Transceiver Transmission]
        --> [Post B: HC-12 Transceiver Reception]
            --> [Post B: Raspberry Pi 5 Decision Making]
                --> [ESP32 Display Controller] --> [P10 LED Panel]
                --> [Audio Player Module]
```

This flow operates symmetrically in the opposite direction — from Post B to Post A — at all times.

---

# 5. Hardware Components

## 5.1 OAK-D Lite Depth Camera

The OAK-D Lite is a compact stereo depth camera equipped with an onboard Myriad X Vision Processing Unit (VPU). It serves as the primary sensing device at each post, responsible for capturing video frames of approaching vehicles and providing depth (distance) measurements through its stereoscopic vision pipeline. The onboard VPU enables neural network inference and spatial coordinate extraction to be executed directly on the camera hardware, significantly offloading the computational burden on the main processing unit.

## 5.2 Raspberry Pi 5

The Raspberry Pi 5 is the central processing unit of each post. It manages data acquisition from the OAK-D Lite camera, executes scene interpretation when local processing is required, handles wireless data transmission and reception via the HC-12 transceiver, applies the system's prioritization and decision-making logic, and issues output commands to the ESP32 and Audio Player Module. The Raspberry Pi 5 was selected for its improved CPU performance over previous generations, making it capable of supporting the real-time demands of the system.

## 5.3 HC-12 Wireless Transceiver

The HC-12 is a long-range, low-power wireless serial communication module operating in the 433 MHz frequency band. It facilitates the real-time exchange of vehicle data between Post A and Post B. The transceiver connects to the Raspberry Pi 5 via serial UART, transmitting structured data packets containing vehicle class, distance, speed, and detection timestamp.

## 5.4 ESP32 Display Controller

The ESP32 microcontroller acts as the dedicated driver for the P10 Full Color LED Panel. It receives formatted display commands from the Raspberry Pi 5 through UART (Rx/Tx pins) and renders the appropriate visual output on the LED panel. This modular design separates display management from the main processing logic, ensuring that display updates are handled efficiently without overloading the Raspberry Pi 5.

## 5.5 P10 Full Color LED Panel

The P10 Full Color LED Panel serves as the primary visual output interface for drivers. It displays real-time traffic information including vehicle type, estimated speed, and the system's recommended action signal — such as GO, GO SLOW, or STOP. The panel is designed to be clearly visible from a distance in varying ambient light conditions, including direct sunlight.

## 5.6 Audio Player Module

The Audio Player Module provides audio-based warnings to supplement the visual LED display. It is triggered by the Raspberry Pi 5 and plays pre-recorded voice messages or buzzer tones corresponding to the current traffic state. Example outputs include voice alerts such as "Truck approaching, please stop," which reinforce the visual signal displayed on the LED panel.

## 5.7 Solar Power System

Each post is powered by an independent solar energy system consisting of a solar panel, an MPPT (Maximum Power Point Tracking) charge controller, a storage battery, and step-down (buck) converters. The MPPT controller maximizes energy harvest from the solar panel and regulates charging of the battery. Buck converters step down the battery voltage to the levels required by the individual hardware components. This configuration ensures continuous operation of each post without dependency on grid power infrastructure, making the system viable for deployment in remote or rural road locations.

---

# 6. Data Processing Pipeline

The end-to-end data flow of the system proceeds through the following stages:

```
OAK-D Lite (Depth Camera)
    --> Raspberry Pi 5 (Local Processing & Data Packaging)
        --> HC-12 Transceiver (Wireless Transmission)
            --> HC-12 Transceiver at Opposite Post (Reception)
                --> Raspberry Pi 5 at Opposite Post (Decision Making)
                    --> ESP32 (Display Command Routing)
                        --> P10 LED Panel (Visual Driver Output)
                    --> Audio Player Module (Audio Driver Output)
```

The pipeline operates bidirectionally and continuously. Each post simultaneously acts as a sender of locally detected vehicle data and a receiver of remotely detected vehicle data from the opposing post. The decision-making logic at the receiving post consolidates both data streams to determine the appropriate output signal for drivers at that location.

---

# 7. Vehicle Detection and Speed Estimation

## 7.1 Detection and Classification

The OAK-D Lite camera captures video frames of the road environment and passes them through a YOLO-based object detection model to identify and classify vehicles within the scene. The system supports two operational configurations:

**NCNN Model Configuration:** In this approach, object detection is performed using an NCNN-optimized neural network model running on the Raspberry Pi 5. This configuration achieves frame rates exceeding 30 FPS and is designed for dashboard or headless deployment scenarios where minimizing CPU load on the Raspberry Pi 5 is a priority. The headless design reduces the graphical overhead and allows the system to dedicate more processing resources to inference and communication tasks.

**DepthAI Library Configuration:** This approach leverages the DepthAI software library, which allows the YOLO object detection model to run directly on the OAK-D Lite's onboard Myriad X VPU. In addition to running inference on-camera, the DepthAI pipeline enables spatial tracking and the extraction of real-time 3D coordinates for each detected object. This results in seamless, low-latency depth-fused detections, wherein each identified vehicle is provided with an estimated 3D position in the scene without requiring the Raspberry Pi 5 to perform depth computation.

## 7.2 Speed Estimation

Speed estimation is performed by tracking changes in a vehicle's measured distance over successive detection frames. The OAK-D Lite provides depth (distance) values for each detected object in real time. By computing the displacement of a vehicle across consecutive frames and dividing by the corresponding time interval, the system derives an instantaneous speed estimate in kilometers per hour (km/h).

Speed values are collected continuously over a sliding observation window to support subsequent kinematic analyses, including acceleration computation and speed variance calculation.

---

# 8. Wireless Communication System

The two roadside posts exchange vehicle data in real time using the HC-12 wireless transceiver modules connected to each Raspberry Pi 5 via UART serial communication. When the OAK-D Lite at a given post detects a vehicle, the Raspberry Pi 5 packages the relevant data — including the detected vehicle class, its current estimated speed in km/h, and its measured distance from the post — into a structured data packet. This packet is transmitted wirelessly to the opposing post via the HC-12 module.

The HC-12 module at the receiving post delivers the data packet to its connected Raspberry Pi 5, which parses the incoming data and integrates it with locally detected vehicle information. The combined data is then passed to the decision-making logic to determine the appropriate driver alert output.

The communication operates in both directions simultaneously, ensuring that each post remains continuously informed of conditions on the opposite side of the road segment.

---

# 9. Driver Alert System

## 9.1 Visual Display (LED Panel)

The P10 Full Color LED Panel at each post displays a structured set of real-time information. The display content includes the type of vehicle detected at the opposing post, the vehicle's estimated speed in km/h, and the system's recommended action signal for drivers at the current post. The three primary action signals are:

- **GO** — The road ahead is clear or the approaching vehicle poses minimal passage conflict. Drivers may proceed.
- **GO SLOW** — An approaching vehicle has been detected. Drivers should proceed with caution at reduced speed.
- **STOP** — An approaching vehicle is present and the road cannot safely accommodate simultaneous two-way passage. Drivers must wait.

When an emergency vehicle has activated Emergency Mode, the display additionally shows a prominently formatted emergency warning alongside the corresponding action signal.

## 9.2 Audio Warnings

The Audio Player Module complements the LED display by delivering spoken voice messages or buzzer tones that correspond to the current alert condition. Example audio outputs include messages such as "Truck approaching, please stop" or a warning tone to reinforce the STOP signal. These audio cues are particularly useful in conditions where the driver's attention may be directed away from the LED panel, providing an additional sensory channel for situational awareness.

---

# 10. Example Operational Scenario

The following scenario illustrates the system's real-time response to a representative traffic condition.

Both Post A and Post B continuously scan their respective approaches to the road segment. Upon detecting a slowly moving motorcycle at Post A's end, the Raspberry Pi 5 at Post A packages the detection data — vehicle class: motorcycle, speed: 12 km/h, distance: 25 m — and transmits it wirelessly to Post B. Post B receives this data, applies the prioritization logic, and updates its LED panel to display the motorcycle icon, its current speed, and a GO signal for drivers on Post B's side, since the motorcycle occupies minimal road width and poses a low obstruction risk.

In contrast, if Post A detects a heavy truck entering the narrow segment — for example, a truck traveling at 20 km/h at a distance of 40 m — Post B's LED panel will immediately switch to a STOP signal and the Audio Player Module will broadcast a voice message such as "Truck approaching, please stop." This alerts drivers at Post B's side to hold their position until the truck has fully cleared the road segment. Once the truck has passed and Post A no longer detects any vehicle, Post B updates the LED panel to a GO or GO SLOW signal accordingly.

In a scenario where vehicles are detected simultaneously from both ends, the system's prioritization logic takes effect. The post detecting the lower-priority vehicle — for instance, a motorcycle against a truck — will hold the signal at STOP for drivers on its side, allowing the higher-priority vehicle (the truck) to pass first. Once the higher-priority vehicle has cleared the road, the system resumes dynamic signal updates based on live sensor input.

---

# 11. Vehicle Size Classification

The system assigns each detected vehicle class to one of three size categories — Small, Medium, or Large — for use in prioritization logic. Emergency vehicles are initially assigned to their corresponding physical size category during normal operation.

| Size Category | Vehicle Types |
|---|---|
| Small | Bicycle, Motorcycle, EV Small |
| Medium | Tricycle, EV Large, Tuktuk, Car, Kariton, Kalesa |
| Large | Van, Jeepney, Bus, Truck |

**Emergency Vehicle Size Handling (Normal Mode):**

| Emergency Vehicle | Size Classification (Normal Mode) |
|---|---|
| Police Car | Medium |
| Ambulance | Large |
| Fire Truck | Large |

When Emergency Mode is not active, emergency vehicles are treated according to the road space and obstruction characteristics corresponding to their physical size. When Emergency Mode is activated, a separate override logic applies as described in Section 13.

---

# 12. Prioritization Logic

## 12.1 Normal Operation (No Emergency Condition)

During normal operation, when no emergency vehicle has triggered Emergency Mode, the system prioritizes vehicles according to their size classification in the following order:

1. **Large Vehicles** — Highest Priority (Van, Jeepney, Bus, Truck, Ambulance, Fire Truck)
2. **Medium Vehicles** — Intermediate Priority (Car, Tricycle, Tuktuk, EV Large, Kariton, Kalesa, Police Car)
3. **Small Vehicles** — Lowest Priority (Bicycle, Motorcycle, EV Small)

The rationale for this prioritization order is grounded in physical and safety considerations:

- **Lane Width Occupation:** Larger vehicles occupy a greater proportion of the road width on narrow segments, leaving insufficient clearance for simultaneous bidirectional passage.
- **Stopping Distance:** Due to their greater mass, larger vehicles require significantly longer distances to come to a complete stop, making it hazardous to require them to yield to smaller vehicles mid-passage on inclined or curved roads.
- **Obstruction Risk:** On blind curves and narrow inclines, a large vehicle that cannot complete its passage creates a greater safety hazard than a small vehicle in the same circumstance.

---

# 13. Emergency Mode Detection

The YOLO-based object detection model used in the system is capable of identifying vehicle classes — including ambulance, fire truck, and police car — but it cannot determine whether a detected emergency vehicle is actively responding to an emergency call or traveling under normal, non-urgent conditions. To address this limitation, the system employs a kinematic-based emergency detection method that infers the likelihood of an active emergency response from the vehicle's observed motion characteristics.

Emergency Mode is not applied to all detected emergency vehicles by default. Instead, it is activated only when the system determines, through kinematic evidence, that the vehicle is exhibiting motion behavior consistent with an active emergency response.

---

# 14. Emergency Detection Conditions

Emergency Mode is activated when the following compound condition is satisfied:

**Emergency vehicle class detected AND at least one of the following kinematic conditions is met.**

## 14.1 Dynamic Speed Threshold

Emergency Mode is activated if the detected speed of the emergency vehicle meets or exceeds the dynamic speed threshold:

```
Detected Speed >= (Average Road Speed + Speed Threshold)
```

The speed threshold value is road-type dependent:

| Road Type | Recommended Speed Threshold |
|---|---|
| Narrow Roads | +10 km/h above average traffic speed |
| Curved Roads | +10 km/h above average traffic speed |
| Inclined Roads | +15 km/h above average traffic speed |
| Narrow, Curved, and Inclined (Combined) | +15 km/h above average traffic speed |

For road sections that simultaneously exhibit narrow, curved, and inclined characteristics — as is the case in the target deployment environment — the highest applicable threshold is used. This conservative selection minimizes the rate of false positive Emergency Mode activations, ensuring that the override is reserved for vehicles exhibiting a meaningfully elevated speed relative to prevailing traffic conditions.

The average road speed is dynamically computed from a rolling measurement of vehicle speeds observed at each post over a defined time window, providing a context-sensitive baseline rather than a fixed reference value.

## 14.2 Aggressive Acceleration Detection

Emergency Mode is activated if the emergency vehicle exhibits sustained longitudinal acceleration meeting the following condition:

```
Longitudinal Acceleration >= 1.5 m/s² sustained for at least 2 seconds
```

The rationale for this condition is that drivers traversing narrow, inclined, or curved roads under normal circumstances tend to accelerate conservatively due to road geometry constraints and safety awareness. Emergency vehicles responding to active calls, on the other hand, exhibit rapid, sustained acceleration that clearly distinguishes them from normal traffic behavior. The two-second sustain requirement reduces transient false triggers caused by brief, incidental speed increases.

## 14.3 Speed Variance Detection

Speed variance measures the degree of speed fluctuation exhibited by a vehicle over a short observation window. Emergency vehicles in active response typically demonstrate greater speed variability than normal drivers, as they accelerate aggressively after curves, maintain higher velocity through transitions, and exhibit larger speed fluctuations driven by urgency rather than safety-conservative driving behavior.

Speed variance is therefore used as a supplementary kinematic indicator to distinguish emergency response behavior from normal driving patterns.

---

# 15. Mathematical Formulation

Speed variance is computed using the standard deviation of instantaneous speed measurements collected over a sliding observation window, as expressed by the following formula:

$$\sigma = \sqrt{\frac{1}{N} \sum_{i=1}^{N} (v_i - \bar{v})^2}$$

Where:

| Symbol | Definition |
|---|---|
| $v_i$ | Instantaneous speed value at frame $i$, measured in km/h |
| $\bar{v}$ | Mean speed over the observation window, computed as $\bar{v} = \frac{1}{N} \sum_{i=1}^{N} v_i$ |
| $N$ | Total number of speed samples within the observation window |
| $\sigma$ | Computed speed standard deviation (speed variance indicator), in km/h |

Emergency Mode is activated for the speed variance condition when:

$$\sigma \geq \sigma_{\text{threshold}}$$

Where $\sigma_{\text{threshold}}$ is a calibrated variance threshold determined through experimental observation during field testing.

---

# 16. Recommended Experimental Parameters

The following parameters are recommended as initial values for system deployment and calibration in the target road environment, which is characterized by road sections that are simultaneously narrow, curved, and inclined.

| Parameter | Recommended Value | Notes |
|---|---|---|
| Observation Window Duration | 0.5 to 1.0 second | Balances responsiveness with noise suppression |
| Frame Count per Window | 10 to 20 frames | Based on an inference rate of approximately 20 FPS |
| Initial Variance Threshold | 3 to 5 km/h standard deviation | Subject to field calibration |
| Speed Threshold for Combined Road Type | +15 km/h above average road speed | Highest threshold applied for conservative false-positive control |
| Acceleration Threshold | 1.5 m/s² sustained for at least 2 seconds | Standard for emergency vehicle kinematic signature detection |

All threshold values specified above are initial recommendations and should be refined through systematic field testing under representative traffic conditions at the deployment site. Calibration sessions should record speed, acceleration, and variance measurements from known vehicle types across multiple trials to establish reliable decision boundaries.

---

# 17. Emergency Mode Trigger Logic

The complete Emergency Mode trigger condition is expressed as follows:

```
Emergency Mode Trigger =
    (Emergency Vehicle Class Detected)
    AND
    [
        (Detected Speed >= Average Road Speed + Threshold)
        OR
        (Longitudinal Acceleration >= 1.5 m/s² for >= 2 seconds)
        OR
        (Speed Variance >= Calibrated Variance Threshold)
    ]
```

This compound condition ensures that Emergency Mode is activated only when there is both contextual evidence (emergency vehicle class) and behavioral evidence (kinematic threshold violation) of an active emergency response. No single factor in isolation is sufficient to trigger Emergency Mode:

- Vehicle class alone is insufficient — a parked or slowly moving emergency vehicle should not trigger an override.
- Speed alone is insufficient — a non-emergency vehicle briefly traveling above average speed should not be classified as an emergency.
- Acceleration or variance alone is insufficient — transient driving behaviors on inclined or curved roads can produce isolated kinematic anomalies.

The requirement for both conditions to be satisfied simultaneously provides a robust and defensible mechanism for distinguishing genuine emergency responses from coincidental similarities in vehicle behavior.

**Special Handling for Fire Trucks:**

Fire trucks enter Emergency Mode only when the speed threshold or acceleration threshold condition is independently satisfied, in addition to the vehicle class detection requirement. This special handling reflects the consideration that fire trucks, due to their size and operational demands, may exhibit large speed variance during normal maneuvering on complex road geometry without necessarily being in active emergency response.

**Police Car Handling:**

Police cars are subject to the standard Emergency Mode logic. When triggered, they transition from their default Medium size classification to Emergency Mode override priority.

**Ambulance Handling:**

When an ambulance triggers Emergency Mode through any of the kinematic threshold conditions, it overrides all active prioritization rules and is assigned the highest priority in the system, superseding even Large-class vehicles from the opposing post.

---

# 18. System Decision Output

The Raspberry Pi 5 at the receiving post consolidates the following inputs to generate the final driver signal:

1. Locally detected vehicle data (class, speed, distance) from the onboard OAK-D Lite camera.
2. Remotely transmitted vehicle data (class, speed, distance) received from the opposing post via HC-12.

Based on this consolidated data, the decision logic evaluates the relative priority of detected vehicles on both sides and determines the appropriate signal for drivers at the current post. The decision output is then relayed to two output systems:

- **ESP32 and P10 LED Panel:** Receives formatted display commands from the Raspberry Pi 5 and renders the appropriate visual output including vehicle icon representation, speed value, and action signal (GO, GO SLOW, or STOP). Under Emergency Mode, the panel additionally displays a clearly formatted emergency warning message alongside the corresponding directive.

- **Audio Player Module:** Receives trigger signals from the Raspberry Pi 5 and plays the audio message associated with the current alert state, reinforcing the visual signal with a spoken or tonal warning.

The system updates its output in real time as new data is received from either the local sensor or the opposing post, ensuring that the displayed information consistently reflects current road conditions.

---

# 19. Hardware Data Flow Summary

The complete end-to-end hardware data flow of the system is summarized as follows:

```
OAK-D Lite (Depth Camera)
    |
    v
Raspberry Pi 5 (Local Edge Processing, Speed Estimation, Data Packaging)
    |
    v
HC-12 Wireless Transceiver (433 MHz Serial Transmission)
    |
    v (Wireless Channel)
    |
HC-12 Wireless Transceiver at Opposite Post (Reception)
    |
    v
Raspberry Pi 5 at Opposite Post (Decision Making, Signal Routing)
    |
    |----> ESP32 Microcontroller (via UART)
    |           |
    |           v
    |       P10 Full Color LED Panel (Visual Driver Output)
    |
    |----> Audio Player Module (Audio Driver Output)
```

This pipeline operates bidirectionally and simultaneously. Each post continuously runs all stages of the pipeline in both the transmission and reception directions, ensuring that the system remains responsive to changes in traffic conditions at either end of the road segment without delay.

---
