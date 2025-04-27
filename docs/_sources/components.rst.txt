Components
==========

SLAM
----
.. automodule:: scripts.slam
   :members:
   :undoc-members:
   :show-inheritance:


Path Planner
------------

.. automodule:: scripts.path_planner
   :members:
   :undoc-members:
   :show-inheritance:


Navigation Controller
---------------------

.. automodule:: scripts.navigation_controller
   :members:
   :undoc-members:
   :show-inheritance:

Speaker Service
---------------

.. automodule:: scripts.speaker_service
   :members:
   :undoc-members:
   :show-inheritance:


Triage System
--------------

.. image:: ../images/triage_behavior_diagram.png
   :width: 500px
   
Diagram Overview
~~~~~~~~~~~~~~~~~

**1. Description of the Sequence Diagram (Triage System)**

This sequence diagram illustrates the interactions between different
entities involved when the robot performs a triage assessment on a
detected victim.

-   **Participants:** The key participants (represented by vertical
    lifelines) are the Operator (human supervisor), the main Triage
    System logic unit, the RGB-D Camera, Speech Recognition module,
    Microphones, and Speakers.

-   **Initial Assessment:** The process begins (presumably after a
    victim is detected, indicated by the alt [Victim Detected]
    fragment) with the Triage System acquiring visual data from the
    RGB-D Camera and audio data from the Microphones. This data is
    initially processed to assess consciousness and potential injuries.

-   **Responsiveness Check:**

    -   The Triage System then initiates an interaction to check
        responsiveness. It instructs the Speakers to play an instruction
        ("Can you hear me?"). This is shown within a Loop(4) fragment,
        suggesting the system might try this interaction up to four
        times if necessary.

    -   The Microphones capture any response, sending the audio data to
        the Triage System.

    -   This audio data is forwarded to the Speech Recognition module to
        determine if a human response was received.

    -   The Triage System processes the result from Speech Recognition.

-   **Conditional Action (Responsiveness):** An alt (alternative)
    fragment shows two paths based on the responsiveness check:

    -   If the victim is responsive ([Victim Responsiveness]), the
        Triage System uses the Speakers to give a set of instructions.

    -   Otherwise ([Else]), the Triage System alerts the Operator
        about the unresponsive victim.

-   **Reporting:** After the responsiveness check (and potential
    instruction), the Triage System sends a Triage Report (categorized
    as Urgent/Stable) to the Operator.

-   **Optional Detailed Assessment:** An opt (optional) fragment at the
    end shows that the Operator can request a Detailed Assessment. If
    requested, the Triage System acquires close-up visual data using the
    RGB-D Camera and sends a Detailed Report back to the Operator.

**In essence, the diagram details the step-by-step message exchange and
processing flow required for the robot to assess a victim's condition,
interact with them, and report the findings.**

**2. Why a Sequence Diagram is an Appropriate Behavioral Diagram for the
Triage System**

A sequence diagram is an excellent choice for representing the behavior
of the **Triage System** component for the following reasons:

-   **Focus on Interactions and Order:** The core function of the Triage
    System involves a specific sequence of interactions between multiple
    components (the system logic, sensors like the camera and
    microphone, actuators like the speaker, external systems like speech
    recognition, and the human operator). Sequence diagrams excel at
    visualizing this *time-ordered sequence of messages* passed between
    different objects or components. This clearly shows *who* does
    *what* and *when*.

-   **Clarity of Collaboration:** The Triage System doesn't operate in
    isolation. It collaborates with the camera, microphone, speaker,
    speech recognition, and the operator. A sequence diagram clearly
    depicts these collaborations and dependencies, showing how the
    components work together to achieve the triage goal.

-   **Represents Use Case Scenarios:** This diagram effectively models a
    specific scenario or use case: "perform triage on a detected
    victim." It shows the flow of events for this behavior, making it
    easy to understand the intended interaction pattern.

-   **Highlights Control Flow:** The use of fragments like alt
    (alternatives), opt (optionals), and loop allows the diagram to
    represent conditional logic and repetition within the interaction
    sequence, which are crucial aspects of the triage process (e.g.,
    checking responsiveness, optionally getting more detail).

-   **Alignment with Component Type:** The Triage System is inherently
    an *interactive* and *process-driven* component. Its behavior is
    defined by the sequence of steps it takes and the messages it
    exchanges. A behavioral diagram that emphasizes this interaction
    sequence, like the sequence diagram, is therefore highly consistent
    with the nature of this component.
    
    
Key Performance Indicators (KPIs) for the Triage System:

.. list-table:: KPIs for Triage System
   :widths: 25 35 40
   :header-rows: 1

   * - KPI
     - Metric
     - Success Criteria
   * - 1. Consciousness Detection Accuracy
     - Correctly classify conscious vs unconscious victims via visual cues
     - ≥85% detection accuracy
   * - 2. Responsiveness Classification
     - Response detection to robot speech
     - ≥90% true positive rate for responsive victims
   * - 3. Injury Severity Estimation
     - Correct categorization of injury levels (e.g., bleeding/ suspected fracture) using color, motion, posture
     - ≥80% accuracy on test cases
   * - 4. Triage Message Accuracy
     - Correct formatting and content of published triage info
     - Matches expected content and topic
   * - 5. ROS Triage Communication Reliability
     - % of successful messages on /triage_report
     - 100% success rate on real detections
   * - 6. Latency of Evaluation + Reporting
     - Time from first input signal (image/audio) to triage report
     - ≤1.5 seconds


code
~~~~~

.. automodule:: scripts.triage_system
   :members:
   :undoc-members:
   :show-inheritance:


Structural Risk Assessment
---------------------------

.. image:: ../images/risk_assessment_diagram.png
	:width: 1500px





This document describes the detailed flow of the **Structural Risk Assessment** system, represented by the activity diagram.
The diagram is organized into five main sections: **Sensor Interface**, **Perception and Fusion**, **Risk Evaluation**, **Mobility and Re-Inspection**, and **Operator Interaction**.

Each section is responsible for a set of operations crucial to detecting, evaluating, and responding to structural risks.

Diagram Overview
~~~~~~~~~~~~~~~~~

Sensor Interface
^^^^^^^^^^^^^^^^^

- This section initiates every **100 ms** and manages the acquisition of sensor data.
- The system collects inputs from multiple sources:
  
  * Sonar (producing Range data)
  * Lidar (producing 3D Point Cloud data)
  * RGB Camera (producing RGB Images)
  * Depth Camera (producing Depth Images)
  
- After acquisition, the system validates the RGB image using a decision point:

  * If the RGB Image is invalid, an error is sent via `SendError_Camera`.
  * If valid, the image is processed.

- Simultaneously, the system checks if other sensor readings are valid:

  * If invalid, an error is sent via `SendError_Sensor`.
  * If valid, the sensor data is processed.

- Once processing is complete, the system proceeds to the next section.

Perception and Fusion
^^^^^^^^^^^^^^^^^^^^^^

- In this phase, sensor data (images, Lidar, Sonar, depth camera, and force sensor) are fused together.
- The system then classifies structural defects based on the fused data.
- A decision is made:

  * If classification fails (`isClassifyFail`), the process terminates.
  * If classification succeeds, the system advances to risk evaluation.

Risk Evaluation
^^^^^^^^^^^^^^^^^

- The system evaluates the **risk level** associated with the classified structural defect.
- Risk level and accuracy are acquired and stored in `RISK_DS`.

- Based on the evaluated risk, the system categorizes the defect into one of three levels:

  * **High Risk**
  * **Medium Risk**
  * **Low Risk**

- Each decision path ensures proper logging or alerts as necessary, followed by a transition to mobility operations.

Mobility and Re-Inspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- This section evaluates the **accuracy** of the risk evaluation.

- A decision point checks whether the current accuracy is sufficient:

  * If sufficient, an alert is sent to the operator.
  * If insufficient, the system extracts a **2D Position** from the 3D pose.

- It then assesses the reachability of the target:

  * If unreachable, an alert is sent to inform the operator.
  * If reachable, the system commands a **MoveCloser(2D Position)** action for better data collection and re-inspection.

Operator Interaction
^^^^^^^^^^^^^^^^^^^^^

- This section handles communication between the system and the human operator.
- Upon sending an alert, the system waits for operator acknowledgment:

  * If the alert is received and acknowledged, the system logs the acknowledgment.
  * If not acknowledged, the system records it as "Not Acknowledged" and ends the current interaction flow.



Key Performance Indicators (KPIs):

.. list-table:: KPIs for Structural Risk Assessment
   :widths: 25 35 40
   :header-rows: 1

   * - KPI
     - Metric
     - Success Criteria
   * - 1. Crack Detection
     - Number of cracks detected from RGB images
     - Randomized dummy value [0–4]; verified image input
   * - 2. Wall Anomaly Detection
     - Number of wall anomalies detected from LiDAR scans
     - Randomized dummy value [0–2]; valid LiDAR readings
   * - 3. Hollow Space Detection
     - Number of hollow spaces detected from sonar sensor
     - Randomized dummy value [0–2]; valid sonar readings
   * - 4. Force Magnitude Assessment
     - Computed magnitude of force from wrist force-torque sensor
     - Correct magnitude calculation; force >8N raises risk
   * - 5. Risk Score Evaluation
     - Aggregated weighted risk score based on sensor readings
     - Risk status correctly classified (LOW, MEDIUM, HIGH)
   * - 6. Risk Report Publication
     - Publishing RiskReport message on /risk_alert
     - Timely publication after evaluation
   * - 7. Manual Reassessment Handling
     - Responding to manual "reassess" requests
     - Proper trigger of reassessment procedure
   * - 8. Reassessment Movement
     - Moving robot 1m forward for closer inspection
     - Goal message sent on /move_base_simple/goal
   * - 9. Reassessment Status Notification
     - Publishing status updates on reassessment progress
     - Correct updates on /reassessment_status topic


Code
~~~~~

The full workflow enables:

* Continuous data acquisition from multiple sensors.
* Fusion of sensory information for accurate structural defect detection.
* Risk evaluation with different severity levels.
* Dynamic mobility actions to enhance inspection quality.
* Real-time interaction with human operators to ensure timely decision-making.

This systematic, cyclic approach ensures robustness, reliability, and safety in structural risk assessment scenarios.




.. automodule:: scripts.structural_risk_assessment
   :members:
   :undoc-members:
   :show-inheritance:


Victim Detection
-----------------

.. image:: ../images/victim_detection_logic.png
	:width: 1500px
	

Diagram Overview
~~~~~~~~~~~~~~~~~

**Behavioral Diagram Type:** UML State Machine Diagram

This is appropriate for modeling the state-based behavior of a system component that changes states based on events, conditions, or actions.
The state machine diagram was chosen for modeling this component because the system's behavior is fundamentally state-driven. In the victim detection task, the robot transitions between a series of well-defined operational modes ? such as Detecting Victims, Locating, and Reporting ? where each mode has specific activities, entry and exit actions, and conditions that trigger transitions to the next state.


**Description of Components:**

The state machine diagram contains two composite states with substates and one simple state. It begins with an initial pseudostate and ends with a terminate pseudostate.

**Process Overview:**

1. Victim detection using audio and visual sensors  
2. Location estimation using depth and odometry data  
3. Reporting the detected victim's location and detection alert  

**State Details:**

State 1: Detecting Victims
~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Substates:**

- Parallel processes:
  - Acquire data from RGB-D camera → Process data  
  - Acquire data from microphones → Process data  

**Transition to Next State:** Occurs if a victim is detected or sound level exceeds 500. Transitions to the *Detected victim* state.  

**Activities:**

- *On entry:* Execute ``turn_on_camera`` and ``turn_on_microphone``  
- *During state:* Perform ``acquire_sensor_data``  
- *On exit:* Call ``turn_off_camera`` and ``turn_off_microphone``  

**Transitions:**

- If ``time_out()`` event occurs: Trigger ``error_handling()`` function  

State 2: Locating
~~~~~~~~~~~~~~~~~

**Substates:**

1. **Data Collection:**
   - Acquire depth data  
   - Acquire odometry data  
2. **Location Estimation:**
   - Calculate location  
   - Verify data  
   - Send data  

**Activities:**

- *On entry:* Execute ``acquiring_sensor_data``  
- *On exit:* Call ``end_communication``  

**Transitions:**

- If ``calculation_failed()`` occurs: Perform ``calculate_again``  

State 3: Reporting
~~~~~~~~~~~~~~~~~~

**Activities:**

- *On entry:* Execute ``calculate_location``  
- *During state:* Perform ``report_location`` and ``report_alert``  
- *On exit:* Call ``verify_communication``  

**Transitions:**

- If ``no_response()`` event occurs: Execute ``send_again``  


.. list-table:: Victim Detection Key Performance Indicators (KPIs)
   :header-rows: 1
   :widths: 30 30 40
   :align: left

   * - **KPIs**
     - **Metric**
     - **Success Criteria**
   * - Victim Detection Accuracy (Visual)
     - Precision/recall in detecting humans in RGB-D images
     - ≥90% accuracy in test set
   * - Victim Detection Accuracy (Audio)
     - Correctly detect sounds
     - ≥85% true positive rate on labeled audio samples
   * - False Positive Rate
     - Percentage of false alerts (e.g., non-human objects)
     - <5% under standard conditions
   * - Location Estimation Accuracy
     - Deviation between reported and actual victim location
     - ≤0.5m deviation
   * - Alert Latency
     - Time between detection and message publication
     - ≤500ms from detection to message receipt

code
~~~~~

.. automodule:: scripts.victim_detection
   :members:
   :undoc-members:
   :show-inheritance:
   


Real Time Alerts
~~~~~~~~~~~~~~~~~


.. automodule:: scripts.real_time_alerts
   :members:
   :undoc-members:
   :show-inheritance:

