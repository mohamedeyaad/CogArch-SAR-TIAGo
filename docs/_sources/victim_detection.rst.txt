Victim Detection
================

.. image:: ../images/victim_detection_logic.png
	:width: 1500px
	

Diagram Overview
----------------

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
~~~~~~~~~~~~~~~~~~~~~~~~~~

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
   * - Localization Accuracy
     - Deviation between robot’s reported and actual location
     - ≤0.5m deviation

code
--------

.. automodule:: scripts.victim_detection
   :members:
   :undoc-members:
   :show-inheritance:
   

