Integration Testing KPIs and Results
=====================================

The integration testing process aims to ensure that the system components interact correctly to meet the defined performance and functional requirements. Below is a summary of the *Key Performance Indicators (KPIs)* used for testing the integration of various subsystems and the corresponding *test results*.

---

1. Sensor Data Availability
----------------------------

- *KPI*: All sensors (LiDAR, SONAR, RGB-D Camera, Force Sensors, Microphones) must produce data within 20 seconds of initialization.
- *Test*: **test_sensor_data_flow**
- *Result*: All sensors were verified to be active and producing data within the specified time frame. No missing or delayed data was observed.

---

2. Navigation Pipeline Validation
----------------------------------

- *KPI*: 
    - SLAM must produce pose and map data within 30 seconds.
    - Path planning must generate valid paths.
    - Movement commands must be issued within 20 seconds.
    - Mission completion must be detected within 60 seconds.
- *Test*: **test_navigation_chain**
- *Result*: 
    - SLAM outputs (pose and map) were successfully validated within 30 seconds.
    - Path planning generated valid paths with poses.
    - Movement commands (cmd_vel) were issued as expected.
    - Mission completion was detected, and success reports were logged within the required time frame.

---

3. Victim Detection and Triage Assessment
------------------------------------------

- *KPI*: 
    - Victim detection must trigger triage assessment within 40 seconds.
    - Triage reports must include valid priority levels (URGENT, STABLE) and victim location.
- *Test*: **test_victim_processing_chain**
- *Result*: 
    - Victim detection successfully triggered triage assessment.
    - Triage reports were validated, with correct priority levels (URGENT, STABLE) and location data.

---

4. Structural Risk Assessment
------------------------------

- *KPI*: 
    - Risk alerts must be generated within 30 seconds.
    - Alerts must include valid risk levels (LOW, MEDIUM, HIGH), crack counts, wall anomalies, hollow spaces, force magnitude, and risk score.
- *Test*: **test_risk_assessment_flow**
- *Result*: 
    - Structural risk alerts were generated and validated.
    - All fields in the RiskReport message were confirmed to be within expected ranges, including risk level, crack count, wall anomalies, and force magnitude.

---

5. Real-Time Responsiveness
----------------------------

- *KPI*: The system must respond to real-time conditions and produce outputs within specified time limits.
- *Test*: **All tests using wait_for_condition**
- *Result*: The system demonstrated real-time responsiveness, consistently meeting the specified timing constraints across all tests. The system responded promptly to changes in conditions, as verified by all individual subsystems.

---


.. automodule:: tests.integration_test
   :members:
   :undoc-members:
   :show-inheritance: