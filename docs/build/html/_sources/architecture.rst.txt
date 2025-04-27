System Architecture
===================

.. image:: ../images/component_diagram.png
   :alt: System Architecture
   :width: 600px
.. list-table:: Design Patterns Table for Subsystems
   :widths: 20 20 60
   :header-rows: 1

   * - Subsystem
     - Design Pattern
     - Description
   * - TIAGo Robot
     - Singleton
     - Ensures motor controllers (left/right) are single instances to prevent conflicting actuator commands.
   * - 
     - Facade
     - Provides a simplified interface for controlling multiple low-level hardware components like encoders, microphones, and motors.
   * - 
     - Adapter
     - Bridges different hardware interfaces to a common control abstraction layer used by higher-level systems.
   * - Perception
     - Adapter
     - Translates diverse sensor outputs (LiDAR, RGB-D, SONAR) into a unified format for SLAM or risk analysis.
   * - 
     - Observer
     - Notifies other subsystems (e.g., navigation, risk assessment) when new perception data is available.
   * - 
     - Decorator
     - Dynamically adds processing layers (e.g., filtering, segmentation, labeling) to sensor data streams.
   * - Navigation
     - Strategy
     - Allows dynamic switching of navigation algorithms depending on environmental conditions (e.g., debris-filled vs. open space).
   * - 
     - State
     - Manages operational modes such as idle, exploring, relocating, or returning, each with its own navigation behavior.
   * - Structural Risk Assessment
     - Observer
     - Reacts to sensor updates and triggers reclassifications when changes are detected.
   * - 
     - Command
     - Encapsulates risk-based decisions into actions (e.g., move closer, send report) for flexible execution.
   * - Victim Handling
     - State
     - Captures the workflow from detection to triage reporting through clear state transitions.
   * - 
     - Command
     - Encapsulates reporting or alerting of victim detection as discrete, reusable commands.
   * - 
     - Strategy
     - Permits runtime selection among various triage evaluation techniques (e.g., sound, motion, interaction).
   * - 
     - Observer
     - Allows real-time systems (e.g., communication) to subscribe to updates when a victim is found or classified.
   * - Communication
     - Observer
     - Distributes alerts and mission updates to subscribed interfaces like the GUI or external operator systems. Reports or sends alert when victim detected.
   * - 
     - Mediator
     - Centralizes control of incoming/outgoing messages to simplify interaction across subsystems.
   * - 
     - Command
     - Allows user-initiated commands (e.g., request deeper inspection) to be encapsulated and processed asynchronously.


.. list-table:: TIAGo Robot Components
   :widths: 25 25 15 15 20 50
   :header-rows: 1

   * - Component
     - Interface
     - Stateless/Stateful
     - Data/Service
     - Type
     - Notes
   * - Left_Motor / Right_Motor
     - Receive motor commands
     - Stateful
     - Service
     - Strongly-typed
     - Motor speed and direction depend on previous commands.
   * - Left_Motor_Controller / Right_Motor_Controller
     - Control motor behavior
     - Stateful
     - Service
     - Strongly-typed
     - Commands affect internal motor state.
   * - Left_Encoder / Right_Encoder
     - Provide odometry data
     - Stateless
     - Data
     - Strongly-typed
     - Just outputs current readings without memory.
   * - Battery
     - Provide battery status
     - Stateless
     - Data
     - Strongly-typed
     - Provides current charge level.
   * - Microphones
     - Stream audio data
     - Stateless
     - Data
     - Strongly-typed
     - Raw audio data stream.
   * - Speakers
     - Play alert sounds or voice
     - Stateless
     - Service
     - Loosely-typed
     - Sends flexible audio/text input.
   * - Force_Sensor
     - Provide contact force information
     - Stateless
     - Data
     - Strongly-typed
     - Real-time force measurements.


.. list-table:: Perception Components
   :widths: 25 25 15 15 20 50
   :header-rows: 1

   * - Component
     - Interface
     - Stateless/Stateful
     - Data/Service
     - Type
     - Notes
   * - LiDAR
     - Provide laser scans
     - Stateless
     - Data
     - Strongly-typed
     - Standardized point cloud output.
   * - RGB-D Camera
     - Provide RGB image + depth map
     - Stateless
     - Data
     - Strongly-typed
     - Depth image and RGB frame synchronized.
   * - Sensor
     - Provide range data (Sonar)
     - Stateless
     - Data
     - Strongly-typed
     - Outputs distance to obstacles.


.. list-table:: Navigation Components
   :widths: 25 25 15 15 20 50
   :header-rows: 1

   * - Component
     - Interface
     - Stateless/Stateful
     - Data/Service
     - Type
     - Notes
   * - SLAM
     - Get current location, update 3D map
     - Stateful
     - Service
     - Strongly-typed
     - Maintains internal map and robot pose.
   * - Path_Planning
     - Generate paths to a target
     - Stateless
     - Service
     - Strongly-typed
     - Path calculation based on map and goals.
   * - Navigation_Controller
     - Manage movement commands
     - Stateful
     - Service
     - Strongly-typed
     - Switches between moving, avoiding, recalculating.



.. list-table:: Structural Risk Assessment Components
   :widths: 25 25 15 15 20 50
   :header-rows: 1

   * - Component
     - Interface
     - Stateless/Stateful
     - Data/Service
     - Type
     - Notes
   * - Sensor_Fusion
     - Fuse LiDAR, RGB-D, SONAR data
     - Stateless
     - Data
     - Strongly-typed
     - Combines different sources into a coherent structure.
   * - Cracks_Detection
     - Analyze images for cracks
     - Stateless
     - Service
     - Strongly-typed
     - Image input, detection results output.
   * - Image_Processing
     - Preprocess sensor images
     - Stateless
     - Service
     - Strongly-typed
     - Filtering, noise reduction etc.
   * - Risk_Classifier
     - Assess wall risk levels
     - Stateless
     - Service
     - Strongly-typed
     - Risk level classification based on sensor fusion.
   * - Weak_Walls
     - Store identified weak points
     - Stateful
     - Data
     - Strongly-typed
     - Maintains a database of risk points.



.. list-table:: Victim Handling Components
   :widths: 25 25 15 15 20 50
   :header-rows: 1

   * - Component
     - Interface
     - Stateless/Stateful
     - Data/Service
     - Type
     - Notes
   * - Victim_Detection
     - Detect injured individuals
     - Stateless
     - Service
     - Strongly-typed
     - Processes sensor data for victim detection.
   * - Speech_to_Text
     - Convert speech input to text
     - Stateless
     - Service
     - Loosely-typed
     - Flexible natural language handling.
   * - Triage_Classifier
     - Assess victim condition
     - Stateless
     - Service
     - Strongly-typed
     - Classifies consciousness/responsiveness levels.
   * - speaker_service.py
     - Trigger audio alerts or instructions
     - Stateless
     - Service
     - Loosely-typed
     - Text-to-speech synthesis.

.. list-table:: Communication Components
   :widths: 25 25 15 15 20 50
   :header-rows: 1

   * - Component
     - Interface
     - Stateless/Stateful
     - Data/Service
     - Type
     - Notes
   * - Mission_Reports
     - Send structural damage/victim reports
     - Stateful
     - Service
     - Loosely-typed
     - Flexible message composition (text, severity levels).
   * - Real_Time_Alerts
     - Push emergency updates
     - Stateless
     - Service
     - Loosely-typed
     - Asynchronous alert delivery to operator.
   * - GUI
     - Display alerts and mission information
     - Stateless
     - Data
     - Loosely-typed
     - Flexible UI rendering.


