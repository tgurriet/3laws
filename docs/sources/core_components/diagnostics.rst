Robot Diagnostic
################

.. warning::
  The Robot Diagnostic Module (RDM) is currently in **experimental** phase. 3Laws is actively working on expanding and maturing its capabilities.

.. contents:: Table of Contents
  :local:



The Supervisor's RDM is a tool for monitoring the health and safety of various critical robot sub-systems. It is designed to work in tandem with the Supervisor's Run-time Assurance Module (RAM).

As explained :doc:`here <runtime_assurance>`, the RAM relies on various critical robot sub-systems to perform collision avoidance and safety functions. The fault Management part of the RAM is able to detect obvious faults in these sub-systems, like timeouts or corrupted messages,
but it is not able to detect more subtle issues that may arise like inconsistent localization or a faulty sensor. The RDM fills this gap by providing a more thorough and holistic monitoring of critical components to ensure collision-free behavior.

.. warning::
  At the moment, the diagnostic performed by the RDM is not used by the RAM's Fault Management system. The user is repsonible for deciding how to use the information provided by the RDM.


Configuration
==============
In order for the RDM to perform its analysis, it must be fed data about the system. The Control Panel allow the user to configure some extra parameters specific to the RDM in addition with the one already used by the RAM (Run-time Assurance Module).


Metric Modules
==============

The RDM is composed of a set of **metric modules** each of which responsible for analyzing a specific aspect of the robot's health. Each module then streams a set of **metrics**.

.. important::
  Please refer to the :ref:`reference_ros_topics` page for the list of ROS topics where these metrics are published.

.. _contact 3Laws: mailto:sales@3laws.io

.. note::
  The RDM is designed to be modular and extensible. For more information on what other modules and metrics 3Laws can offer, please `Contact 3Laws`_.

The following modules are currently available:

Clock Health
------------

A properly functioning robot should have consistent clocks across all its components. The Clock Health module monitors the synchronization between `system clock <https://en.cppreference.com/w/cpp/chrono/system_clock>`_ of the computer the Supervisor is running on and an `NTP server <https://www.ntppool.org/>`_. Inconsistent clocks can lead to a variety of issues, including inaccurate, improper timeout and communication detection by the RAM's Fault Management system.

This module publishes the following metrics:

- **clock_health_utc**: The offset between the system clock and the NTP server in seconds. The important fields of the associated message are:

  - **has_utc**: A boolean indicating if the NTP server time is available.

  - **utc_time**: The latest time returned by the NTP in nanoseconds since 1970.

  - **rtc_time**: The latest time returned by the System Clock in nanoseconds since 1970.

  - **offset_from_utc**: The offset between the NTP server and the System Clock in nanoseconds.

Odometry Consistency
--------------------

The Odometry Consistency module is responsible for monitoring the consistency between the robot's odometries and the robot's localization. Inconsistencies between the two is a sign of a problem in the robot's and can lead to collisions.

This module publishes the following metrics:

- **max_linear_position_difference**: The maximum difference between the robot's odometries and the robot's localizations data in the linear position in meters.
- **max_angular_position_difference**: The maximum difference between the robot's odometries and the robot's localizations data in the angular position in rad.
- **max_linear_velocity_difference**: The maximum difference between the robot's odometries and the robot's localizations data in the linear velocity in m/s.
- **max_angular_velocity_difference**: The maximum difference between the robot's odometries and the robot's localizations data in the angular velocity in rad/s.

- **linear_position_difference_bound**: The maximum allowed difference between the robot's odometries and the robot's localizations data in the linear position.
- **angular_position_difference_bound**: The maximum allowed difference between the robot's odometries and the robot's localizations data in the angular position.
- **linear_velocity_difference_bound**: The maximum allowed difference between the robot's odometries and the robot's localizations data in the linear velocity.
- **angular_velocity_difference_bound**: The maximum allowed difference between the robot's odometries and the robot's localizations data in the angular velocity.

- **odometry_data**: It's the list of detail differences between each robot's odometries and the robot's localizations data.
  - **odometry_id**: The id of the odometry.

  - **status**: The status of the odometry. The possible values are: **[ok, bad, incomparable, uncertain]**

  - **consistency_score**: The score of the odometry consistency, it's the number of possible configuration where this sensor is in a valid state.

  - **differences**: list of all differences between the odometry and the other comparable localizations data.

  - **max_linear_position_difference**: The maximum difference between the current odometry (see odometry_id) and the other comparable localizations data in the linear position in meters.

  - **max_angular_position_difference**: The maximum difference between the current odometry (see odometry_id) and the other comparable localizations data in the angular position in rad.

  - **max_linear_velocity_difference**: The maximum difference between the current odometry (see odometry_id) and the other comparable localizations data in the linear velocity in m/s.

  - **max_angular_velocity_difference**: The maximum difference between the current odometry (see odometry_id) and the other comparable localizations data in the angular velocity in rad/s.


Dynamic Consistency
----------------------

The RAM (Run-time Assurance Module) relies on a quantitative understanding of the robot's physical behavior and control capabilities (i.e. a `Dynamical Model of the system <https://en.wikipedia.org/wiki/Dynamical_system>`_) to make decisions about the appropriate collision avoidance strategy. The Dynamic Consistency module is responsible for monitoring the consistency between the system's Dynamical Model and the observed system behavior. Inconsistencies between the two implies the RAM may not be able to make accurate decisions regarding when to intervene and might lead to collisions.

This module publishes the following metrics:

- **dynamic_consistency**: Consistency between the Dynamical Model of the system and the observed system behavior. The important fields of the associated message are:

  - **state_timeout**: A boolean indicating if the state information has timed out.

  - **input_timeout**: A boolean indicating if the input information has timed out.

  - **input_domain_value**: The signed distance from the current input to the boundary of the input constraints set.

  - **state_domain_value**: The signed distance from the current state to the boundary of the state domain set.

  - **time_vector**: The time vector of the integration.

  - **state_difference_1sigma**: List of the signed distance from the value of the norm of xdot_difference to the 1sigma level set of the pdf.

  - **state_difference_2sigma**: List of the signed distance from the value of the norm of xdot_difference to the 2sigma level set of the pdf.

  - **state_difference_3sigma**: List of the signed distance from the value of the norm of xdot_difference to the 3sigma level set of the pdf.

  - **state_difference_matrix**: List of each state difference between computed and actual for each integration time step described in the time_vector

  - **state_difference_pdf_value**: List of the pdf value of the state difference vector.

  - **state_difference_pdf_value_normalized**: List of the normalized pdf value of the state difference vector.

  - **system_degradation_probability**: The probability of the system degradation (experimental).

.. important::
  The process covariance matrix used for the statical analysis is currently the identity matrix.


System Health
---------------

Typical autonomy stacks are composed of **multiple Systems** (One per computer) than run **nodes** (One per process), each responsible for a specific task than each can publish data on different topics (One per data type). The System Health module monitors the health of this tree by checking a number of information. This module is useful for detecting issues like node crashes or communication loses between nodes. This is critical for nodes like Localization and Perception that are central to the RAM's proper operation.

.. warning::
  The current version of the Supervisor support monitoring of only one system (with any number of nodes).

A node is defined by the set of topics it publishes and eventually a process ID (OS pid or command line string). The Node Health module monitors the health of the nodes by checking if the topics are being published and if the data is consistent with the expected rate and values.

The associated metric is published at 1hz, and is an aggregate of the signal data received over that period.

This module publishes the following metrics:

- **system_health**: The health of the system running the Supervisor. The important fields of the associated message are:

  - **cpu_load**: The CPU usage of the system in percentage.

  - **ram_usage**: The RAM usage of the system in percentage.

  - **disk_usage**: The used disk space of the system in percentage of total capacity.

  - **network_read**: The network read usage of the system in bytes/s.

  - **network_write**: The network write usage of the system in bytes/s.

  - **cpu_nb**: The number of CPUs available on the system.

  - **procs_nb**: The number of processes running on the system.

  - **nodes**: The health of the nodes in a list in the system. The important fields of the associated message are:

    - **timeout**: A boolean indicating if all the nodes's topics have timed out.

    - **ok**: A boolean indicating that none of the nodes's topics have timed out.

    - **error_code**: An enum indicating the type of error that occurred. The possible values are: **[ok, some_topics_timeout, out_of_bounds, all_topics_timeout]**

    - **process**: Process related data:

      - **status**: The status of the process. The possible values are: **[AMBIGUOUS, RUNNING, DEAD]** (AMBIGUOUS means that the process is not found or multiple are found based on the provided string).

      - **cpu_usage**: The CPU usage of the process in percentage.

      - **cpu_usage_total**: The total CPU usage of the process in percentage.

      - **ram_usage**: The RAM usage of the process in percentage.

      - **n_threads**: The number of threads of the process.

      - **process_up_time**: The time the process has been up (CPU time used) in seconds.

      - **process_run_time**: The time the process has been running in seconds.

      - **io_total_read**: The total read IO of the process in bytes.

      - **io_total_write**: The total write IO of the process in bytes.

      - **io_disk_read**: The disk read IO of the process in bytes.

      - **io_disk_write**: The disk write IO of the process in bytes.

    - **topics**: A list of information for each of the node's topics. Each topic message contains the following important fields:

      - **topic_id**: The topic's name/identifier.

      - **timeout**: A boolean indicating if the topic has timed out.

      - **has_timestamp**: A boolean indicating if the topic's data has a non-zero timestamp.

      - **sender_rate**: The average rate at which the topic is being published by the node.

      - **receiver_rate**: The average rate at which the topic is being received by the RDM.

      - **delay**: The average delay between the time the message was sent and the time it was received.

      - **sample_size**: The number of samples received within the aggregation period.

      - **wrong_size**: A boolean indicating if the signal received has an unexpected size.

      - **bad_timestamp**: A boolean indicating if the signal received has a bad timestamp, i.e. a timestamp equal to 0.

      - **has_nan**: A boolean indicating if the signal received has NaN values.

      - **has_infinity**: A boolean indicating if the signal received has infinity values.

      - **has_zero**: A boolean indicating if the signal received has values exactly equal to `+0.f`

      - **has_subnormal**: A boolean indicating if the signal received has subnormal values.


Sensor characterization
------------------------

The Sensor Characterization module is responsible for monitoring the health of the robot's sensors. This module is useful for detecting issues like sensor malfunctions, or calibration issues. The nominal functioning of the robot's sensors is obviously critical for the RAM's proper operation. If the sensors are not functioning properly, the RAM will make decision based on improper assumption on obstacles location and may lead to collisions.

This module publishes the following metrics:

- **sensor_obstruction**: The list of sensor obstructions detected for the specified 2D laserscans. An obstruction is detected when a set of rays returns the same distance over time even through the robot is moving. The important fields of the associated message are:

  - **nb_obstructions**: The number of obstructions detected.

  - **obstructions**: A list of the detected obstruction clusters. Each obstruction message contains the following important fields:

    - **start_angle**: The start angle of the obstruction in radians in the laserscan frame.

    - **end_angle**: The end angle of the obstruction in radians in the laserscan frame.

    - **min_dist**: The minimum distance of the rays points in that obstruction cluster in meters.

    - **max_dist**: The maximum distance of the rays points in that obstruction cluster in meters.

    - **type**: The type of the obstruction. The possible values are: **[near, middle, far]**. Near obstructions are detected when the rays are below the minimum range configured, far obstructions are detected when the rays are beyond the maximum range configured, and middle obstructions are detected in between.

- **sensor_noise**: This metric presents statistics on the noise characteristics for the sensors. The important fields of the associated message are:

  - **average_std_error**: The standard deviation of all the sensor measurements over a 1 second window.

  - **max_std_error**: The maximum deviation of all the sensor measurements over a 1 second window.

  - **angle_max_error**: The angle at which the maximum deviation occurred.

  - **percent_of_sigma**: Signed distance from the value of the average_std_error to the 1-sigma level set of the pdf.

  - **p_value**: The p-value of the sensor noise consistency test.

  - **reject_model**: A boolean indicating that the sensor noise statistics are not consistent with the observed noise.


.. important::
  Currently, the expected sensor measurement covariance for 2D laserscans is 1m.



Incident Diagnostic
-------------------

The diagnostic module aggregates information published by the various metric modules and provides a high-level view of the robot's health. This module is useful for detecting issues that may not be apparent when looking at the individual metrics.

The following metrics are published:

- **domain_status**: The high-level status of the health of the various robot components.

  - **overall**: The overall status of the robot. The possible values are: **[ok, minor, severe, critical]**

  - **motion_planning**: The status of the robot's motion planning stack. The possible values are: **[ok, minor, severe, critical]**

  - **localization**: The status of the robot's localization stack. The possible values are: **[ok, minor, severe, critical]**

  - **perception**: The status of the robot's perception stack. The possible values are: **[ok, minor, severe, critical]**

  - **hardware**: The status of the robot's hardware. The possible values are: **[ok, minor, severe, critical]**

  - **compute**: The status of the robot's compute resources. The possible values are: **[ok, minor, severe, critical]**

  - **network**: The status of the robot's network. The possible values are: **[ok, minor, severe, critical]**

  - **sensors**: The status of the robot's sensors. The possible values are: **[ok, minor, severe, critical]**


- **incidents_log**: A stream of incident logs. The important fields of the associated message are:

  - **name**: The name of the incident.

  - **detail**: The details of the incident.

  - **in_progress**: A boolean indicating if the incident is still in progress.

  - **domain**: The domain of the incident. The possible values are: **[behavior, system, hardware, perception, control]**


Output modules
==============

The RDM uses **output modules** to make the metrics available to the user. Currently, the RDM ships with a single output module to publish all metrics to :ref:`reference_ros_topics`.

.. note::
  The RDM's design is modular and extensible for applications that have more specific monitoring needs. For more information on what other types of output modules 3Laws can offer, please `Contact 3Laws`_.
