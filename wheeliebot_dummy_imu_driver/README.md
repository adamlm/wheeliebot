# WheelieBot Dummy IMU Driver

This package contains a dummy IMU driver. The implementation aims to conform with
[ROS Enhancement Proposal (REP) 145: *Conventions for IMU Sensor Drivers*](https://www.ros.org/reps/rep-0145.html).

## Interface

### Subscribers

This package does not provide subscribers.


### Publishers

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| `~/imu/data_raw` | `sensor_msgs/Imu` | 20 Hz | Sensor output grouping accelerometer (`linear_acceleration`) and gyroscope (`angular_velocity`) data. |
| `~/imu/data` | `sensor_msgs/Imu` | 20 Hz | Same as `~/imu/data_raw`, with an included quaternion orientation estimate (`orientation`). |
| `~/imu/mag` | `sensor_msgs/MagneticField` | 20 Hz | Sensor output containing magnetometer data. |

If a data field's covariances are unknown, the covariance matrix's elements will all be `0` unless overridden by a
parameter. If a data field is unreported, its covariance matrix's zeroth element will be `-1`.

### Parameters

| Topic | Data Type | Default Value | Description |
|-------|-----------|---------------|-------------|
| `~/frame_id` | `string` | `imu_link` or `imu_link_ned` | The frame ID to set in outgoing messages. |
| `~/linear_acceleration_stddev` | `double` | | Square root of the `linear_acceleration_covariance` diagonal elements in meters per second squared (m/s^2). Overrides any values reported by the sensor. |
| `~/angular_velocity_stddev` | `double` | | Square root of the `angular_velocity_covariance` diagonal elements in radians per second (rad/s). Overrides any values reported by the sensor. |
| `~/magnetic_field_stddev` | `double` | | Square root of the `magnetic_field_covariance` diagonal elements in teslas (T). Overrides any values reported by the sensor. |
| `~/orientation_stddev` | `double` | | Square root of the `orientation_covariance` diagonal elements in radians (rad). Overrides any values reported by the sensor. |


### Services

This package does not provide services.


### Actions

This package does not provide actions.
