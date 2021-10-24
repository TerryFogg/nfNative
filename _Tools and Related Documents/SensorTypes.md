## This doco was taken from QML.

#### Looks like an interesting way to create classes that abstract the actual sensor type interface underneath



QML Sensor Types

These types represent specific types of sensors.

Note that most properties come from the abstract Sensor element.

| Accelerometer            | Element reports on linear acceleration along the X, Y and Z axes      |
|--------------------------|-----------------------------------------------------------------------|
| Altimeter                | Element reports on altitude                                           |
| AmbientLightSensor       | Element repors on ambient lighting conditions                         |
| AmbientTemperatureSensor | Element reports on the ambient temperature                            |
| Compass                  | Element reports on heading using magnetic north as a reference        |
| DistanceSensor           | Element reports the distance in cm from an object to the device       |
| Gyroscope                | Element reports on rotational acceleration around the X, Y and Z axes |
| HolsterSensor            | Element reports on whether a device is holstered                      |
| HumiditySensor           | Element reports on humidity                                           |
| IRProximitySensor        | Element reports on infra-red reflectance values                       |
| LidSensor                | Element reports on whether a device is closed                         |
| LightSensor              | Element reports on light levels using LUX                             |
| Magnetometer             | Element reports on magnetic field strength along the Z, Y and Z axes  |
| OrientationSensor        | Element reports device orientation                                    |
| PressureSensor           | Element reports on atmospheric pressure values                        |
| ProximitySensor          | Element reports on object proximity                                   |
| RotationSensor           | Element reports on device rotation around the X, Y and Z axes         |
| TapSensor                | Element reports tap and double tap events along the X, Y and Z axes   |
| TiltSensor               | Element reports tilt events along the X and Y axes                    |

QML Reading Types

The data from a sensor comes via a reading class.

| AccelerometerReading      | Element holds the most recent Accelerometer reading     |
|---------------------------|---------------------------------------------------------|
| AltimeterReading          | Element holds the most recent Altimeter reading         |
| AmbientLightReading       | Element holds the most AmbientLightSensor reading       |
| AmbientTemperatureReading | Element holds the most recent temperature reading       |
| CompassReading            | Element holds the most recent Compass reading           |
| DistanceReading           | Element holds the most recent DistanceSensor reading    |
| GyroscopeReading          | Element holds the most recent Gyroscope reading         |
| HolsterReading            | Element holds the most recent HolsterSensor reading     |
| HumidityReading           | Element holds the most recent HumiditySensor reading    |
| IRProximityReading        | Element holds the most recent IR proximity reading      |
| LidReading                | Element holds the most recent LidSensor reading         |
| LightReading              | Element holds the most recent LightSensor reading       |
| MagnetometerReading       | Element holds the most recent Magnetometer reading      |
| OrientationReading        | Element holds the most recent OrientationSensor reading |
| PressureReading           | Element holds the most recent PressureSensor reading    |
| ProximityReading          | Element holds the most recent ProximitySensor reading   |
| RotationReading           | Element holds the most recent RotationSensor reading    |
| TapReading                | Element holds the most recent TapSensor reading         |
| TiltReading               | Element holds the most recent TiltSensor reading        |

All QML Types

| Accelerometer             | Element reports on linear acceleration along the X, Y and Z axes      |
|---------------------------|-----------------------------------------------------------------------|
| AccelerometerReading      | Element holds the most recent Accelerometer reading                   |
| Altimeter                 | Element reports on altitude                                           |
| AltimeterReading          | Element holds the most recent Altimeter reading                       |
| AmbientLightReading       | Element holds the most AmbientLightSensor reading                     |
| AmbientLightSensor        | Element repors on ambient lighting conditions                         |
| AmbientTemperatureReading | Element holds the most recent temperature reading                     |
| AmbientTemperatureSensor  | Element reports on the ambient temperature                            |
| Compass                   | Element reports on heading using magnetic north as a reference        |
| CompassReading            | Element holds the most recent Compass reading                         |
| DistanceReading           | Element holds the most recent DistanceSensor reading                  |
| DistanceSensor            | Element reports the distance in cm from an object to the device       |
| Gyroscope                 | Element reports on rotational acceleration around the X, Y and Z axes |
| GyroscopeReading          | Element holds the most recent Gyroscope reading                       |
| HolsterReading            | Element holds the most recent HolsterSensor reading                   |
| HolsterSensor             | Element reports on whether a device is holstered                      |
| HumidityReading           | Element holds the most recent HumiditySensor reading                  |
| HumiditySensor            | Element reports on humidity                                           |
| IRProximityReading        | Element holds the most recent IR proximity reading                    |
| IRProximitySensor         | Element reports on infra-red reflectance values                       |
| LidReading                | Element holds the most recent LidSensor reading                       |
| LidSensor                 | Element reports on whether a device is closed                         |
| LightReading              | Element holds the most recent LightSensor reading                     |
| LightSensor               | Element reports on light levels using LUX                             |
| Magnetometer              | Element reports on magnetic field strength along the Z, Y and Z axes  |
| MagnetometerReading       | Element holds the most recent Magnetometer reading                    |
| OrientationReading        | Element holds the most recent OrientationSensor reading               |
| OrientationSensor         | Element reports device orientation                                    |
| PressureReading           | Element holds the most recent PressureSensor reading                  |
| PressureSensor            | Element reports on atmospheric pressure values                        |
| ProximityReading          | Element holds the most recent ProximitySensor reading                 |
| ProximitySensor           | Element reports on object proximity                                   |
| QmlSensors                | Singleton provides the module API                                     |
| RotationReading           | Element holds the most recent RotationSensor reading                  |
| RotationSensor            | Element reports on device rotation around the X, Y and Z axes         |
| Sensor                    | Element serves as a base type for sensors                             |
| SensorGesture             | Provides notifications when sensor-based gestures are detected        |
| SensorReading             | Element serves as a base type for sensor readings                     |
| TapReading                | Element holds the most recent TapSensor reading                       |
| TapSensor                 | Element reports tap and double tap events along the X, Y and Z axes   |
| TiltReading               | Element holds the most recent TiltSensor reading                      |
| TiltSensor                | Element reports tilt events along the X and Y axes                    |
