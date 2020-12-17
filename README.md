# laser_distance
The aim is to collect obstacle distance in all 360 deg directions

* Infra: LAN <-> Raspberry PI 3/ LCD 1602 <-> SERIAL USB <-> RPLIDAR A1 M8 (Laser range scanner)

* Software/design: MQTT <-> PROTOBUF <-> PYTHON <-> ADAFRUIT CircuitPython RPLIDAR

Robot is moving via stepper motors, at any time there is a need to know obstacle distance in 4 directions (at least) North, East, South and West. RPLIDAR is offering this 360 deg scan with precise meter results. Small blue LCD embarked with the robot is showing X,Y distances.
