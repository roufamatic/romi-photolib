# Romi and PhotonLib

A small exploration of using [PhotonVision](https://photonvision.org) with a [Romi](https://docs.wpilib.org/en/stable/docs/romi-robot/index.html) robot.

Files of interest:

* [Robot.java](src/main/java/frc/robot/Robot.java): Main entry class, includes Commands setup
* [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java): The "glue" holding the robot together.
* [AprilTagRoundRobinFinder.java](src/main/java/frc/robot/commands/AprilTagRoundRobinFinder.java): Example showing AprilTag detection with [PhotonLib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/index.html), the client library for PhotonVision.
* [Drivetrain.java](src/main/java/frc/robot/subsystems/Drivetrain.java): The software <-> hardware wiring, encapsulated in a Subsystem.

I also recommend looking at some of the other [Commands](src/main/java/frc/robot/commands) to see how they can simplify robot logic.