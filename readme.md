# FRC 2019 Robot Code

This is the code for Team 4585's 2019 Robot: Laika! 
Below, there are instructions to opening this repository in Eclipse and IntelliJ. 
Additionally, it has instructions on how to build and deploy the robot

## Setup Instructions

### General
- Clone this repo
- Run `./gradlew` to download gradle and needed FRC libraries
- Run `./gradlew tasks` to see available build options
- Enjoy!

#### Eclipse
- Run `./gradlew eclipse`
- Open Eclipse and go to "File > Open Projects" from "File System..."
- Set the import source to the `Laika-2019` folder then click finish

#### IntelliJ
- Run `./gradlew idea`
- Open the `Laika-2019.ipr` file with IntelliJ

### Building/Deploying to the Robot
- Run `./gradlew build` to build the code. Use the `--info` flag for more details
- Run `./gradlew deploy` to deploy to the robot in Terminal (Mac) or Powershell (Windows)

## Features
- Limelight

This project contains a class that uses the Limelight 2.0 where it centers on sight of reflective tape. This is simply done by the functions defined in the VisionController.java class in subsystems/drive.

- Gear Shifting

This project allows for a two speed shifting using a single solenoid. This is described by the Shift() function in the Drivetrain.java class in subsystems/drive.

- Closed Loop Teleop Control

Using the powerful CTRE libraries, the Talon SRXs and Victor SPXs in the drivetrain have access to closed loop control, allowing for error correction during the Teleop period. This allows for a much better experience for the drivers, who don't have to account for any unnecessary error.


## Variable Naming Conventions
- k*** (i.e. `kDriveLowGearVelocityKi`): Final constants
- m_*** (i.e. `m_subsystem`): Private instance variables
