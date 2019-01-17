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
- Set the import source to the `FRC-2018-Public` folder then click finish

#### IntelliJ
- Run `./gradlew idea`
- Open the `FRC-2018-Public.ipr` file with IntelliJ

### Building/Deploying to the Robot
- Run `./gradlew build` to build the code. Use the `--info` flag for more details
- Run `./gradlew deploy` to deploy to the robot in Terminal (Mac) or Powershell (Windows)