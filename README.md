# ESP32 Autonomous Navigation Vehicle

An ESP32-based autonomous navigation system that guides a mobile vehicle toward a predefined source location, which is the user's static co-ordinates, using coordinate-based distance evaluation, MQTT communication, and iterative direction searches.

## Overview

The vehicle receives its own coordinates and the target user's coordinates through MQTT. It calculates the distance between the two points using the Haversine formula and controls its motors to determine a direction that progressively reduces the distance to the target.

Instead of relying on a conventional navigation or path-planning algorithm, the system uses an iterative search strategy: the vehicle tests movement in different directions, evaluates the resulting change in distance, selects the best-performing heading, and continues moving along that heading until a better direction is found which is iteratively found.

## Key Features

- ESP32-based autonomous vehicle
- MQTT for sharing vehicle and target coordinates
- Distance calculation using the Haversine formula
- Direction search based on changes in distance to the target
- Clockwise and counter-clockwise movement testing
- Saves the best direction found during the search
- Repeats the search if the vehicle starts moving away from the target
- Configurable destination threshold
- MQTT communication maintained while the motors are running

## System Architecture

```text
                Target / User
             |
             | Target Coordinates
             v
        MQTT Broker
             |
             v
          ESP32
        /       \
       /         \
  AGV Coordinates  Navigation Algorithm
       |                 |
       +---------> Distance Calculation
                         |
                         v
                  Direction Selection
                         |
                         v
                    Motor Control
                         |
                         v
                   Autonomous Vehicle
                         |
                         | Updated Position
                         v
                  Distance Evaluation
                         |
                  ┌──────┴──────┐
                  |             |
              Improving      Worsening
                  |             |
                  v             v
             Continue       Iterative
             Forward         Direction
                             Search
                                |
                                └──────> Direction Selection
