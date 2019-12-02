<h1 align="center"> TerpBotics: Enigma
</h1>
ENPM808X-Final Project

[![Build Status](https://travis-ci.org/sandeep-kota/TerpBotics-Enigma.svg?branch=week_1)](https://travis-ci.org/sandeep-kota/TerpBotics-Enigma)
[![Coverage Status](https://coveralls.io/repos/github/sandeep-kota/TerpBotics-Enigma/badge.svg?branch=week_1)](https://coveralls.io/github/sandeep-kota/TerpBotics-Enigma?branch=week_1)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

Enigma is a simulation of a material handling robot that is capable of transporting packages from a pickup spot to a drop-off location within confined environments. In general, robots are a great alternative to perform labor-intensive tasks like material handling, especially in places like warehouses and industrial environments where large amounts of goods are required to be moved from one station to another. Other potential use cases of such automated systems could be in the handling of materials in places like hospitals or personal spaces. The current simulation is run using a TurtleBot robot, but it can be extended to any differential drive mobile robot catering to the payload requirements. This project is expected to be delivered to Acme Robotics within a timeframe of three weeks.


## Team Members

- Sandeep Kota Sai Pavan - [Github Link](https://github.com/sandeep-kota)
- Satyarth Praveen - [Github Link](https://github.com/satyarth934)

## Personnel

 - Sandeep Kota Sai Pavan (Email: skotasai@umd.edu, LinkedIn: [Link](https://www.linkedin.com/in/sandeepkota341997/)): A graduate student of Masters in Engineering - Robotics at University of Maryland, College Park. I'm a robotics enthusiast interested in software development for perception applications in autonomous vehicles.

  - Satyarth Praveen (Email: satyarth@umd.edu, LinkedIn: [Link](https://www.linkedin.com/in/satyarth934/)): I am currently a Robotics graduate student and a Graduate Teaching Assistant with the Department of Mathematics at the University of Maryland College Park. I was previously employed at Hi-Tech Robotic Systemz where I was involved with the perception group of the Unmanned Ground Vehicles (autonomous vehicles). I find my interest in Artificial Intelligence.

## Development using Agile Iterative Process (AIP) and Test-Driven Development

An agile iterative process (AIP) is followed for the development of this project where the product backlog is developed first. Based on the priority of the tasks, the sprint cycles and tasks are decided. A daily meeting at the beggining of the sprint cycle is conducted to decide on the daily tasks for each developer. The project backlog consists of the estimated time of completion which is alloted for each task. The actual time of completion is altered based on the progress of the project and the remaining tasks.

After planning the product backlog, UML flow diagram and UML class diagram for the software are created. A set of unit tests are used to verify the performance on a wide range of exple scenarios. Stub implementations are written with the functions to ensure that the code coverage of the software is maintained.

The link for the product backlog, time log, error log and release log can be found here - [link](https://docs.google.com/spreadsheets/d/1_W5MeEY2wuFKOgrL02KUvVZgGzwXJrWAY3AVcIMq8zY/edit#gid=241005242) 

The link for the sprint review document can be found here - [link](https://docs.google.com/document/d/1372jK7DAAn1wwATMmXONC1Ds_ZeT41WqeLHxTDLHxzQ/edit?usp=sharing)

## Project Overview

This project is a simulation of a material handling robot in a gazebo environment of a warehouse.

## Run commands

Launch the Gazebo world, gmapping node, and the walker node:
```
roslaunch terpbotics_enigma walker.launch
```

Command to save the generated map:
```
rosrun map_server map_saver -f /map/warehouse
```
This command saves two files named `warehouse.yaml` and `warehouse.pgm`

This map can later be used by other modules / nodes for further applications.                                  

## License

This software is released under the [BSD 3-Clause license](./LICENSE). 
