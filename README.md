# AnyTraverse ROS2 Containers

This project provides a containerized solution to run AnyTraverse experiments using ROS2 and Docker Compose.

## Installation

> [!NOTE]
> Install Docker and Docker compose before continuing with the setup

1. Clone the repo to the your machine
2. Recursively initialize the submodules
  ```bash
  git submodule update --init --recursive
  ```
3. Build the basic images required
  ```bash
  docker compose build anytraverse zenoh navigation
  ```

