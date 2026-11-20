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

## Usage

### Configuration

- To configure the pipeline for your robot, refer to the `config` folder.
- Create a folder for your robot in that folder, e.g. `myrobot`.
- `config/myrobot/nav2_params.yaml` should contain the Nav2 configuration for your robot.
- `config/myrobot/anytraverse.yaml` contains the AnyTraverse configuration.
- `config/myrobot/pipelines` contains presets for the AnyTraverse ROS2 wrapper
- While running for the first time, ensure the models for AnyTraverse are downloaded from Huggingface by disabling `TRANSFORMERS_OFFLINE` and `HF_HUB_OFFLINE` and starting the `anytraverse` service (This may fail to publish any topics if camera node is not active, but that is alright).
  ```bash
  HF_HUB_OFFLINE=0 TRANSFORMERS_OFFLINE=0 docker compose up anytraverse
  ```
- The models are stored in a Docker volume which is mounted automatically on every `docker compose up`. Subsequent runs do not need the offline flags.

> For more information on configuring AnyTraverse ROS2 parameters, refer to [`sattwik-sahu/anytraverse_ros`](https://github.com/sattwik-sahu/anytraverse_ros)

### Quick Start

By default, the project supports starting up the basic Docker Compose services required to run the experiments:
- `anytraverse`: The AnyTraverse perception pipeline
- `navigation`: The Nav2 navigation stack
- `zenoh`: The Zenoh router to ensure communication between the containers

Start the entire stack with
```bash
docker compose up
```

> [!TIP]
> Additionally, the project comes with an option to visualize your data with Foxglove. This requires starting the Foxglove bridge service with Docker Compose by enabling the `viz` profile. You can do so with
> ```bash
> docker compose --profile viz up anytraverse navigation up
> ```
>
> After starting the Foxglove bridge, you could connect to your robot's computer and visualize the topics by going to [`app.foxglove.dev`](https://app.foxglove.dev/) and connecting to websocket `<myrobot-ip>:8765`
