# Repository Guidelines

## Project Structure & Module Organization
`src_robot/` contains robot control code. Use `robot_main/` for runtime entry points and configuration, `controller/` for joystick and kinematics logic, `leadshine/` and `fastech/` for AGV and servo drivers, and `ros2/` for ROS2 integration assets. `vision/` contains the perception stack: `calibration/`, `dataset_gen/`, and `yolo/` for training, evaluation, and inference scripts. Generated outputs belong in `result/` and `runs/`; keep large artifacts and one-off experiments out of source folders when possible.

## Build, Test, and Development Commands
Use Python 3.10 and `uv` from the repository root.

- `uv sync` installs the locked dependencies from `pyproject.toml` and `uv.lock`.
- `uv run python src_robot/robot_main/main.py` starts the robot control entry point.
- `uv run python vision/yolo/train_coin_pose.py --help` shows training options for the current COIN-Pose pipeline.
- `uv run python vision/yolo/yolo_backbone.py --mode eval --data_root <path>` runs backbone evaluation.
- `uv run python vision/test_vision.py` runs the ROS2 vision publisher script for local/manual checks.

## Coding Style & Naming Conventions
Follow existing Python conventions: 4-space indentation, `snake_case` for functions and modules, `PascalCase` for classes, and descriptive constant names in `UPPER_SNAKE_CASE` when needed. Keep hardware-facing changes narrow and avoid broad refactors in legacy driver folders such as `src_robot/fastech/`. Prefer small, script-friendly modules and keep task/state-machine orchestration in wrapper modules such as `suprevisor.py` and `tasks.py`.

## Testing Guidelines
This repository does not currently use a formal `pytest` suite. Validation is script-based and hardware-dependent. For vision changes, run the relevant training or evaluation script and record dataset path, weights, and output directory. For robot changes, validate with the smallest safe entry point first, then run `src_robot/robot_main/main.py` only when controller, ROS2, and motor settings are correct.

## Commit & Pull Request Guidelines
Recent history shows short, inconsistent commit subjects. Going forward, use concise imperative messages such as `vision: tune COIN occlusion recovery loss` or `robot: fix joystick shutdown path`. Keep each commit scoped to one change. Pull requests should include a clear summary, affected modules, manual test steps, and screenshots or logs for vision results when behavior changes.

## Security & Configuration Tips
Do not commit secrets, device-specific IP changes, or generated weights unless intentionally versioned. Review `src_robot/fastech/servolist.py`, controller mappings, and ROS2 message dependencies before running on real hardware.
