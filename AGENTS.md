# dog_final project rules

- This is the separately named organization release `dog_final_20261030`.
- All project ROS node names must start with `dog_`; new interfaces must use dog-prefixed namespaces.
- Keep the main bringup monitor-only: no automatic enable, zero, stand, walking, route execution, or motor output. Hardware tests require a separate explicit user instruction.
- Never change the original `D:/Project/dog` working deployments, the original NUC workspaces/services, or `dog_nav_v6_three30_20261024` and its frozen archives/evidence.
- Main ROS entry: `ros2_ws/src/dog_bringup/launch/dog_system.launch.py`. User commands: `./dog`.
- Keep machine-specific settings in `config/site.json`; generate relocated deployment paths under a new `runs/` directory. Do not inherit historical motor authorization from source records.
- `reference/` is provenance and third-party source, not an additional live control stack. Do not launch legacy controllers beside the consolidated controller.
- Do not store SSH passwords, private keys, or credentials in this tree.
