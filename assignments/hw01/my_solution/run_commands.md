# HW1 run commands

Open 3 terminals and run them in this order. For presets 2 and 3, change both `1`s to `2` or `3`.

## 0. Clean up leftovers (before every run)

Stop all terminals with Ctrl+C, then make sure no old simulation is still running
(otherwise you see two arms):

```bash
pkill -f "gz sim"; pkill -f check_reach; pkill -f marker_reacher; pgrep -af "gz sim"
```

The last command should print nothing.

## 1. Start the simulation (Terminal A)

```bash
cd ~/robotics-course-HW && source /opt/ros/jazzy/setup.bash && source assignments/hw01/environment/install/setup.bash && ros2 launch hw01_sim simulation.launch.py preset:=1
```

## 2. Check reach (Terminal B, start before the reacher)

```bash
cd ~/robotics-course-HW && source /opt/ros/jazzy/setup.bash && source assignments/hw01/environment/install/setup.bash && ros2 run hw01_sim check_reach --preset 1
```

## 3. Run the marker reacher (Terminal C)

```bash
cd ~/robotics-course-HW && source /opt/ros/jazzy/setup.bash && source assignments/hw01/environment/install/setup.bash && source assignments/hw01/my_solution/install/setup.bash && ros2 run hw01_solution marker_reacher
```
