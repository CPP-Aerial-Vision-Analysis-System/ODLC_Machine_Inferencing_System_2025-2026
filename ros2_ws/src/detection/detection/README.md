# How to run 

```bash
cd ros2_ws/
colcon build && source install/setup.bash && ros2 run detection new_od 
```

# How to stop

```bash
ros2 lifecycle set /new_od shutdown
```

