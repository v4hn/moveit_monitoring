# moveit_monitoring

This package provides basic MoveIt state information to the ROS diagnostics subsystem,
as monitored by the diagnostics_aggregator, various robot dashboards and rqt_robot_monitor.

![RQT monitor showing 5 contacts reported as error in the diagnostics subsystem](doc/rqt_monitor.png)

# install for use

The capability provided by this package has to be loaded as a MoveGroupCapability in your robot's moveit_config.

To do so, either pass the capability as an `arg` to your `move_group.launch`, e.g.,
```
roslaunch myrobot_moveit_config move_group.launch capabilities:=moveit_monitoring/Monitoring
```

or add it to the list of capabilities to load directly in the `move_group.launch`:
```
  <arg name="capabilities" value="
                moveit_monitoring/Monitoring
                " />
```

a demo configuration and launch file are provided with this package.
