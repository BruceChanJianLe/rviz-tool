# RViz Tool

This repository demonstrates the way to create a new tool to be used in RViz. The example is a tool that can send 2D goal to robot (move_base_flex).

## Rviz Icon

Create `icons/classes` and place the icon which should be the exact same name as the tool class. In this example, it is `mbfGoal`.

## Rviz TF

Include `rviz/frame_manager.h` in your header file. And in the `onInitialize()` function, obtain the `tf2_ros::Buffer` as so.

```cpp
    tf_ = context_->getFrameManager()->getTF2BufferPtr();
```

## Reference
- New tool ROS wiki [link_wiki](http://docs.ros.org/en/kinetic/api/rviz_plugin_tutorials/html/tool_plugin_tutorial.html)
- Rviz tool that uses tf [github_link](https://github.com/nobleo/rviz_satellite/blob/master/src/aerialmap_display.h)
