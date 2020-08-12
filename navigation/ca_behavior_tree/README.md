# ca_behavior_tree

https://www.behaviortree.dev/
https://github.com/zmunro/BehaviorTree/blob/master/problem_specification.pdf
https://github.com/RoboticaUtnFrba/create_autonomy/issues/164
<!-- NavigationModeSelector -->
https://github.com/AndreasZachariae/PeTRA/blob/3aeb742b5b95c441a681997961334f5465494294/petra_central_control/src/actions/SkillSelection.cpp

```bash
rosrun groot Groot
GUI=false RVIZ=true LOCALIZATION=amcl roslaunch ca_gazebo create_house.launch
roslaunch ca_behavior_tree test.launch
```

## TODO

* Interrupt BT while robot is docking (Simply: fast charging and return to autonomy)
* Creat simple BT with two nodes:
  * Go somewhere and do nothing (should be doing a subtree)
  * Go to dock and recharge battery
