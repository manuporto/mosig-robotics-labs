# mosig-robotics-labs
Laboratories of the course of Robotics and IoT from MoSIG

#### TODO
+ process update from local_planner in the decision node and send it to the translation/rotation node
+ understand structure of the grid map representation
+ assign a position (start, end, current) to a cell in the grid
+ plan a path from cell A to cell B in the grid
+ use robot's current and end position in planning
+ compare the robot's actual position with the desired one, if error > threshold, update planning


#### Topics:
decision node:
+ subscribes:
++ move_base_simple/goal
++ global_planer/planned_path
++ local_planner/translation_rotation
++ rotation_node/rotation_done
++ translation_node/translation_done
++ check_node/recalculate_goal

+ publishes
++ global_planer/global_goal
++ local_planner/local_goal
++ rotation_node/rotation_to_do
++ translation_node/translation_to_do
++ check_node/point

check node:
+ subscribes:
++ check_node/point
++ amcl_pose

+ publishes
++ check_node/recalculate_goal

local_planner node:
+ subscribes:
++ local_planner/local_goal

+ publishes
++ local_planner/translation_rotation
++ amcl_pose

global_planer goal:
+ subscribes:
++ global_planner/global_goal

+ publishes
++ global_planer/planned_path

rotation_node:
+ subscribes:
++ rotation_node/rotation_to_do

+ publishes
++ rotation_node/rotation_done

translation_node:
+ subscribes:
++ translation_node/translation_to_do

+ publishes
++ translation_node/translation_done

obstacle_detection_node:
+ subscribes:
++ scan

+ publishes
++ obstacle_detection_node/closest_obstacle
++ obstacle_detection_node/closest_obstacle_marker
