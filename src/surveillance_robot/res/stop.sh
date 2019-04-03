#!/bin/bash
pkill -f 'surveillance_robot_decision_node'
pkill -f 'surveillance_robot_translation_node'
pkill -f 'surveillance_robot_rotation_node'
pkill -f 'surveillance_robot_local_planner_node'
pkill -f 'map_server'
pkill -f 'amcl'