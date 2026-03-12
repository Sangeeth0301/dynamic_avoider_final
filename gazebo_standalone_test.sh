#!/bin/bash

# Gazebo Standalone Verifier
# This script tests the physics and sensors of your SUV in Gazebo Harmonic.

echo "Starting Gazebo Standalone Verification..."

# 1. Launch Gazebo with the optimal world
# -s runs the server, -r runs the simulation immediately
gz sim -r src/dynamic_obstacle_pkg/worlds/dynamic_world.sdf --force-version 8 &
GZ_PID=$!

sleep 10

echo ""
echo "--- STEP 1: VERIFYING TOPICS ---"
gz topic -l | grep -E "scan|odom|cmd_vel"

echo ""
echo "--- STEP 2: VERIFYING PHYSICS (DRIVE & STEER) ---"
echo "1. Sending drive command: linear x=1.0"
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}"
sleep 1
echo "2. Sending steering command: angular z=0.5"
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: 0.5}"
sleep 2
echo "Stopping car..."
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"

echo ""
echo "--- HOW TO CHECK FOR PERFECTION IN THE GAZEBO UI ---"
echo "1. Observe the Front Wheels: They should PIVOT left/right during steering (the 'Proper' way)."
echo "2. Observe the SUV mesh: It should follow a curved path when steering is applied."
echo "3. Check the Actors: All 6 humans should be walking their pseudo-random paths."
echo "4. Enable Visual Sensors: Click 'View' -> 'Visual Indicators' -> 'Sensors' to see BLUE LiDAR rays."
echo ""
echo "Verification Complete. Close the Gazebo window to exit."

trap "kill $GZ_PID" SIGINT SIGTERM
wait
