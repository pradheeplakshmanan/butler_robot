# Butler Robot

A cafe butler robot built using ROS2 and Nav2. The robot delivers food from the kitchen to customer tables automatically.

## What it does

The robot starts at its home position. When an order comes in, it goes to the kitchen to pick up the food, delivers it to the right table, and returns home.

## Cafe Layout

- Home — where the robot starts (green marker on floor)
- Kitchen — where food is collected
- Table 1, Table 2, Table 3 — customer tables

## Robot

TurtleBot3 Burger running in Gazebo simulation with Nav2 navigation.

## Progress

- [x] Cafe world created in Gazebo
- [ ] Milestone 1 — Basic delivery (Home → Kitchen → Table → Home)
- [ ] Milestone 2 — Timeout handling
- [ ] Milestone 3 — Kitchen and table timeout scenarios
- [ ] Milestone 4 — Task cancellation
- [ ] Milestone 5 — Multiple orders
- [ ] Milestone 6 — Skip unconfirmed table
- [ ] Milestone 7 — Skip cancelled table