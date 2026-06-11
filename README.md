# Butler Robot

Autonomous cafe food delivery robot built with ROS 2 Jazzy, TurtleBot3 Burger, Gazebo and Nav2.

---

## What it does

Robot starts at home. When an order comes in, it goes to the kitchen to pick up the food, delivers to the table, and returns home. It handles timeouts, cancellations and multiple orders.

---

## How I built it

First I created a custom cafe world in Gazebo with a home position, kitchen and 3 tables.

Then I mapped the environment by driving the robot manually using teleop keyboard and SLAM Toolbox. Saved the map and loaded it in Nav2.

For navigation I started with DWB controller but the robot was getting stuck when the goal was directly behind it. Switched to Regulated Pure Pursuit Controller which handles rotation to heading cleanly before driving. Tuned the costmap inflation radius to 0.2m to match the cafe walls.

Waypoints are stored in a YAML file so adding a new table doesn't need any code change.

---

## ROS Topics

- `/butler/order` — send order, single table or multiple like `table1,table2,table3`
- `/butler/kitchen_confirm` — confirm food is ready at kitchen
- `/butler/table_confirm` — confirm food received at table
- `/butler/cancel` — cancel the active order
- `/butler/cancel_table` — cancel a specific table order (Milestone 7)

---

## How to Run

```bash
# Terminal 1 - Launch world and Nav2
ros2 launch butler_robot butler_world.launch.py

# Terminal 2 - Run the robot
ros2 run butler_robot butler_state_machine

# Send an order
ros2 topic pub --once /butler/order std_msgs/msg/String "data: 'table1'"
```

---

## Milestones

### Milestone 1 — Basic Delivery

Robot goes from home to kitchen, then to the table, then back home. No confirmation needed.

States: `IDLE → GOING_TO_KITCHEN → GOING_TO_TABLE → GOING_HOME → IDLE`

### Milestone 2 — Timeout Handling

Robot waits at kitchen and table for confirmation. If nobody responds within 10 seconds it goes home.

States: `GOING_TO_KITCHEN → WAITING_KITCHEN → (timeout) → GOING_HOME`

### Milestone 3 — Kitchen and Table Timeout Scenarios

Two scenarios handled separately.

If nobody confirms at the kitchen, robot goes home directly.
`WAITING_KITCHEN → (timeout) → GOING_HOME`

If kitchen confirms but nobody confirms at the table, robot goes back to kitchen first and then home.
`WAITING_TABLE → (timeout) → RETURNING_TO_KITCHEN → GOING_HOME`

### Milestone 4 — Cancellation

Cancel signal can be sent any time while the robot is busy.

If cancelled while going to kitchen, robot goes home directly.
`GOING_TO_KITCHEN → (cancel) → GOING_HOME`

If cancelled while going to table, robot returns to kitchen first then goes home.
`GOING_TO_TABLE → (cancel) → RETURNING_TO_KITCHEN → GOING_HOME`

```bash
ros2 run butler_robot cancel_order
```

### Milestone 5 — Multiple Orders

Multiple tables sent in one message. Robot visits all tables one by one and returns home after all deliveries.

`GOING_TO_KITCHEN → WAITING_KITCHEN → GOING_TO_TABLE (table1) → WAITING_TABLE → GOING_TO_TABLE (table2) → WAITING_TABLE → GOING_HOME`

```bash
ros2 topic pub --once /butler/order std_msgs/msg/String "data: 'table1,table2,table3'"
```

### Milestone 6 — Skip Unconfirmed Table

If nobody confirms at a table within the timeout, robot skips that table and moves to the next one. After finishing all tables, robot goes to kitchen before going home.

`WAITING_TABLE → (timeout) → skip → next table → ... → RETURNING_TO_KITCHEN → GOING_HOME`

### Milestone 7 — Skip Cancelled Table

A specific table can be cancelled by name before the robot reaches it. Robot skips only that table and delivers to the remaining ones. After finishing, goes to kitchen then home.

`GOING_TO_TABLE (table2 cancelled) → skip → GOING_TO_TABLE (table3) → RETURNING_TO_KITCHEN → GOING_HOME`

```bash
ros2 topic pub --once /butler/cancel_table std_msgs/msg/String "data: 'table2'"
```
