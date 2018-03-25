## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates
   (format for `self.all_waypoints` is [N, E, altitude, heading],
   where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how Iaddressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

- The `planning_utils.py` script contains utility functions to support the planning implementation.
  In particular, it provides a function `create_grid` to create a numpy 2D grid where cells
  with value 1 are considered as "obstacles". In addition, an implementation of a grid-based
  A* algorithm, `a_star`, is also provided, together with helper functions needed for it,
  like the heuristic function and an `Action` class defining the possible actions to
  take while moving within the grid. This code is the same as the one studied in the lectures.

- The `motion_planning.py` script is the main entrypoint for this project. The provided code
  sets up the MAVLink connection with the drone simulator, and implements the different
  callbacks for state, position and velocity changes, in the same way as done
  in the `backyard_flyer` project. The position callback is used to decide whether
  the drone should start or continue following waypoints, or finish the simulation
  by landing. The velocity callback is used for disarming the drone when it stopped
  moving after landing. Finally, the state callback implements the different transitions
  from arming to waypoint following. All the state transition functions
  are also provided.

  The only thing left to implement for the students is the `plan_path`
  function, which is executed before taking off, with the aim of setting the `self.waypoints`
  array for the drone to follow the desired trajectory. First, the current position
  is extracted in local coordinates. Then, a start and goal position are manually
  set. This information is passed to the A* planner which outputs a list of waypoints
  from start to goal, if the path was found. Next, the path is cleaned by removing
  redundant intermediate waypoints by some algorithm, like a colinearity check
  or the Bresenham algorithm. Finally, the waypoints are sent to the simulator
  to execute the trajectory and for visualization.

  In the given starter code, the drone is given a start position (it's current position),
  and a global position, 10 meters forward in X and Y directions. This information
  is then passed to the A* planner to find a trajectory in the grid. Since initially
  the A* planner only has 4 actions, and the drone has to go diagonally, the result
  is a zig-zag trajectory. After adding the diagonal motion in the following
  exercises, the planned trajectory will be straight instead.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
The global home position is read from the first line of the CSV file:

```
lat0 37.792480, lon0 -122.397450
```

The following code is used to parse that line and extract `lan0` and `lon`:

```
lat0 = 0.0
lon0 = 0.0
with open(COLLIDERS_CSV, 'r') as csv_file:
    first_line = csv_file.readline().split(',')
    lat0 = float(first_line[0].strip().split(' ')[1])
    lon0 = float(first_line[1].strip().split(' ')[1])
```

Basically the first line is read, then we split into 2 tokens separated by comma,
and then we split each token again separating by space. Finally, we take the
second token from this separation and cast to a float value.

Finally, the home position is set using the Udacidrone API:

```
self.set_home_position(lon0, lat0, 0)
```

We set the altitude to 0 since the drone starts on the ground.

#### 2. Set your current local position
The next step is to determine our current position in the local frame (NED),
relative to the global position we just set.

First, we obtain the current global position, as a public attribute of
the `Drone` class:

```
# Retrieve current global position
global_position = self.global_position
```

Finally, we obtain the `local_position` by using the `global_to_local`
function provided in the `udacidrone` package:

```
# Convert to current local position using global_to_local()
local_position = global_to_local(global_position, self.global_home)
```

#### 3. Set grid start position from local position
The start position is set to the local position using the following code:

```
# Convert start position to current position rather than map center
grid_start = (int(local_position[0]) - north_offset,
              int(local_position[1]) - east_offset)
```

Simply, the `local_position` (NED) is cast to int and transformed into the
local grid coordinate system, by removing the `north_offset` and `east_offset`
variables which are the minimum north and east coordinates of the grid.
In other words, a local position of `(nort_offset, east_offset)` would be
equivalent to position (0,0) in the grid.


#### 4. Set grid goal position from geodetic coords
First,  we set a variable in the code with the global GPS position of the goal:

```
# Set goal as some arbitrary position on in global (GPS) coordinates
goal_lon = -122.401242
goal_lat = 37.796730
global_goal = (goal_lon, goal_lat, 0)
```

Then we convert it to local (NED) coordinates:

```
# Convert to local (NED) coordinates
local_goal = global_to_local(global_goal, self.global_home)
```

Finally, we convert to grid coordinates following the same procedure as with
the `local_position`, in previous rubric point #3:

```
# Convert to grid coordinates
grid_goal = (int(local_goal[0]) - north_offset,
             int(local_goal[1]) - east_offset)
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
The A* algorithm is updated as follows to include diagonal motion.

First, four new actions are added to the `Action` enum, in `planning_utils.py`:

```
NORTH_WEST = (-1, -1, np.sqrt(2))
NORTH_EAST = (-1, 1, np.sqrt(2))
SOUTH_WEST = (1, -1, np.sqrt(2))
SOUTH_EAST = (1, 1, np.sqrt(2))
```

Notice there is always non-zero motion in both the north and east directions.
The cost of these actions is `sqrt(2)` instead of 1.

Finally, we also need to update the `valid_actions` function in order
to check if these actions are valid or not. To make the function more
generic, we do a little refactoring where we look over all available
actions and decide which ones provide valid actions, as seen here:

```

def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    all_actions = list(Action)
    valid_actions = []
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle
    for action in all_actions:
        dx, dy = action.delta
        new_x = x + dx
        new_y = y + dy

        if new_x >= 0 and new_x <= n and new_y >= 0 and new_y <= m and grid[new_x, new_y] == 0:
            valid_actions.append(action)

    return valid_actions
```

These modifications are enough to extend A* with diagonal motion.

#### 6. Cull waypoints
The last step is to prune the path to remove unnecessary points to have a smoother
flight. This is implemented in the `prune_path` function, in `planning_utils.py`:

```
def prune_path(path, grid):
    pruned_path = [p for p in path]

    finished_prunning = False
    while not finished_prunning:
        prunned = False
        for i in range(len(pruned_path) - 2):
            p1 = pruned_path[i]
            p2 = pruned_path[i+1]
            p3 = pruned_path[i+2]

            # If can go directly from p1 to p3, then p2 is not required
            if _is_collision_free(p1, p3, grid):
                pruned_path.remove(p2)
                prunned = True
                break
        finished_prunning = not prunned

    return pruned_path
```

Basically, we loop over the original path and take 3 points at a time,
`p1`, `p2` and `p3`. If it's possible to go on a straight line between
`p1` and `p3` without crossing any obstacle, it means `p2` is unnecessary,
and thus we remove it from the original path.

To check if there's a collision-free trajectory between two points, we implement
the function `_is_collision_free`:

```
def _is_collision_free(p1, p2, grid):
    # Compute cells covered by the line p1-p2 using the Bresenham algorithm
    covered_cells = list(bresenham(p1[0], p1[1], p2[0], p2[1]))

    # Check if any of the cells is an obstacle cell
    for cell in covered_cells:
        if grid[cell[0], cell[1]]:
            return False
    return True
```

Here we use the Bresenham algorithm (`bresenham` Python package) to extract
a list of cells that are covered by the line between `p1` and `p2`.
Finally, we loop over that list of cells to check if any of them lies on
an obstacle in the grid. If it does, we return immediately and decide
that the trajectory between `p1` and `p2` is NOT collision-free.

The implemented prunning algorithm significantly reduces the number of waypoints:
from 621 to 14, making the flight much smoother and faster.

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.
