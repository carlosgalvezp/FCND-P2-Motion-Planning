import os
import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

REPO_ROOT = os.path.dirname(__file__)
COLLIDERS_CSV = os.path.join(REPO_ROOT, 'colliders.csv')

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # Read lat0, lon0 from colliders into floating point values
        lat0 = 0.0
        lon0 = 0.0
        with open(COLLIDERS_CSV, 'r') as csv_file:
            first_line = csv_file.readline().split(',')
            lat0 = float(first_line[0].strip().split(' ')[1])
            lon0 = float(first_line[1].strip().split(' ')[1])

        # Set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # Retrieve current global position
        global_position = self.global_position

        # Convert to current local position using global_to_local()
        local_position = global_to_local(global_position, self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Convert start position to current position rather than map center
        grid_start = (int(local_position[0]) - north_offset,
                      int(local_position[1]) - east_offset)

        # Set goal as some arbitrary position on in global (GPS) coordinates
        goal_lon = -122.401242
        goal_lat = 37.796730
        global_goal = (goal_lon, goal_lat, 0)

        # Convert to local (NED) coordinates
        local_goal = global_to_local(global_goal, self.global_home)

        # Convert to grid coordinates
        grid_goal = (int(local_goal[0]) - north_offset,
                     int(local_goal[1]) - east_offset)

        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', grid_start, grid_goal)
        print('Planning, please wait...')
        t1 = time.time()
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print('Planning took {} seconds'.format(time.time() - t1))
        print('Number of waypoints: {}'.format(len(path)))

        # Prune path to minimize number of waypoints
        print('Pruning path, please wait...')
        t1 = time.time()
        path = prune_path(path, grid)
        print('Prunning took {} seconds'.format(time.time() - t1))
        print('Number of waypoints: {}'.format(len(path)))

        # Convert path to waypoints
        waypoints = []
        for i in range(len(path)):
            px = path[i][0] + north_offset
            py = path[i][1] + east_offset
            pz = TARGET_ALTITUDE

            px_prev = path[max(i-1, 0)][0] + north_offset
            py_prev = path[max(i-1, 0)][1] + east_offset

            yaw = np.arctan2(py - py_prev, px - px_prev)
            waypoints.append([px, py, pz, yaw])

        # Set self.waypoints
        self.waypoints = waypoints

        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
