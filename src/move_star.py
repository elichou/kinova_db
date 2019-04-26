#!/usr/bin/env python

from move_generic import MoveGroupPythonInterface
from random import *
import numpy as np
import rospy


def main ():
    try:
        print("")
        print("Movements data base creation")
        print("First movement:")
        print("5 random postions : 10 star-like movements")
        print("")

        print("===== Press Enter to begin")
        raw_input()
        move = MoveGroupPythonInterface()

        print("==== Press Enter to execute the STAR PLAN")
        raw_input()
        waypoints = move.star_waypoints()
        cartesian_plan, fraction = move.plan_cartesian_path(waypoints)
        move.display_trajectory(cartesian_plan)
        move.execute_plan(cartesian_plan)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
