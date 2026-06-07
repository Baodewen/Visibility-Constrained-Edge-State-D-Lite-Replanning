#!/usr/bin/env python3
"""
Minimal ROS1 wrapper skeleton.

This file is intentionally a template. It imports rospy only when executed
inside a ROS1 environment.
"""

from vces_dstar_lite import EdgeState, MazeNavigator


class MazePlannerRosNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import Int32, String

        self.rospy = rospy
        start = rospy.get_param("~start", 1)
        goals = rospy.get_param("~goals", [14, 44, 68, 38])
        self.nav = MazeNavigator(start=start, goals=goals)

        self.next_cell_pub = rospy.Publisher("~next_cell", Int32, queue_size=10)
        self.status_pub = rospy.Publisher("~status", String, queue_size=10)
        rospy.Subscriber("~current_cell", Int32, self.on_current_cell)

    def on_current_cell(self, msg):
        if msg.data != self.nav.current:
            try:
                self.nav.confirm_move(msg.data)
            except Exception as exc:
                self.status_pub.publish(f"confirm_move failed: {exc}")

        action, next_cell, info = self.nav.decide_next_action()
        self.status_pub.publish(f"{action}: {info}")
        if action == "MOVE" and next_cell is not None:
            self.next_cell_pub.publish(next_cell)

    def on_edge_observation(self, msg):
        # TODO: adapt your own message, e.g.
        # observations = [(msg.edge_id, EdgeState.FREE if msg.state == 1 else EdgeState.BLOCKED)]
        # self.nav.apply_observations(observations)
        pass

    def spin(self):
        self.rospy.spin()


def main():
    import rospy
    rospy.init_node("vces_dstar_lite_planner")
    MazePlannerRosNode().spin()


if __name__ == "__main__":
    main()
