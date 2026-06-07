import unittest

from vces_dstar_lite import EdgeGrid, MazeNavigator, generate_random_obstacles


class TestVCESDStarLite(unittest.TestCase):
    def test_topology_has_144_internal_edges(self):
        grid = EdgeGrid()
        self.assertEqual(len(grid.edges), 144)
        self.assertEqual(len(grid.neighbors[1]), 2)
        self.assertEqual(len(grid.neighbors[41]), 4)

    def test_random_obstacle_generation(self):
        grid = EdgeGrid()
        blocked = generate_random_obstacles(grid, start=1, goals=[14, 44, 68, 38], seed=11)
        self.assertEqual(len(blocked), 20)
        self.assertTrue(all(e not in blocked for e in grid.incident_edges(1)))
        self.assertTrue(all(e not in blocked for e in grid.incident_edges(9)))

    def test_navigation_simulation_completes(self):
        nav = MazeNavigator(start=1, goals=[14, 44, 68, 38])
        true_blocked = generate_random_obstacles(nav.grid, start=1, goals=[14, 44, 68, 38], seed=11)
        for _ in range(220):
            result = nav.step_simulation(true_blocked)
            if result.action == "DONE":
                break
        self.assertTrue(nav.done)


if __name__ == "__main__":
    unittest.main()
