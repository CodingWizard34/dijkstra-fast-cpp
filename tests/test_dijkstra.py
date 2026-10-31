import unittest
import json
import math
from dijkstra_fast_cpp.dijkstra_pure import dijkstra_python, dijkstra_cover_waypoints_python
from dijkstra_fast_cpp.map_generator import RoadNetwork, generate_random_map, segment_intersection
from dijkstra_fast_cpp.json_exporter import export_path_to_json

try:
    from dijkstra_fast_cpp._dijkstra_cython import dijkstra_cython, dijkstra_cover_waypoints_cython
    CYTHON_AVAILABLE = True
except ImportError:
    CYTHON_AVAILABLE = False


class TestDijkstraRouter(unittest.TestCase):

    def test_segment_intersection(self):
        # Perpendicular lines intersecting at (5, 5)
        p1, p2 = (0.0, 5.0), (10.0, 5.0)
        p3, p4 = (5.0, 0.0), (5.0, 10.0)
        pt = segment_intersection(p1, p2, p3, p4)
        self.assertIsNotNone(pt)
        self.assertAlmostEqual(pt[0], 5.0, places=3)
        self.assertAlmostEqual(pt[1], 5.0, places=3)

        # Parallel non-intersecting lines
        p5, p6 = (0.0, 0.0), (10.0, 0.0)
        p7, p8 = (0.0, 2.0), (10.0, 2.0)
        self.assertIsNone(segment_intersection(p5, p6, p7, p8))

    def test_simple_graph_dijkstra(self):
        # 0 --(2)--> 1 --(3)--> 2
        #  \-------(7)-------->/
        num_nodes = 3
        edges = [
            (0, 1, 2.0),
            (1, 2, 3.0),
            (0, 2, 7.0),
            (1, 0, 2.0),
            (2, 1, 3.0),
            (2, 0, 7.0)
        ]

        py_dist, py_path, py_reach = dijkstra_python(num_nodes, edges, 0, 2)
        self.assertTrue(py_reach)
        self.assertAlmostEqual(py_dist, 5.0)
        self.assertEqual(py_path, [0, 1, 2])

        if CYTHON_AVAILABLE:
            cy_dist, cy_path, cy_reach = dijkstra_cython(num_nodes, edges, 0, 2)
            self.assertTrue(cy_reach)
            self.assertAlmostEqual(cy_dist, 5.0)
            self.assertEqual(cy_path, [0, 1, 2])

    def test_map_generation_and_routing(self):
        net = generate_random_map(width=400, height=300, num_roads=8, num_waypoints=3, seed=123)
        self.assertGreater(len(net.nodes), 4)
        self.assertGreater(len(net.edges), 4)
        self.assertGreaterEqual(len(net.waypoints), 2)

        start = net.waypoints[0]
        end = net.waypoints[-1]

        py_dist, py_path, py_reach = dijkstra_python(len(net.nodes), net.edges, start, end)

        if CYTHON_AVAILABLE:
            cy_dist, cy_path, cy_reach = dijkstra_cython(len(net.nodes), net.edges, start, end)
            self.assertEqual(py_reach, cy_reach)
            if py_reach:
                self.assertAlmostEqual(py_dist, cy_dist, places=4)
                self.assertEqual(py_path[0], start)
                self.assertEqual(py_path[-1], end)

    def test_waypoint_coverage(self):
        net = generate_random_map(width=500, height=400, num_roads=10, num_waypoints=4, seed=999)
        if len(net.waypoints) >= 2:
            py_dist, py_path, py_reach = dijkstra_cover_waypoints_python(
                len(net.nodes), net.edges, net.waypoints
            )
            if CYTHON_AVAILABLE:
                cy_dist, cy_path, cy_reach = dijkstra_cover_waypoints_cython(
                    len(net.nodes), net.edges, net.waypoints
                )
                self.assertEqual(py_reach, cy_reach)
                if py_reach:
                    self.assertAlmostEqual(py_dist, cy_dist, places=4)
                    self.assertEqual(py_path, cy_path)

    def test_json_export(self):
        nodes = [{'id': 0, 'x': 0.0, 'y': 0.0}, {'id': 1, 'x': 3.0, 'y': 4.0}]
        edges = [(0, 1, 5.0), (1, 0, 5.0)]
        path = [0, 1]
        dist = 5.0
        json_str = export_path_to_json(nodes, edges, path, dist, waypoints=[0, 1])

        parsed = json.loads(json_str)
        self.assertEqual(parsed['summary']['total_distance'], 5.0)
        self.assertEqual(len(parsed['path_nodes']), 2)
        self.assertEqual(len(parsed['path_edges']), 1)


if __name__ == "__main__":
    unittest.main()
