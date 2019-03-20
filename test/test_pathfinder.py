from unittest import TestCase
import json
import pathfinder

class PathfinderTest(TestCase):

    def setUp(self):
        self.mesh = json.load('fixtures/test_mesh.json')

if __name__ == '__main__':
    unittest.main()
