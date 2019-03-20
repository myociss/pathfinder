from unittest import TestCase
import json
import pathfinder

class TestGraph(TestCase):

    def setUp(self):
        with open('fixtures/test_mesh.json', 'r') as mesh_file:
            self.mesh = json.load(mesh_file)
    '''
    def test_graph(self):
        print(self.mesh['vertices'])
        graph = pathfinder.Graph(self.mesh['vertices'])
        vertices_from_json = self.mesh['vertices']
        for idx, v in enumerate(graph.vertices()):
            print('here')
            #vec_from_json = vertices_from_json[idx]
            #vec = v.Vec()
            #self.assertEqual(vec_from_json, vec)
    '''

if __name__ == '__main__':
    unittest.main()
