from unittest import TestCase
import json
import pathfinder

class TestGraph(TestCase):

    def setUp(self):
        with open('fixtures/test_mesh.json', 'r') as mesh_file:
            self.test_mesh = json.load(mesh_file)
    
    def test_target_set(self):
        # should probably also include a test for a point that definitely lies in more than one sphere

        mesh = pathfinder.Mesh(num_vertices=len(self.test_mesh['vertices']), num_faces=len(self.test_mesh['faces']), num_tetrahedrons=len(self.test_mesh['tetrahedrons']), num_threads=8)

        mesh.set_vertices(self.test_mesh['vertices'])


        for idx, tet in enumerate(self.test_mesh['tetrahedrons']):
            mesh.add_tetrahedron(tetrahedron_id=idx, neighbor_ids=tet['neighbors'], vertex_ids=tet['vertices'], weight=tet['weight'])
  
        for idx, tet in enumerate(self.test_mesh['tetrahedrons']):
            vertices = [self.test_mesh['vertices'][i] for i in tet['vertices']]
            pt = [ sum([vertices[i][0] for i in range(4)])/4, sum([vertices[i][1] for i in range(4)])/4, sum([vertices[i][2] for i in range(4)])/4]

            mesh.set_target(pt)
            self.assertEqual(idx, mesh.get_target_idx())
    

if __name__ == '__main__':
    unittest.main()
