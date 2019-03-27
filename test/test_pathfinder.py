from unittest import TestCase
import json
import pathfinder
import numpy as np
import math

class TestGraph(TestCase):

    def setUp(self):
        with open('fixtures/test_mesh.json', 'r') as mesh_file:
            self.test_mesh=json.load(mesh_file)
        self.mesh=pathfinder.Mesh(num_vertices=len(self.test_mesh['vertices']), num_faces=len(self.test_mesh['faces']), num_tetrahedrons=len(self.test_mesh['tetrahedrons']), num_threads=8)

        self.mesh.set_vertices(self.test_mesh['vertices'])

        for idx, tet in enumerate(self.test_mesh['tetrahedrons']):
            self.mesh.add_tetrahedron(tetrahedron_id=idx, neighbor_ids=tet['neighbors'], vertex_ids=tet['vertices'], weight=tet['weight'])
  
    
    def test_target_set(self):
        # should probably also include a test for a point that definitely lies in more than one sphere
        for idx, tet in enumerate(self.test_mesh['tetrahedrons']):
            vertices=[self.test_mesh['vertices'][i] for i in tet['vertices']]
            pt=[sum([vertices[i][0] for i in range(4)])/4, sum([vertices[i][1] for i in range(4)])/4, sum([vertices[i][2] for i in range(4)])/4]

            self.mesh.set_target(pt)
            self.assertEqual(idx, self.mesh.get_target_idx())

    def test_slice(self):
        target=[0.2,0.2,0.2]
        self.mesh.set_target(target)

        alpha=0.25
        theta=0.25
        plane_intersection=self.mesh.slice(alpha=alpha, theta=theta)
        tet_ids=self.mesh.get_slice_ids()

        rotation_x=np.array([[1,0,0], [0,math.cos(alpha),math.sin(alpha)], [0,-math.sin(alpha),math.cos(alpha)]])
        rotation_y=np.array([[math.cos(theta),0,math.sin(theta)], [0,1,0], [-math.sin(theta),0,math.cos(theta)]])

        normal=(np.matmul(rotation_x, rotation_y))[2]
        contains_count=0

        for idx, tet in enumerate(self.test_mesh['tetrahedrons']):
            intersection_count=0
            vertices=[self.test_mesh['vertices'][v_id] for v_id in tet['vertices']]
            for i in range(4):
                for j in range(i+1, 4):
                    v0=np.array(vertices[i])
                    v1=np.array(vertices[j])

                    dot_product_i=np.dot(normal, v0-target)
                    dot_product_j=np.dot(normal, v1-target)
                    if dot_product_i==0 and dot_product_j==0:
                        intersection_count+=2
                    elif dot_product_i * dot_product_j < 0:
                        intersection_count+=1

            if intersection_count>2:
                self.assertIn(idx, tet_ids)
                contains_count+=1

        self.assertEqual(contains_count, len(plane_intersection))
    

if __name__ == '__main__':
    unittest.main()
