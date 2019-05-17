from unittest import TestCase
import json, random, numpy as np, multiprocessing, math
import pathfinder

class TestGraph(TestCase):

    def setUp(self):
        with open('fixtures/test_mesh.json', 'r') as mesh_file:
            self.test_mesh=json.load(mesh_file)
        self.mesh=pathfinder.Mesh(num_vertices=len(self.test_mesh['vertices']), num_faces=len(self.test_mesh['faces']), num_tetrahedrons=len(self.test_mesh['tetrahedrons']))

        self.mesh.set_vertices(self.test_mesh['vertices'])

        for idx, tet in enumerate(self.test_mesh['tetrahedrons']):
            self.mesh.add_tetrahedron(tetrahedron_id=idx, neighbor_ids=tet['neighbors'], vertex_ids=tet['vertices'], weight=tet['weight'], label=0)

        for face in self.test_mesh['faces']:
            self.mesh.add_face(vertex_ids=face['vertices'], tetrahedron_id=face['tetrahedron'])
  

    def test_target_set(self):
        # should probably also include a test for a point that definitely lies in more than one sphere
        for idx, tet in enumerate(self.test_mesh['tetrahedrons']):
            vertices=[self.test_mesh['vertices'][i] for i in tet['vertices']]
            pt=[sum([vertices[i][0] for i in range(4)])/4, sum([vertices[i][1] for i in range(4)])/4, sum([vertices[i][2] for i in range(4)])/4]

            self.mesh.set_target(pt)
            self.assertEqual(idx, self.mesh.get_target_idx())


    def test_slice(self):
        for test_iter in range(20):
            target=[2*random.random(), 2*random.random(), 2*random.random()]
            self.mesh.set_target(target)
            alpha=math.pi*random.random()
            theta=math.pi*random.random()

            plane_intersection=self.mesh.slice(rotation=[alpha,theta])
            tet_ids=[shape.tet_id() for shape in plane_intersection]

            rotation_x=np.array([[1,0,0], [0,math.cos(alpha),math.sin(alpha)], [0,-math.sin(alpha),math.cos(alpha)]])
            rotation_y=np.array([[math.cos(theta),0,math.sin(theta)], [0,1,0], [-math.sin(theta),0,math.cos(theta)]])

            normal=(np.matmul(rotation_x, rotation_y))[2]
            contains_count=0

            for idx, tet in enumerate(self.test_mesh['tetrahedrons']):
                intersection_count=0
                vertices=[self.test_mesh['vertices'][v_id] for v_id in tet['vertices']]
                for i in range(4):
                    v0=np.array(vertices[i])
                    dot_product_i=np.dot(normal, v0-target)
                    if dot_product_i==0:
                        intersection_count+=1
                    else:
                        for j in range(i+1, 4):
                            v1=np.array(vertices[j])
                            dot_product_j=np.dot(normal, v1-target)
                            if dot_product_i * dot_product_j < 0:
                                intersection_count+=1

                if intersection_count>2:
                    self.assertIn(idx, tet_ids)
                    contains_count+=1

            self.assertEqual(contains_count, len(plane_intersection))

    def test_flatten_plane(self):
        for test_iter in range(5):
            target=[2*random.random(), 2*random.random(), 2*random.random()]
            self.mesh.set_target(target)
            alpha=math.pi*random.random()
            theta=math.pi*random.random()

            plane_intersection=self.mesh.slice(rotation=[alpha,theta])
            plane3d = pathfinder.Plane3d(id=0, alpha=alpha, theta=theta, target=np.array(target))
            plane2d = pathfinder.Plane2d(plane_intersection, plane3d)

            for shape_idx, shape in enumerate(plane2d.shapes()):
                vertices_2d=shape.vertices()
                vertices_3d=plane_intersection[shape_idx].vertices()
                
                for vertex_idx, vertex_2d in enumerate(vertices_2d):
                    vertex_3d=vertices_3d[vertex_idx]
                    if vertex_idx==len(vertices_2d)-1:
                        next_2d=vertices_2d[0]
                        next_3d=vertices_3d[0]
                    else:
                        next_2d=vertices_2d[vertex_idx+1]
                        next_3d=vertices_3d[vertex_idx+1]
                   
                    distance_2d = math.sqrt( ((vertex_2d[0]-next_2d[0])**2)+((vertex_2d[1]-next_2d[1])**2) )
                    distance_3d = math.sqrt( ((vertex_3d[0]-next_3d[0])**2)+((vertex_3d[1]-next_3d[1])**2) + ((vertex_3d[2]-next_3d[2])**2))
                    self.assertLess(abs(distance_2d - distance_3d), 10e-8)
                    
    def test_arrange_vertices(self):
        for test_iter in range(5):
            target=[2*random.random(), 2*random.random(), 2*random.random()]
            self.mesh.set_target(target)
            alpha=math.pi*random.random()
            theta=math.pi*random.random()

            plane_intersection=self.mesh.slice(rotation=[alpha,theta])
            plane3d = pathfinder.Plane3d(id=0, alpha=alpha, theta=theta, target=np.array(target))
            plane2d = pathfinder.Plane2d(plane_intersection, plane3d)

            for shape_idx in range(1, len(plane2d.shapes())):
                vertices_2d=plane2d.shapes()[shape_idx].arranged_vertices()

                angle_inter=math.acos(np.dot(vertices_2d[0], vertices_2d[1])/(np.linalg.norm(vertices_2d[0]) * np.linalg.norm(vertices_2d[1])))
                angle_first=math.atan2(vertices_2d[0][1], vertices_2d[0][0])
                angle_next=math.atan2(vertices_2d[1][1], vertices_2d[1][0])
                angle_expected=angle_first+angle_inter

                if angle_expected>math.pi:
                    angle_expected=-math.pi + (angle_expected % math.pi)

                self.assertLess(abs(angle_next-angle_expected), 10e-8)

                angle_inter=math.acos(np.dot(vertices_2d[0], vertices_2d[-1])/(np.linalg.norm(vertices_2d[0]) * np.linalg.norm(vertices_2d[-1])))
                angle_last=math.atan2(vertices_2d[-1][1], vertices_2d[-1][0])
                angle_expected=angle_first+angle_inter

                if angle_expected>math.pi:
                    angle_expected=-math.pi + (angle_expected % math.pi)

                self.assertLess(abs(angle_last-angle_expected), 10e-8)

    

if __name__ == '__main__':
    unittest.main()
