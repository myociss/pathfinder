from unittest import TestCase
import json, random, numpy as np, multiprocessing, math, itertools
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
                shape=plane2d.shapes()[shape_idx]
                vertices_2d=shape.arranged_vertices()

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

    def test_hulls(self):
        for test_iter in range(5):
            target=[2*random.random(), 2*random.random(), 2*random.random()]
            self.mesh.set_target(target)
            alpha=math.pi*random.random()
            theta=math.pi*random.random()

            plane_intersection=self.mesh.slice(rotation=[alpha,theta])
            plane3d = pathfinder.Plane3d(id=0, alpha=alpha, theta=theta, target=np.array(target))
            plane2d = pathfinder.Plane2d(plane_intersection, plane3d)

            for shape_idx in range(1, len(plane2d.shapes())):
                shape=plane2d.shapes()[shape_idx]
                vertices_2d=shape.arranged_vertices()

                convex_hull_vertex=vertices_2d[shape.hull_supporting_idx()]
                A=convex_hull_vertex[1]-vertices_2d[0][1]
                B=vertices_2d[0][0]-convex_hull_vertex[0]
                C=convex_hull_vertex[0]*vertices_2d[0][1]-vertices_2d[0][0]*convex_hull_vertex[1]

     
                origin_side=C
                #print('---------')
                #print(shape.hull_supporting_idx())
                #for v in vertices_2d:
                #    print(math.atan2(v[1], v[0]))

                for i in range(1, shape.hull_supporting_idx()):
                    vertex=vertices_2d[i]
                    vertex_side=vertex[0]*A+vertex[1]*B+C
                    self.assertGreater(origin_side*vertex_side, 0.0)

                for i in range(shape.hull_supporting_idx()+1, len(vertices_2d)):
                    vertex=vertices_2d[i]
                    vertex_side=vertex[0]*A+vertex[1]*B+C
                    self.assertLess(origin_side*vertex_side, 0.0)
    
    def test_all_interval_calculations(self):
        target=[2*random.random(), 2*random.random(), 2*random.random()]
        self.mesh.set_target(target)
        alpha=math.pi*random.random()
        theta=math.pi*random.random()
        print(alpha)
        print(theta)

        plane_intersection=self.mesh.slice(rotation=[alpha,theta])
        plane3d = pathfinder.Plane3d(id=0, alpha=alpha, theta=theta, target=np.array(target))
        plane2d = pathfinder.Plane2d(plane_intersection, plane3d)
        plane2d.calc_intervals_init()
        interval_bounds = plane2d.interval_bounds()

        all_vertices=list(itertools.chain.from_iterable([[tuple(v) for v in shape.vertices()] for shape in plane2d.shapes()]))
        intervals=sorted(set([math.atan2(v[1], v[0]) for v in all_vertices]))
        self.assertEqual(len(intervals), len(interval_bounds))

        #interval_shapes=[0 for i in range(len(intervals))]
                
        for idx, angle in enumerate(intervals):
            if idx==len(intervals)-1:
                angle_next=intervals[0] + 2 * math.pi
            else:
                angle_next=intervals[idx+1]

            interval_shapes=[0]
            #print(angle)
            #print(angle_next)

            for i in range(1, len(plane2d.shapes())):
                shape=plane2d.shapes()[i]
                vertices=shape.arranged_vertices()
                #print(i)
                #print(shape.hull_supporting_idx())
                for vertex_idx in range(shape.hull_supporting_idx()):
                    vertex=vertices[vertex_idx]
                    next=vertex_idx+1
                    vertex_next=vertices[next]
                    vertex_angle=math.atan2(vertex[1], vertex[0])
                    vertex_angle_next=math.atan2(vertex_next[1], vertex_next[0])
                    if vertex_angle>vertex_angle_next:
                        if idx==len(intervals)-1:
                            vertex_angle_next += 2 * math.pi
                        else:
                            vertex_angle -= 2 * math.pi
                    if angle>=vertex_angle and angle_next<=vertex_angle_next:
                        interval_shapes.append(i)
                        break
            print(len(interval_shapes))
    

if __name__ == '__main__':
    unittest.main()
