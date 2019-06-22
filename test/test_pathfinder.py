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
        self.assertFalse(self.mesh.set_target([5.0, 5.0, 5.0]))
        # should probably also include a test for a point that definitely lies in more than one sphere
        for idx, tet in enumerate(self.test_mesh['tetrahedrons']):
            vertices=[self.test_mesh['vertices'][i] for i in tet['vertices']]
            pt=[sum([vertices[i][0] for i in range(4)])/4, sum([vertices[i][1] for i in range(4)])/4, sum([vertices[i][2] for i in range(4)])/4]

            self.mesh.set_target(pt)
            self.assertEqual(idx, self.mesh.get_target_idx())


    def test_slice(self):
        for test_iter in range(5):
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

                hull_idx=shape.hull_supporting_idx()
                next_idx=(hull_idx+1)%len(vertices_2d)

                angle_inter=math.acos(np.dot(vertices_2d[hull_idx], vertices_2d[next_idx])/(np.linalg.norm(vertices_2d[hull_idx]) * np.linalg.norm(vertices_2d[next_idx])))

                angle_hull=math.atan2(vertices_2d[hull_idx][1], vertices_2d[hull_idx][0])
                angle_next=math.atan2(vertices_2d[next_idx][1], vertices_2d[next_idx][0])
                angle_expected=angle_hull-angle_inter

                if angle_expected<-math.pi:
                    angle_expected=2*math.pi + angle_expected

                self.assertLess(abs(angle_next-angle_expected), 10e-8)


                prev_idx=hull_idx-1

                angle_inter=math.acos(np.dot(vertices_2d[hull_idx], vertices_2d[prev_idx])/(np.linalg.norm(vertices_2d[hull_idx]) * np.linalg.norm(vertices_2d[prev_idx])))

                angle_prev=math.atan2(vertices_2d[prev_idx][1], vertices_2d[prev_idx][0])
                angle_expected=angle_hull-angle_inter

                if angle_expected<-math.pi:
                    angle_expected=2*math.pi + angle_expected

                self.assertLess(abs(angle_prev-angle_expected), 10e-8)

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

                for i in range(1, shape.hull_supporting_idx()):
                    vertex=vertices_2d[i]
                    vertex_side=vertex[0]*A+vertex[1]*B+C
                    self.assertGreater(origin_side*vertex_side, 0.0)

                for i in range(shape.hull_supporting_idx()+1, len(vertices_2d)):
                    vertex=vertices_2d[i]
                    vertex_side=vertex[0]*A+vertex[1]*B+C
                    self.assertLess(origin_side*vertex_side, 0.0)
    '''
    def test_interval_calculations(self):
        target=[0.1+1.8*random.random(), 0.1+1.8*random.random(), 0.1+1.8*random.random()]
        self.mesh.set_target(target)
        alpha=math.pi*random.random()
        theta=math.pi*random.random()

        plane_intersection=self.mesh.slice(rotation=[alpha,theta])
        plane3d = pathfinder.Plane3d(id=0, alpha=alpha, theta=theta, target=np.array(target))
        plane2d = pathfinder.Plane2d(plane_intersection, plane3d)
        plane2d.calc_intervals_init()
        interval_bounds = plane2d.interval_bounds()

        all_vertices=list(itertools.chain.from_iterable([[tuple(v) for v in shape.vertices()] for shape in plane2d.shapes()]))
        intervals=sorted(set([math.atan2(v[1], v[0]) for v in all_vertices]))
        self.assertEqual(len(intervals), len(interval_bounds))
        for interval_bound in interval_bounds:
            self.assertLess(interval_bound[0], interval_bound[1])
            self.assertGreater(interval_bound[0], 0.0)

        #interval_shape_ids=plane2d.interval_shape_ids()

        for interval_idx, interval in enumerate(intervals):
            interval_shapes = [0]
            interval_start=interval
            if interval_idx==len(intervals)-1:
                interval_end=intervals[0] + 2 * math.pi
            else:
                interval_end=intervals[interval_idx+1]
            
            for shape_idx in range(1, len(plane2d.shapes())):
                shape=plane2d.shapes()[shape_idx]
                for vertex_idx in range(1, shape.hull_supporting_idx()+1):
                    v0=shape.arranged_vertices()[vertex_idx-1]
                    v1=shape.arranged_vertices()[vertex_idx]
                    angle_start=math.atan2(v0[1], v0[0])
                    angle_end=math.atan2(v1[1], v1[0])
                    if angle_start>angle_end:
                        if interval_start<0:
                            angle_start -= 2 * math.pi
                        else:
                            angle_end += 2 * math.pi
                    if angle_start<=interval_start and angle_end>=interval_end:
                        interval_shapes.append(shape_idx)
                        break

            #build a second list from pathfinder
            pathfinder_interval_shapes=plane2d.interval_shape_ids(interval_idx)
    
            print(f'testing interval {interval_idx+1} of {len(intervals)}')
            self.assertListEqual(sorted(pathfinder_interval_shapes), sorted(interval_shapes))
    '''

    def test_unflatten_paths(self):
        target=[0.1+1.8*random.random(), 0.1+1.8*random.random(), 0.1+1.8*random.random()]
        self.mesh.set_target(target)

        paths=self.mesh.get_paths(search_planes=8, threads=1, width_bound=2)
        normals=[]
        for alpha_id in range(8):
            alpha=alpha_id*math.pi/8
            normals.append([])
            for theta_id in range(8):
                theta=theta_id*math.pi/8
                rotation_x=np.array([[1,0,0], [0,math.cos(alpha),math.sin(alpha)], [0,-math.sin(alpha),math.cos(alpha)]])
                rotation_y=np.array([[math.cos(theta),0,math.sin(theta)], [0,1,0], [-math.sin(theta),0,math.cos(theta)]])
                normal=(np.matmul(rotation_x, rotation_y))[2]
                normals[alpha_id].append(normal)

        for path in paths:
            plane_id=path.plane_id()
            theta_id=plane_id%8
            alpha_id=plane_id//8
            normal=normals[alpha_id][theta_id]
            pt0=path.points()[0]
            pt1=path.points()[1]
            self.assertLess(abs(np.dot(target-pt0, normal)), 10e-7)
            self.assertLess(abs(np.dot(target-pt1, normal)), 10e-7)
    '''
    def test_width_bound(self):
        target=[0.1+1.8*random.random(), 0.1+1.8*random.random(), 0.1+1.8*random.random()]
        self.mesh.set_target(target)

        paths=self.mesh.get_paths(search_planes=8, threads=1, width_bound=0.0001)
        print('len paths' + str(len(paths)))
        for path in paths:
            diff=path.points()[0]-path.points()[1]
            self.assertLessEqual((diff[0]**2 + diff[1]**2 + diff[2]**2)**0.5, 0.0001)
    '''
   
    def test_pruned_intervals(self):
        for i in range(200):
            target=[0.1+1.8*random.random(), 0.1+1.8*random.random(), 0.1+1.8*random.random()]
            self.mesh.set_target(target)
            alpha=math.pi*random.random()
            theta=math.pi*random.random()

            plane_intersection=self.mesh.slice(rotation=[alpha,theta])
            plane3d = pathfinder.Plane3d(id=0, alpha=alpha, theta=theta, target=np.array(target))
            plane2d = pathfinder.Plane2d(plane_intersection, plane3d)
            plane2d.calc_intervals_init()
            init_interval_bounds = plane2d.interval_bounds()

            all_vertices=list(itertools.chain.from_iterable([[tuple(v) for v in shape.vertices()] for shape in plane2d.shapes()]))
            intervals=sorted(set([math.atan2(v[1], v[0]) for v in all_vertices]))

            plane2d = pathfinder.Plane2d(plane_intersection, plane3d)
            paths = plane2d.find_paths(width_bound=0.01)

            intervals_found=[-1 for i in range(len(paths))]

            for path_idx, path in enumerate(paths):
                points=path.points()
                path_angle_start=math.atan2(points[0][1], points[0][0])
                path_angle_end=math.atan2(points[1][1], points[1][0])
                
                if path_angle_start>path_angle_end:
                    intervals_found[path_idx]=len(intervals)-1
                    continue

                path_angle_mid=(path_angle_start+path_angle_end)/2.0
                for interval_idx, interval in enumerate(intervals):
                    interval_start=interval
                          
                    if interval_idx==len(intervals)-1:
                        interval_end=intervals[0]
                        if path_angle_mid<interval_end or path_angle_mid>interval_start:
                            intervals_found[path_idx]=interval_idx
                            break
                    else:
                        interval_end=intervals[interval_idx+1]
                        if path_angle_mid>interval_start and path_angle_mid<interval_end:
                            intervals_found[path_idx]=interval_idx
                            break

            self.assertTrue(all([element>-1 for element in intervals_found]))
            for path_idx, path in enumerate(paths):
                interval_idx=intervals_found[path_idx]
                interval_bounds=init_interval_bounds[interval_idx]
                self.assertGreaterEqual(path.lower_bound(), interval_bounds[0])
                self.assertLessEqual(path.upper_bound(), interval_bounds[1])


if __name__ == '__main__':
    unittest.main()
