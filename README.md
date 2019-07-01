# pathfinder
C++/Python implementation of the minimum separation in a 3D weighted subdivision algorithm.

## Getting Started

### Prerequisites

Python 3, git, pip3.

### Dependencies

Install Eigen:

```
sudo install libeigen3-dev
```

### Installation

```
git clone https://github.com/myociss/pathfinder
pip3 install ./pathfinder
```

To run tests:

```
cd pathfinder
python3 setup.py test
```

## Usage

Initializing a mesh:

```
#!/usr/bin/env python
import pathfinder, json

with open(‘mesh.json’, ‘r’) as f:
  json_mesh=json.load(f)

mesh=pathfinder.Mesh(num_vertices=len(json_mesh[‘vertices’]),
  num_tetrahedrons=len(json_mesh[‘tetrahedrons’]),
  num_faces=len(json_mesh[‘faces’]))

```

Adding mesh components:

```
#!/usr/bin/env python
import pathfinder, json

...

mesh.set_vertices(json_mesh[‘vertices’])

for tet_idx, tet in enumerate(json_mesh[‘tetrahedrons’]):
  mesh.add_tetrahedron(tetrahedron_id=tet_idx,
    neighbor_ids=tet[‘neighbors’],
    vertex_ids=tet[‘vertices’], weight=tet[‘weight’])

for face in json_mesh[‘faces’]:
  mesh.add_face(vertex_ids=face[‘vertices’], 
    tetrahedron_id=face[‘tetrahedron’])
```

Locating and setting the target point inside a mesh:

```
#!/usr/bin/env python
import pathfinder, json, sys

...

target_pt=[0.2334, 0.8387, 0.96465]
mesh_contains_target=mesh.set_target(target_pt)

if not mesh_contains_target:
  sys.exit(‘the mesh object does not contain that point’)
```

Obtaining an individual intersection of the mesh with a plane containing the target point:

```
#!/usr/bin/env python
import pathfinder, json, sys, random, math

...

alpha=math.pi*random.random()
theta=math.pi*random.random()
plane_intersection=self.mesh.slice(rotation=[alpha, theta])
for shape_object in plane_intersection:
  if len(shape_object.vertices())==3:
    print(f‘tetrahedron {shape_object.tet_id} intersects this plane as a triangle!’)
  else:
    print(f‘tetrahedron {shape_object.tet_id} intersects this plane as a quadrilateral!’)
```

Obtaining a set of potential paths from a pathfinder mesh object. The mesh will call the prune/search algorithm for 100 planes, distributed among one thread per core available on the user’s machine:

```
#!/usr/bin/env python
import pathfinder, json, sys, random, math, multiprocessing

...


paths=mesh.get_paths(search_planes=10,
  threads=multiprocessing.cpu_count(), width_bound=0.002)

for path in paths:
  points = path.points()
  pt_1 = points[0]
  pt_2 = points[1]
  x = (pt_1[0]+pt_2[0])/2.0
  y = (pt_1[1]+pt_2[1])/2.0
  z = (pt_1[2]+pt_2[2])/2.0
  print(f’enter the domain at (x y z)=({x} {y} {z})’)
```