import bpy
from math import sqrt, sin, cos, atan2, pi
from mathutils import Matrix, Vector

class VertCap:
  def __init__(self, v, **kwargs):
    self.input_vert = Vector(v.co)
    self.input_edge_verts = []

    if 'radius' in kwargs:
      self.radius = kwargs['radius']
    else:
      self.radius = 0.1

    if 'dist' in kwargs:
      self.dist = kwargs['dist']
    else:
      self.dist = 0,25

    if 'inside_radius' in kwargs:
      self.inside_radius = kwargs['inside_radius']
    else:
      self.inside_radius = self.radius

    if 'outside_radius' in kwargs:
      self.outside_radius = kwargs['outside_radius']
    else:
      self.outside_radius = self.radius

    # two vert indices
    self.poles = []
    # ordered array of profiles around each edge
    self.profiles = []
    # Need to find a profile given the edge it sits on
    self.edge_profile_map = {}

    # Output ... (faces need reindexing when added to mesh)
    self.verts = []
    self.faces = []

    # I'm not proud of this ... it's used to create connections between
    # vert caps
    self.index_base = None

  def add_edge_vert(self, edge, v):
    edge_vert = { 'v': Vector(v.co),
                  'e': edge }

    self.input_edge_verts.append(edge_vert)

  def compute_cap(self):
    self.create_pole_verts()
    self.reorder_edge_verts()
    self.create_profile_verts()
    self.create_pole_faces()
    self.create_inter_profile_faces()

  def create_pole_verts(self):
    if len(self.input_edge_verts) == 0:
      return

    vave = Vector((0.0, 0.0, 0.0))
    for ev in self.input_edge_verts:
      vert = ev['v']
      vave += (vert - self.input_vert).normalized()

    vave *= (1.0/len(self.input_edge_verts))

    if vave.magnitude > 0.0000001:
      vvec = self.input_vert - vave
      if vvec.magnitude > 0.0000001:
        vvec = vvec.normalized()
        numverts = len(self.verts)
        self.verts.append(self.input_vert + vvec * self.outside_radius)
        self.verts.append(self.input_vert - vvec * self.inside_radius)
        self.poles += [numverts, numverts + 1]

  def reorder_edge_verts(self):
    # Treat the pole as the up vector, and project the edge verts
    # to the plane normal to the pole. Examine the projected verts in
    # polar coordinates with the first edge vert having theta = 0
    # (on x axis). Reorder the others by theta value
    to_z = (self.verts[self.poles[0]] - self.input_vert).normalized()
    vedge = self.input_edge_verts[0]['v'] - self.input_vert
    to_y = to_z.cross(vedge).normalized()
    to_x = to_y.cross(to_z).normalized()
    mat = Matrix((to_x, to_y, to_z))
    mat.transpose()
    mat.invert()
    theta_edge_verts = {}
    for i in range(1, len(self.input_edge_verts)):
      edge_vert = self.input_edge_verts[i]['v']
      v = mat * (edge_vert - self.input_vert)
      theta = atan2(v[1], v[0])
      if theta < 0.0:
        theta += 2 * pi
      theta_edge_verts[theta] = self.input_edge_verts[i]

    sorted_edge_verts = [self.input_edge_verts[0]]
    for key in sorted(theta_edge_verts):
      sorted_edge_verts.append(theta_edge_verts[key])
    # Whew, what a pain that was ...
    self.input_edge_verts = sorted_edge_verts
    
  def create_profile_verts(self):
    # Note that the list below is sorted ...
    for edge_vert in self.input_edge_verts:
      self.create_profile_vert(edge_vert)

  def create_profile_vert(self, edge_vert):
    # For now, squares

    vert = edge_vert['v']
    # This vector points in the direction of the pole from the vert
    vpole = self.verts[self.poles[0]] - self.input_vert
    # This vector points down the edge
    etangent = (vert - self.input_vert).normalized()
    # The center of the profile, sitting on the edge
    ecenter = self.input_vert + (etangent * self.dist)

    # Normal to the pole and the tangent vector
    ebinormal = vpole.cross(etangent).normalized() * self.radius
    enormal = etangent.cross(ebinormal).normalized() * self.radius

    numverts = len(self.verts)
    self.verts += [
      ecenter + enormal + ebinormal,
      ecenter + enormal - ebinormal,
      ecenter - enormal - ebinormal,
      ecenter - enormal + ebinormal ]

    profile = [numverts, numverts + 1, numverts + 2, numverts + 3]

    self.profiles.append(profile)
    self.edge_profile_map[edge_vert['e'].index] = profile

  def get_edge_profile(self, edge):
    # Remapping the edge profiles so that faces can be created in the mesh
    profile = self.edge_profile_map[edge.index]
    return [(idx + self.index_base) for idx in profile]

  def create_pole_faces(self):
    for profile in self.profiles:
      self.faces.append([self.poles[0], profile[1], profile[0]])
      self.faces.append([self.poles[1], profile[3], profile[2]])

  def create_inter_profile_faces(self):
    num_profiles = len(self.profiles)
    for i in range(num_profiles):
      this_i = i
      next_i = (i+1) % num_profiles
      this_profile = self.profiles[this_i]
      next_profile = self.profiles[next_i]
      # Top triangle
      self.faces.append([self.poles[0], this_profile[0], next_profile[1]])
      # Bottom triange
      self.faces.append([self.poles[1], next_profile[2], this_profile[3]])
      # A quad inbetween the two triangles
      self.faces.append([this_profile[0], this_profile[3], \
                         next_profile[2], next_profile[1], ])

class WireSkin:
  def __init__(self, mesh, **kwargs):
    self.mesh = mesh

    # Has radius, dist, etc.
    self.kwargs = kwargs

    self.vert_caps = []

  def compute(self):
    # Returns a mesh
    self.calc_vert_caps()

    # TODO ...
    # self.join_vert_caps()

    return self.create_mesh()

  def create_mesh(self):
    verts = []
    faces = []

    vert_index = 0
    for vert_cap in self.vert_caps:
      vert_index = len(verts)
      # See comment above about not being proud ...
      vert_cap.index_base = vert_index
      verts += vert_cap.verts
      for face in vert_cap.faces:
        faces.append([x + vert_index for x in face])

    # Joining the vert caps ...
    for edge in self.mesh.edges:
      vc0 = self.vert_caps[edge.vertices[0]]
      vc1 = self.vert_caps[edge.vertices[1]]
      p0 = vc0.get_edge_profile(edge)
      p1 = vc1.get_edge_profile(edge)
      for idx in range(4):
        idx1 = (idx + 1) % 4
        idx2 = (0 - idx) % 4
        idx3 = (1 - idx) % 4
        faces.append([p0[idx], p0[idx1], p1[idx2], p1[idx3]])

    me = bpy.data.meshes.new('wireskin')
    me.from_pydata(verts, [], faces)
    return me

  def calc_vert_caps(self):
    for vert in self.mesh.vertices:
      self.vert_caps.append(VertCap(vert, **self.kwargs))

    for edge in self.mesh.edges:
      for i in range(2):
        this_vert_index = edge.vertices[i]
        other_vert_index = edge.vertices[(i + 1) % 2]
        other_vert = self.mesh.vertices[other_vert_index]
        self.vert_caps[this_vert_index].add_edge_vert(edge, other_vert)

    for vert_cap in self.vert_caps:
      vert_cap.compute_cap()

  def join_vert_caps(self):
    for edge in self.mesh.edges:
      polygons = []
      for i in range(2):
        polygons[i] = self.vert_cap[edge.vertex[i]].get_edge_polygon(edge)
