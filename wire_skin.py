import bpy
from math import sqrt, sin, cos, atan2, pi
from mathutils import Matrix, Vector

class VertCap:
  def __init__(self, v, radius, dist):
    self.input_vert = v
    self.input_edge_verts = []
    self.radius = radius
    self.dist = dist

    # two vert indices
    self.poles = []
    # ordered array of profiles around each edge
    self.profiles = []
    # array of faces pointing to vert indices
    self.profile_pole_triangles = []
    # array of faces pointing to vert indices
    self.inter_profile_filler = []

    self.verts = []
    self.faces = []

  def add_edge_vert(self, v):
    self.input_edge_verts.append(v)

  def compute_cap(self):
    self.create_pole_verts()
    self.create_profile_verts()
    self.create_pole_faces()
    self.create_inter_profile_faces()

  def create_pole_verts(self):
    if len(self.input_edge_verts) == 0:
      return

    vvert = Vector(self.input_vert.co)
    # Bogus ...
    vave = Vector((0.0, 0.0, 0.0))
    for vert in self.input_edge_verts:
      vave += Vector(vert.co)

    vave *= (1.0/len(self.input_edge_verts))

    if vave.magnitude > 0.0000001:
      vvec = vave - vvert
      if vvec.magnitude > 0.0000001:
        vvec = vvec.normalized() * self.radius
        self.verts.append(vvert + vvec)
        self.verts.append(vvert - vvec)

  def create_profile_verts(self):
    #self.calc_ordered_square_locations()
    pass

  def create_pole_faces(self):
    pass

  def create_inter_profile_faces(self):
    pass

class WireSkin:
  def __init__(self, mesh, radius, dist):
    self.radius = radius
    self.dist = dist

    self.mesh = mesh

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
      verts += vert_cap.verts
      for face in vert_cap.faces:
        faces.append([x + vert_index for x in face])

    me = bpy.data.meshes.new('wireskin')
    me.from_pydata(verts, [], faces)
    return me

  def calc_vert_caps(self):
    for vert in self.mesh.vertices:
      self.vert_caps.append(VertCap(vert, self.radius, self.dist))

    for edge in self.mesh.edges:
      for i in range(2):
        this_vert_index = edge.vertices[i]
        other_vert_index = edge.vertices[(i + 1) % 2]
        other_vert = self.mesh.vertices[other_vert_index]
        self.vert_caps[this_vert_index].add_edge_vert(other_vert)

    for vert_cap in self.vert_caps:
      vert_cap.compute_cap()

  def join_vert_caps(self):
    for edge in self.mesh.edges:
      polygons = []
      for i in range(2):
        polygons[i] = self.vert_cap[edge.vertex[i]].get_edge_polygon(edge)
