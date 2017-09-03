import bpy, bmesh
from math import sqrt, sin, cos, atan2, pi
from mathutils import Matrix, Vector

class VertCap:
  def __init__(self, v, bm, **kwargs):
    self.input_vert = Vector(v.co)
    self.bm = bm
    self.input_edge_verts = []

    if 'width' in kwargs:
      self.width_2 = kwargs['width'] / 2.0
    else:
      self.width_2 = 0.1

    if 'height' in kwargs:
      self.height_2 = kwargs['height'] / 2.0
    else:
      self.height_2 = self.width_2

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

    if 'crease' in kwargs:
      self.crease = kwargs['crease']
    else:
      self.crease = None

    if 'displace' in kwargs:
      self.displace = kwargs['displace']
    else:
      self.displace = None

    # two vert indices
    self.poles = []
    # ordered array of profiles around each edge
    self.profiles = []
    # Need to find a profile given the edge it sits on
    self.edge_profile_map = {}

    self.creased_edges = []

  def add_edge_vert(self, edge, v):
    edge_vert = { 'v': Vector(v.co),
                  'e': edge }

    self.input_edge_verts.append(edge_vert)

  def compute_cap(self):
    if len(self.input_edge_verts) == 0:
      return

    self.create_pole_verts()
    self.reorder_edge_verts()
    self.create_profiles()
    self.create_pole_faces()
    self.create_inter_profile_faces()

  def create_pole_verts(self):
    vave = Vector((0.0, 0.0, 0.0))
    for ev in self.input_edge_verts:
      vert = ev['v']
      vave += (vert - self.input_vert).normalized()

    if vave.magnitude > 0.0000001:
      vave = vave.normalized()
      # Outside pole
      mult = self.outside_radius
      if self.displace:
        mult += self.displace
      v1 = self.bm.verts.new(self.input_vert - vave * mult)
      self.poles.append(v1)
      if len(self.input_edge_verts) > 1:
        mult = -self.inside_radius
        if self.displace:
          mult += self.displace
        v2 = self.bm.verts.new(self.input_vert - vave * mult)
        self.poles.append(v2)

  def reorder_edge_verts(self):
    # Treat the pole as the up vector, and project the edge verts
    # to the plane normal to the pole. Examine the projected verts in
    # polar coordinates with the first edge vert having theta = 0
    # (on x axis). Reorder the others by theta value
    if len(self.input_edge_verts) < 2:
      return

    to_z = (Vector(self.poles[0].co) - self.input_vert).normalized()
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
    
  def create_profiles(self):
    # Note that the list below is sorted ...
    for edge_vert in self.input_edge_verts:
      self.create_profile(edge_vert)

  def create_profile(self, edge_vert):
    vert = edge_vert['v']
    # This vector points in the direction of the pole from the vert
    vpole = Vector(self.poles[0].co) - self.input_vert
    # This vector points down the edge
    etangent = (vert - self.input_vert).normalized()
    # The center of the profile, sitting on the edge
    ecenter = self.input_vert + (etangent * self.dist)

    if len(self.input_edge_verts) < 2:
      # When the vert cap only has one edge coming to it,
      # then vpole and etangent are parallel, and cross products
      # blow up. Total kludge, but lets fudge things a little
      fudge = Vector((1, 0, 0))
      if abs(abs(fudge * etangent) - etangent.magnitude) > 0.0001:
        vpole = etangent + fudge
      else:
        vpole = etangent + Vector((1, 1, 0))

    # Normal to the pole and the tangent vector
    ebinormal = vpole.cross(etangent).normalized() * self.width_2
    enormal = etangent.cross(ebinormal).normalized()
    if self.displace:
      ecenter += self.displace * enormal
    enormal *= self.height_2

    v1 = self.bm.verts.new(ecenter + enormal + ebinormal)
    v2 = self.bm.verts.new(ecenter + enormal - ebinormal)
    v3 = self.bm.verts.new(ecenter - enormal - ebinormal)
    v4 = self.bm.verts.new(ecenter - enormal + ebinormal)

    profile = [v1, v2, v3, v4]

    self.profiles.append(profile)
    self.edge_profile_map[edge_vert['e'].index] = profile

  def get_edge_profile(self, edge):
    # Remapping the edge profiles so that faces can be created in the mesh
    return self.edge_profile_map[edge.index]

  def create_pole_faces(self):
    for profile in self.profiles:
      if len(self.input_edge_verts) > 1:
        self.bm.faces.new([self.poles[1], profile[3], profile[2]])
        self.bm.faces.new([self.poles[0], profile[1], profile[0]])
      else:
        for i in range(4):
          self.bm.faces.new([self.poles[0], profile[(i+1) % 4], profile[i]])

  def create_inter_profile_faces(self):
    if len(self.input_edge_verts) < 2:
      return

    num_profiles = len(self.profiles)
    for i in range(num_profiles):
      this_i = i
      next_i = (i+1) % num_profiles
      this_profile = self.profiles[this_i]
      next_profile = self.profiles[next_i]
      # Top triangle
      self.bm.faces.new([self.poles[0], this_profile[0], next_profile[1]])
      # Bottom triange
      self.bm.faces.new([self.poles[1], next_profile[2], this_profile[3]])
      # A quad inbetween the two triangles
      self.bm.faces.new([this_profile[0], this_profile[3], \
                         next_profile[2], next_profile[1], ])

      if self.crease != None:
        self.creased_edges.append(self.bm.edges.get([this_profile[0],
                                                     next_profile[1]]))
        self.creased_edges.append(self.bm.edges.get([next_profile[2],
                                                     this_profile[3]]))

  def crease_edges(self, crease_layer):
    for edge in self.creased_edges:
      edge[crease_layer] = self.crease

class ProfileConnector:
  def __init__(self, edge, vert_caps, bm, **kwargs):
    self.edge = edge
    self.vert_caps = vert_caps
    self.bm = bm

    if 'crease' in kwargs:
      self.crease = kwargs['crease']
    else:
      self.crease = None

    self.creased_edges = []

  def join_profiles(self):
    # We have two profiles that need to be connected that are
    # on other ends of an edge. Unfortunately the points in
    # the profiles aren't ordered in a meaningful way that will
    # help us create the faces. We need to figure out which
    # vertices on the opposite profiles are closest to each other.
    vc0 = self.vert_caps[self.edge.vertices[0]]
    vc1 = self.vert_caps[self.edge.vertices[1]]

    loop0 = vc0.get_edge_profile(self.edge)
    loop1 = vc1.get_edge_profile(self.edge)

    v0 = Vector(loop0[0].co)
    v1 = Vector(loop0[1].co)

    vloop1 = [Vector(p.co) for p in loop1]

    lengths = [(v0 - p).magnitude for p in vloop1]
    min_len = lengths[0]
    min_i = 0
    for i in range(1, len(lengths)):
      if lengths[i] < min_len:
        min_len = lengths[i]
        min_i = i

    len1 = (vloop1[(min_i + 1) % 4] - v1).magnitude
    len2 = (vloop1[(min_i - 1) % 4] - v1).magnitude

    mult = 1
    if len1 < len2:
      min_i2 = (min_i + 1) % 4
    else:
      min_i2 = (min_i - 1) % 4
      mult = -1

    for idx in range(4):
      idx1 = (idx + 1) % 4
      idx2 = (min_i2 + mult * idx) % 4
      idx3 = (min_i + mult * idx) % 4
      self.bm.faces.new([loop0[idx], loop0[idx1], loop1[idx2], loop1[idx3]])
      if self.crease != None:
        self.creased_edges.append(self.bm.edges.get([loop0[idx1], \
                                                     loop1[idx2]]))

  def crease_edges(self, crease_layer):
    for edge in self.creased_edges:
      edge[crease_layer] = 1.0

class WireSkin:
  def __init__(self, mesh, **kwargs):
    self.mesh = mesh
    self.crease_layer = None

    # Has radius, dist, etc.
    self.kwargs = kwargs

    self.vert_caps = []
    self.profile_connectors = []

  def create_mesh(self):
    bm = bmesh.new()
    if 'crease' in self.kwargs:
      if self.kwargs['crease'] != None:
        self.crease_layer = bm.edges.layers.crease.new()

    self.create_vert_caps(bm)
    self.connect_profiles(bm)
    self.crease_edges(bm)

    me = bpy.data.meshes.new('wireskin')
    bm.to_mesh(me)
    bm.free()
    return me

  ### Below should be private

  def create_vert_caps(self, bm):
    for vert in self.mesh.vertices:
      self.vert_caps.append(VertCap(vert, bm, **self.kwargs))

    for edge in self.mesh.edges:
      for i in range(2):
        this_vert_index = edge.vertices[i]
        other_vert_index = edge.vertices[(i + 1) % 2]
        other_vert = self.mesh.vertices[other_vert_index]
        self.vert_caps[this_vert_index].add_edge_vert(edge, other_vert)

    for vert_cap in self.vert_caps:
      vert_cap.compute_cap()

  def connect_profiles(self, bm):
    # Joining the vert caps ...
    for edge in self.mesh.edges:
      profile_connector =\
        ProfileConnector(edge, self.vert_caps, bm, **self.kwargs)
      self.profile_connectors.append(profile_connector)
      profile_connector.join_profiles()

  def crease_edges(self, bm):
    if self.crease_layer:
      for vert_cap in self.vert_caps:
        vert_cap.crease_edges(self.crease_layer)
      for profile_connector in self.profile_connectors:
        profile_connector.crease_edges(self.crease_layer)
