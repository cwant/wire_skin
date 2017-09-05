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

    if 'proportional_scale' in kwargs and kwargs['proportional_scale']:
      self.proportional_scale = True
    else:
      self.proportional_scale = False

    if 'edges_without_poles' in kwargs and kwargs['edges_without_poles']:
      self.edges_without_poles = True
    else:
      self.edges_without_poles = False

    # These are for verts with only two edges
    self.make_poles = True
    self.make_faces = True

    # two vert indices
    self.poles = []

    # Normal to vert (direction of poles)
    self.normal = None

    # ordered array of profiles around each edge
    self.profiles = []
    # Need to find a profile given the edge it sits on
    self.edge_profile_map = {}

    self.creased_edges = []

  def add_edge_vert(self, edge, v):
    vec = Vector(v.co)
    edge_vert = { 'v': vec,
                  'i': v.index,
                  'e': edge,
                  'l': (self.input_vert - vec).magnitude }

    self.input_edge_verts.append(edge_vert)

  def diff_average(self, diffs):
    vave = Vector((0.0, 0.0, 0.0))
    for diff in diffs:
      vave += diff
    return vave * (1.0/len(diffs))

  def least_squares_normal(self, vave, diffs):
    # Method taken from here:
    # http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
    # Get normal vector for plane that best fits the vectors in diffs
    n = len(diffs)
    centroid = vave

    # Calc full 3x3 covariance matrix, excluding symmetries:
    (xx, xy, xz, yy, yz, zz) = [0.0]*6

    for p in diffs:
      r = p - centroid;
      xx += r[0] * r[0]
      xy += r[0] * r[1]
      xz += r[0] * r[2]
      yy += r[1] * r[1]
      yz += r[1] * r[2]
      zz += r[2] * r[2]

    det_x = yy*zz - yz*yz
    det_y = xx*zz - xz*xz
    det_z = xx*yy - xy*xy

    det_max = max([abs(det_x), abs(det_y), abs(det_z)])
    if (det_max < 0.00001):
      # Give up
      return vave

    if det_max == abs(det_x):
      a = (xz*yz - xy*zz) / det_x
      b = (xy*yz - xz*yy) / det_x
      normal = Vector((1.0, a, b))
    elif det_max == abs(det_y):
      a = (yz*xz - xy*zz) / det_y
      b = (xy*xz - yz*xx) / det_y
      normal = Vector((a, 1.0, b))
    else:
      a = (yz*xy - xz*yy) / det_z
      b = (xz*xy - yz*xx) / det_z
      normal = Vector((a, b, 1.0))
    if vave * normal < 0.0:
      return -normal
    return normal

  def create_pole_verts(self):
    if len(self.input_edge_verts) == 0:
      return

    if len(self.input_edge_verts) == 2:
      if (self.edges_without_poles):
        self.make_poles = False
        self.make_faces = False

    # Unit vectors pointing in direction of connected edges
    diffs = [(ev['v'] - self.input_vert).normalized()
             for ev in self.input_edge_verts]
    vave = self.diff_average(diffs)

    # Things break if vave is really small (ambiguous curvature)
    if vave.magnitude > 0.0000001:
      if (len(self.input_edge_verts) < 3):
        # This is first guess at unit vector in pole direction
        self.normal = -vave.normalized()
      else:
        # This is second (better) guess
        self.normal = -self.least_squares_normal(vave, diffs).normalized()

      # Outside pole
      scale = 1.0
      if self.proportional_scale:
        # Scale of poles is proportional to average edge length
        scale = 0.0
        for edge_vert in self.input_edge_verts:
          scale += edge_vert['l']
        scale *= (1.0 / len(self.input_edge_verts))
      mult = self.outside_radius * scale
      if self.displace:
        mult += self.displace
      if self.make_poles:
        v1 = self.bm.verts.new(self.input_vert + self.normal * mult)
        self.poles.append(v1)
      if len(self.input_edge_verts) > 1:
        mult = -self.inside_radius * scale
        if self.displace:
          mult += self.displace
        if self.make_poles:
          v2 = self.bm.verts.new(self.input_vert + self.normal * mult)
          self.poles.append(v2)

  def reorder_edge_verts(self):
    # Treat the pole as the up vector, and project the edge verts
    # to the plane normal to the pole. Examine the projected verts in
    # polar coordinates with the first edge vert having theta = 0
    # (on x axis). Reorder the others by theta value
    if len(self.input_edge_verts) < 2:
      return

    to_z = self.normal
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
    
  def create_profiles(self, vert_caps):
    if len(self.input_edge_verts) == 0:
      return
    if len(self.input_edge_verts) > 2:
      self.reorder_edge_verts()
    if self.make_poles:
      # Note that the list below is sorted ...
      for edge_vert in self.input_edge_verts:
        other_normal = vert_caps[edge_vert['i']].normal
        self.create_regular_profile(edge_vert, other_normal)
    else:
      self.create_poleless_profiles()

  def create_poleless_profiles(self):
    edge_vert0 = self.input_edge_verts[0]
    edge_vert1 = self.input_edge_verts[1]
    vert0 = edge_vert0['v']
    etangent = vert0 - self.input_vert
    ecenter = self.input_vert
    enormal = self.normal.normalized()
    ebinormal = enormal.cross(etangent).normalized()

    scale = 1.0
    if self.proportional_scale:
      scale = (edge_vert0['l'] + edge_vert1['l']) / 2.0

    profile0 = self.create_profile_verts(ecenter, enormal, ebinormal, scale)
    profile1 = [profile0[(-i) % 4] for i in range(4)]

    self.profiles.append(profile0)
    self.profiles.append(profile1)
    self.edge_profile_map[edge_vert0['e'].index] = profile0
    self.edge_profile_map[edge_vert1['e'].index] = profile1

  def create_regular_profile(self, edge_vert, other_normal):
    scale = 1.0
    if self.proportional_scale:
      scale = edge_vert['l']
    dist = self.dist * scale

    vert = edge_vert['v']
    # This vector points down the edge
    etangent = vert - self.input_vert
    vpole = self.normal
    # Like to have a blended average of this normal and other vert cap normal
    # To do: use quaternions to blend via rotations
    if etangent.magnitude > 0.000001 and self.make_poles:
      proportion = min([0.5, dist/etangent.magnitude])
      vpole = self.normal * (1 - proportion) + other_normal * proportion

    etangent = etangent.normalized()

    # The center of the profile, sitting on the edge
    ecenter = self.input_vert + (etangent * dist)

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
    ebinormal = vpole.cross(etangent).normalized()
    enormal = etangent.cross(ebinormal).normalized()

    profile = self.create_profile_verts(ecenter, enormal, ebinormal, scale)

    self.profiles.append(profile)
    self.edge_profile_map[edge_vert['e'].index] = profile

  def create_profile_verts(self, ecenter, enormal, ebinormal, scale):
    width_2 = self.width_2 * scale
    height_2 = self.height_2 * scale
    if self.displace:
      ecenter += self.displace * enormal
    ebinormal *= width_2
    enormal *= height_2

    v1 = self.bm.verts.new(ecenter + enormal + ebinormal)
    v2 = self.bm.verts.new(ecenter + enormal - ebinormal)
    v3 = self.bm.verts.new(ecenter - enormal - ebinormal)
    v4 = self.bm.verts.new(ecenter - enormal + ebinormal)

    return [v1, v2, v3, v4]
    
  def get_edge_profile(self, edge):
    # Remapping the edge profiles so that faces can be created in the mesh
    if edge.index not in self.edge_profile_map:
      return None
    return self.edge_profile_map[edge.index]

  def create_pole_faces(self):
    if self.make_faces == False:
      return
    for profile in self.profiles:
      if len(self.input_edge_verts) > 1:
        self.bm.faces.new([self.poles[1], profile[3], profile[2]])
        self.bm.faces.new([self.poles[0], profile[1], profile[0]])
      else:
        for i in range(4):
          self.bm.faces.new([self.poles[0], profile[(i+1) % 4], profile[i]])

  def create_inter_profile_faces(self):
    if self.make_faces == False:
      return
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
    # help us create the faces.

    vc = [None] * 2; v = [None] * 2; loop = [None] * 2;
    for i in range(2):
      vc[i] = self.vert_caps[self.edge.vertices[i]]
      v[i] = vc[i].input_vert
      loop[i] = vc[i].get_edge_profile(self.edge)

    if not loop[0] or not loop[1]:
      return

    vloop = [None] * 2; up = [None] * 2; spin = [None] * 2; out = [None] * 2
    # Figure out orientation (do the loops rotate in the same direction?)
    for i in range(2):
      vloop[i] = [Vector(p.co) for p in loop[i]]
      up[i] = vloop[i][0] - v[i]
      spin[i] = vloop[i][1] - vloop[i][0]
      out[i] = up[i].cross(spin[i])

    if out[0] * out[1] <= 0:
      spin = -1
    else:
      spin = 1

    # Whelp, all of the above doesn't work, but this does:
    spin = -1

    # Figure out which rotation of loops yeilds edges with
    # least length
    lengths = [0.0] * 4
    for i in range(4):
      for j in range(4):
        lengths[i] += (vloop[0][j] - vloop[1][(i + j * spin) % 4]).magnitude

    min_len = lengths[0]
    min_i = 0
    for i in range(1, len(lengths)):
      if lengths[i] < min_len:
        min_len = lengths[i]
        min_i = i

    for idx in range(4):
      idx1 = (idx + 1) % 4
      idx2 = (min_i + (idx + 1) * spin) % 4
      idx3 = (min_i + idx * spin) % 4
      self.bm.faces.new([loop[0][idx], loop[0][idx1], loop[1][idx2], loop[1][idx3]])
      if self.crease != None:
        self.creased_edges.append(self.bm.edges.get([loop[0][idx1], \
                                                     loop[1][idx2]]))

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
    self.add_edges_to_vert_caps()
    self.create_vert_cap_poles()
    self.create_vert_cap_profiles()
    self.create_vert_cap_faces()
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

  def add_edges_to_vert_caps(self):
    for edge in self.mesh.edges:
      for i in range(2):
        this_vert_index = edge.vertices[i]
        other_vert_index = edge.vertices[(i + 1) % 2]
        other_vert = self.mesh.vertices[other_vert_index]
        self.vert_caps[this_vert_index].add_edge_vert(edge, other_vert)

  def create_vert_cap_poles(self):
    for vert_cap in self.vert_caps:
      vert_cap.create_pole_verts()

  def create_vert_cap_profiles(self):
    for vert_cap in self.vert_caps:
      vert_cap.create_profiles(self.vert_caps)

  def create_vert_cap_faces(self):
    for vert_cap in self.vert_caps:
      vert_cap.create_pole_faces()
      vert_cap.create_inter_profile_faces()

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
