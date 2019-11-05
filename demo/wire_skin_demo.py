# Don't run me directly!
# Take a look at wire_skin_demo.blend

import bpy
import sys
from importlib import reload

# Get wire_skin from current directory
sys.path.append(bpy.path.abspath("//"))

import wire_skin
reload(wire_skin)

from wire_skin import WireSkin

def main():
  # A bunch of layers have demos

  # Generic mesh
  layer1()
  
  # Creased mesh
  layer2()

  # Spiky mesh
  layer3()
  
  # Generating multiple from same wire
  layer4()

  # Displace with booleans
  layer5()

  # Torus made of hexagons
  layer6()

  # Torus made of hexagons, proportial scale
  layer7()

  # 'Edges without poles' option
  layer8()

def wire_to_skin(in_name, out_name, **kwargs):
  input_object = bpy.data.objects[in_name]
  output_object = bpy.data.objects[out_name]
  output_materials = output_object.data.materials

  wire_skin = \
    WireSkin(input_object.data, **kwargs)

  me = wire_skin.create_mesh()
  for material in output_materials:
    me.materials.append(material)
  output_object.data = me

def layer1():
  options = {
    'width': 0.15,
    'height': 0.1,
    'inside_radius': 0.3,
    'outside_radius': 0.8,
    'dist': 0.4
  }
  wire_to_skin("Wire1", "WireSkin1", **options)

def layer2():
  options = {
    'width': 0.1,
    'height': 0.2,
    'inside_radius': 0.1,
    'outside_radius': 0.1,
    'dist': 0.2,
    'crease': 1.0
  }
  wire_to_skin("Wire2", "WireSkin2", **options)

def layer3():
  options = {
    'width': 0.1,
    'height': 0.2,
    'inside_radius': 0.1,
    'outside_radius': 0.8,
    'dist': 0.2,
  }
  wire_to_skin("Wire3", "WireSkin3", **options)

def layer4():
  options = {
    'width': 0.5,
    'height': 0.2,
    'inside_radius': 0.1,
    'outside_radius': 0.1,
    'dist': 0.8,
  }
  wire_to_skin("Wire4", "WireSkin4", **options)

  options = {
    'width': 0.3,
    'height': 0.3,
    'inside_radius': 0.1,
    'outside_radius': 0.1,
    'dist': 0.8,
  }
  wire_to_skin("Wire4", "WireSkin4a", **options)

  options = {
    'width': 0.1,
    'height': 0.4,
    'inside_radius': 0.1,
    'outside_radius': 0.1,
    'dist': 0.8,
  }
  wire_to_skin("Wire4", "WireSkin4b", **options)

def layer5():
  options = {
    'width': 0.5,
    'height': 0.3,
    'inside_radius': 0.1,
    'outside_radius': 0.1,
    'dist': 1.0,
    'crease': 1.0
  }
  wire_to_skin("Wire5", "WireSkin5", **options)

  # This one is on layer 15
  # It is subtracted from previous object
  options = {
    'width': 0.3,
    'height': 0.3,
    'inside_radius': 0.1,
    'outside_radius': 0.1,
    'dist': 0.8,
    'crease': 1.0,
    'displace': 0.15
  }
  wire_to_skin("Wire5", "WireSkin5a", **options)

  options = {
    'width': 0.05,
    'height': 0.3,
    'inside_radius': 0.1,
    'outside_radius': 0.1,
    'dist': 0.8,
    'crease': 1.0,
  }
  wire_to_skin("Wire5", "WireSkin5b", **options)

def layer6():
  options = {
    'width': 0.07,
    'height': 0.05,
    'inside_radius': 0.04,
    'outside_radius': 0.02,
    'dist': 0.025
  }
  wire_to_skin("Wire6", "WireSkin6", **options)

def layer7():
  options = {
    'width': 0.3,
    'height': 0.3,
    'inside_radius': 0.2,
    'outside_radius': 0.1,
    'dist': 0.3,
    'proportional_scale': True
  }
  wire_to_skin("Wire7", "WireSkin7", **options)

def layer8():
  options = {
    'width': 0.15,
    'height': 0.1,
    'inside_radius': 0.3,
    'outside_radius': 0.8,
    'dist': 0.4
  }
  wire_to_skin("Wire8", "WireSkin8", **options)
  options['edges_without_poles'] = True
  wire_to_skin("Wire8", "WireSkin8a", **options)

main()
