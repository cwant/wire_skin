* [Click me to skip all the blah-blah-blah and look at some **pictures**.](#the-demo-file)
* [Click me to jump to the **options**.](#okay-but-how-do-i-control-this-thing)

# wire_skin
A python script to construct a simple skin around a wire frame mesh in Blender.

```
+-----+     #=====#   (My ASCII art sucks,
|     |  => ||   ||    but you get the picture)
+-----+     #=====#
```

Non-ASCII version:

![Screenshot](documentation/wire_skin_demo1.png?raw=true "Screenshot")

## Installation

This software works with Blender 2.8 and earlier versions too.

Check out the repository and look in the demo directory.

This software may also be installed via pip:

python3 -m pip install wire_skin

or

pip3 install wire_skin

## Motivation
Blender has a skin modifier that does great things for a lot of meshes. It doesn't seem to do the right thing for some of the sorts of meshes I want 3D printed (usually the caps where the vertices are have an asymmetry that is displeasing to me). The meshes I am trying to create are typically geometric cages for presenting other 3D objects.

This script is meant to address this for me. It certainly isn't a general purpose skinning script, and doesn't pretend to be -- it will likely fail if your input mesh doesn't match a use case similar to mine. I hope the script is useful to you, but really the Blender skin modifier should be more than adequate for most purposes.

Play around with the demo blend file before adapting the script (imported as a module) into your project.

## That's great Chris, but how does it work?

We break the problem down into two parts: calculating "VertCaps" for each vertex in the input mesh, and joining the VertCaps using profile connectors.

![Screenshot](documentation/wire_skin_demo2.png?raw=true "Screenshot")

Each VertCap can be decomposed further. The vertices that are needed to make a VertCap are comprised of a couple of "poles" that stradle either side of the concavity of the vertex, and some profile shapes (at this stage of development, rectangles) that go around each edge connected to the vertex.

![Screenshot](documentation/wire_skin_demo3.png?raw=true "Screenshot")

Next, faces connect these vertices. These are broken into two groups: the faces (triangles) that connect the profiles to a pole, and strips of faces that run between the profiles that connect the pole (a triangle, a quad, and another triangle).

![Screenshot](documentation/wire_skin_demo4.png?raw=true "Screenshot")

At this point, the VertCap is complete.

Quads connect the VertCaps together along edges, and this is pretty straight forward.

## Okay, but how do I control this thing?

Well, take a look at the demo file. The `WireSkin` object generates a mesh when it's `create_mesh()` method is called. The object is initialized with a mesh as input, and a number of keyword arguments:

* `dist`: distance between each vertex in the input mesh and the profile vertices described above.
* `width`: the width of the rectangular profiles.
* `height`: the height of the rectangular profiles.
* `outside_radius`: the distance between a vertex in the base mesh and a outside pole vertex in a VertCap
* `inside_radius`: the distance between a vertex in the base mesh and a inside pole vertex in a VertCap.

A couple of cryptic illustrations should make this clear:

![Screenshot](documentation/wire_skin_demo5.png?raw=true "Screenshot")
![Screenshot](documentation/wire_skin_demo6.png?raw=true "Screenshot")

But wait, there's more! ...

* `crease`: this sets the subsurf edge creases for edges that surround the holes in the output mesh. Set this to a float between `0.0` and `1.0` or `None`.

![Screenshot](documentation/wire_skin_demo_crease_0.0.png?raw=true "Screenshot")
![Screenshot](documentation/wire_skin_demo_crease_1.0.png?raw=true "Screenshot")

* `displace`: this float value moves each vert cap in the direction of the outside pole.

![Screenshot](documentation/wire_skin_demo_displace.png?raw=true "Screenshot")
w

* `proportional_scale`: This is a boolean option. When this option is selected, the various length settings become proportions of the edge lengths near the vert cap:
  * pole measurements `inside_radius` and `outside_radius` are proportial to the average length of the incoming edges to a vertex;
  * profile measurements `dist`, `width`, and `height` are proportial to the edge that the profile is on.

Now how much would you pay? But wait:

* `edges_without_poles`: This is a boolean option. When this option is selected, the vertices that are connected to two edges (basically a bisected edge) do not have a full VertCap on them -- just a profile is produced at the vertex, not poles. This allows the user some control of the curvature on edges without. Take a look at the demo file.

## The demo file

The demo file (wire_skin_demo.blend) had five examples on layers 1 - 5. The script in the text window generates all of the examples (alt-P to run when your mouse is in the text window).

### Layer 1

Just a cube-ish shape with some sharp corners:

![Screenshot](documentation/wire_skin_demo_layer1.png?raw=true "Screenshot")

### Layer 2

An example using creasing:

![Screenshot](documentation/wire_skin_demo_layer2.png?raw=true "Screenshot")

### Layer 3

A spikey thingy:

![Screenshot](documentation/wire_skin_demo_layer3.png?raw=true "Screenshot")

### Layer 4

Using the same wire frame to generate a few intersecting objects with different parameters:

![Screenshot](documentation/wire_skin_demo_layer4.png?raw=true "Screenshot")

### Layer 5

Generating multiple creased objects from the same wireframe, but using a boolean difference with the displace option to make some interesting shapes:

![Screenshot](documentation/wire_skin_demo_layer5.png?raw=true "Screenshot")

### Layer 6

A honeycomb torus thingy:

![Screenshot](documentation/wire_skin_demo_layer6.png?raw=true "Screenshot")

### Layer 7

A honeycomb torus thingy, but using the proportional scale option:

![Screenshot](documentation/wire_skin_demo_layer7.png?raw=true "Screenshot")

### Layer 8

Demo of the "Edges without poles" feature:

![Screenshot](documentation/wire_skin_demo_layer8.png?raw=true "Screenshot")

## 3D Printing on Shapeways

I primarily use wire_skin to create objects for 3D printing (on Shapeways). I tag my models with the keyword `wire_skin`. If you also use Shapeways to print models made with wire_skin, feel free to use this tag -- I'd be very interested to see what you make!

https://www.shapeways.com/marketplace?q=wire_skin
