# wire_skin
A python script to construct a simple skin around a wire frame mesh in Blender.

```
+-----+     #=====#   (My ASCII art sucks,
|     |  => ||   ||    but you get the picture)
+-----+     #=====#
```
## Motivation
Blender has a skin modifier that does great things for a lot of meshes. It doesn't seem to do the right thing for some of the sorts of things I want 3D printed (usually the caps where the vertices are have an asymmetry that is displeasing to me). The meshes I am trying to create are typically geometric cages for presenting other 3D objects.

This script is meant to address this for me. It certainly isn't a general purpose skinning script, and doesn't pretend to be -- it will likely fail if your input mesh doesn't match a use case similar to mine. I hope the script is useful to you, but really the Blender skin modifier should be more than adequate for most purposes.

## An important note

Please note that this script is in development and currently doesn't do much. I am mostly testing some ideas about how I want my skin meshes created.
