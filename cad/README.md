# Directory Structure

## Resources
All unmodified external files for the object.
This can include meshes, textures, URDF's, or anything else provided by the manufacturer.
If the object was custom designed (e.g. in Blender), this may be empty.
If the file is a URDF, the relevant STL files should also be here.

## Edit
Typically, this directory will only contain a single .blend file, but if other files (like images) were created or are necessary for the .blend file to function, they should be stored here too.

## Export
There should be a .usdc file here, typically exported from Blender. (See rpl_omniverse/scripts/blend_to_usd.py)
Any other file types that need to be created for export should also be stored here.
If the object needs to be exported across several files of the same type, they should be grouped in a subdirectory.
