import bpy
import bmesh
from pathlib import Path
from time import time
from pprint import pprint

# https://docs.blender.org/api/current/bpy.ops.wm.html#bpy.ops.wm.usd_export
# bpy.ops.export_mesh.stl(filepath='/Users/rmbutler/Desktop/PhD/Github/AD-SDL/rpl_omniverse/robots/ot2/ot2.stl')

def convert_blend_to_usd(fname_blend, fname_usd, verbose=False):
    # Load .blend file
    bpy.ops.wm.open_mainfile(filepath=fname_blend)
    if bpy.context.active_object is not None:
        bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')

    # # Apply all modifiers
    # depsgraph = bpy.context.evaluated_depsgraph_get()
    # for obj in bpy.context.scene.collection.objects:
    #     if obj is None or obj.hide_get() or obj.type not in {'MESH', 'CURVE', 'SURFACE', 'FONT', 'META'}:
    #         continue

    #     # Get the mesh with modifiers applied
    #     object_eval = obj.evaluated_get(depsgraph)
    #     mesh = bpy.data.meshes.new_from_object(object_eval)

    #     # In case the mesh somehow evaporated
    #     if mesh is None:
    #         continue

    #     # Triangulate faces
    #     bm = bmesh.new()
    #     bm.from_mesh(mesh)
    #     bmesh.ops.triangulate(bm, faces=bm.faces[:])
    #     bm.to_mesh(mesh)
    #     bm.free()

    #     mesh.name = obj.name + '_mesh'
    #     oldmesh = obj.data
    #     obj.data = mesh
    #     bpy.data.meshes.remove(oldmesh)

    # # Delete hidden objects
    # hidden = [obj for obj in bpy.context.scene.collection.objects if obj.hide_get()]
    # override = bpy.context.copy()
    # override['selected_objects'] = hidden
    # with bpy.context.temp_override(**override):
    #     bpy.ops.object.delete()

    # Parent everything under a 'World' object (for usd)
    world = bpy.data.objects.new('World', None)
    bpy.context.scene.collection.objects.link(world)
    # bpy.context.view_layer.active_layer_collection.collection.objects.link(world)
    world.select_set(True)
    for obj in bpy.context.scene.collection.objects:
        if not obj.parent and not obj.hide_get() and obj is not world:
            obj.parent = world
            obj.select_set(True)

    # Export .usd file
    bpy.ops.wm.usd_export(
        filepath=fname_usd,
        check_existing=True,
        selected_objects_only=True,
        visible_objects_only=True,
        export_animation=False,
        export_hair=False,
        export_uvmaps=True,
        export_normals=True,
        export_materials=False,
        use_instancing=True,
        generate_preview_surface=False,
        export_textures=True,
        overwrite_textures=True,
        relative_paths=True
    )

def main():
    convert_blend_to_usd('tip_rack_20.blend', 'tip_rack_20.usda', True)
    return

    for fname_blend in Path('../cad/').rglob('*.blend'):
        s = time()

        b1 = fname_blend.with_suffix('.blend1')
        if b1.is_file():
            b1.unlink()

        fname_usd = list(fname_blend.parts)
        fname_usd[-2] = '2_export'
        fname_usd = Path(*fname_usd).with_suffix('.usda')

        if fname_usd.is_file():
            print('Replacing', fname_usd.name)
        else:
            print('Creating', fname_usd.name)

        convert_blend_to_usd(str(fname_blend), str(fname_usd))

        e = time()
        print(f'Finished {fname_blend.with_suffix("").name} in {round(e-s, 2)}s')

if __name__ == '__main__':
    main()
