from multiprocessing import Process
from pathlib import Path
from time import time
import sys

# https://docs.blender.org/api/current/bpy.ops.wm.html#bpy.ops.wm.usd_export
# bpy.ops.export_mesh.stl(filepath='')

def convert_blend_to_usd(fname_blend, fname_usd):
    import bpy
    import bmesh

    # Load .blend file
    bpy.ops.wm.open_mainfile(filepath=fname_blend)
    if bpy.context.active_object is not None:
        bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')

    # Parent everything under a 'World' object (for usd)
    world = bpy.data.objects.new('World', None)
    bpy.context.scene.collection.objects.link(world)
    world.select_set(True)

    # Touch up the scene before exporting
    for obj in bpy.context.scene.collection.objects:
        if not obj.hide_get() and obj is not world:
            # Change parent
            if not obj.parent:
                obj.parent = world

            # Triangulate with modifier
            m = obj.modifiers.new('Triangulate', 'TRIANGULATE')
            m.quad_method = 'BEAUTY'

            # Mesh names show up in OV, so make it pretty
            print('    Renaming', obj.data.name, 'to', obj.name + '_mesh')
            obj.data.name = obj.name + '_mesh'

            # Select so we can filter at usd_export
            obj.select_set(True)

    # Export .usd file
    bpy.ops.wm.usd_export(
        filepath=fname_usd,
        check_existing=True,
        selected_objects_only=True,
        visible_objects_only=True,
        export_animation=False,
        export_hair=True,
        export_uvmaps=True,
        export_normals=True,
        export_materials=True,
        use_instancing=True,
        generate_preview_surface=False,
        export_textures=True,
        overwrite_textures=True,
        relative_paths=True
    )

def modify_world_prim(fname_usd, verbose=False):
    from pxr import Usd, Sdf, Gf, Vt

    stage = Usd.Stage.Open(fname_usd)

    if verbose:
        for prim in stage.Traverse():
            print('   ', prim.GetPath())

    world = stage.GetPrimAtPath('/World')
    if not world.IsValid():
        print('#################################################################')
        print('######################## No /World found ########################')
        print('#################################################################')

        return

    stage.SetDefaultPrim(world)

    attr = world.CreateAttribute('xformOp:orient', Sdf.ValueTypeNames.Quatd)
    attr.Set(Gf.Quatd(1, Gf.Vec3d(0, 0, 0)))

    attr = world.CreateAttribute('xformOp:scale', Sdf.ValueTypeNames.Double3)
    attr.Set(Gf.Vec3d(1, 1, 1))

    attr = world.CreateAttribute('xformOp:translate', Sdf.ValueTypeNames.Double3)
    attr.Set(Gf.Vec3d(0, 0, 0))

    attr = world.CreateAttribute('xformOpOrder', Sdf.ValueTypeNames.TokenArray)
    attr.Set(Vt.TokenArray(3, ('xformOp:translate', 'xformOp:orient', 'xformOp:scale')))

    stage.Save()

def do(fname_blend, fname_usd):
    # This Process hack is dumb, but if you try to import bpy and pxr in the same
    # process on macOS you get an error:
    # ------------------------------ Python terminated -------------------------------
    # Python crashed. FATAL ERROR: [TF_DEBUG_ENVIRONMENT_SYMBOL] multiple debug symbol
    # definitions for 'TF_SCRIPT_MODULE_LOADER'.  This is usually due to software
    # misconfiguration, such as multiple versions of the same shared library loaded
    # simultaneously in the process.  Please check your build configuration.
    # in _Register at line 139 of /Users/runner/work/1/s/pxr/base/tf/debug.cpp
    # writing crash report to [ ### ] ... done.
    # --------------------------------------------------------------------------------

    p = Process(target=convert_blend_to_usd, args=(fname_blend, fname_usd,))
    p.start()
    p.join()

    p = Process(target=modify_world_prim, args=(fname_usd, True))
    p.start()
    p.join()

def main():
    S = time()

    filter_ = sys.argv[1:]

    for fname_blend in Path('../cad/').rglob('*.blend'):
        s = time()

        if len(filter_) and str(fname_blend.with_suffix('').name) not in filter_:
            continue

        b1 = fname_blend.with_suffix('.blend1')
        if b1.is_file():
            b1.unlink()

        fname_usd = list(fname_blend.parts)
        fname_usd[-2] = '2_export'
        fname_usd = Path(*fname_usd).with_suffix('.usdc')

        if fname_blend.stat().st_mtime < fname_usd.stat().st_mtime:
            continue

        if fname_usd.is_file():
            print('Replacing', fname_usd.name)
        else:
            print('Creating', fname_usd.name)

        do(str(fname_blend), str(fname_usd))

        e = time()
        print(f'Finished {fname_blend.with_suffix("").name} in {round(e-s, 2)}s')

    E = time()
    print(f'Finished all in {round(E-S, 2)}s')

if __name__ == '__main__':
    main()
