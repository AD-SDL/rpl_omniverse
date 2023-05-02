from omni.kit.scripting import BehaviorScript
from pprint import pprint
from pxr import Usd, UsdGeom, Gf

class BehaviorBasedMove(BehaviorScript):
    def on_init(self):
        print(f"{__class__.__name__}.on_init()->{self.prim_path}")

    def on_destroy(self):
        print(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

    def on_play(self):
        print(f"{__class__.__name__}.on_play()->{self.prim_path}")

    def on_pause(self):
        print(f"{__class__.__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        print(f"{__class__.__name__}.on_stop()->{self.prim_path}")

    def on_update(self, current_time: float, delta_time: float):
        print(f"{__class__.__name__}.on_update(current_time={current_time}, delta_time={delta_time})->{self.prim_path}")

        xform = UsdGeom.Xform(self.prim)

        ops = xform.GetOrderedXformOps()
        tOp = None
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                tOp = op
        if tOp == None:
            tOp = xform.AddTranslateOp()

        pos = tOp.Get()
        pos += Gf.Vec3d(0, 0, 0.01)
        tOp.Set(pos)
