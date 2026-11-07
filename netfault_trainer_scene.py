import bpy
import bmesh
import math
import random
import os
import traceback
from mathutils import Vector, Matrix, Euler

XRAY            = True
CUT_LAPTOP      = 1
SEED            = 11
JITTER          = 1.0

WEAR_STRENGTH   = 0.60
DIRT_STRENGTH   = 0.50
EDGE_WEAR_RADIUS= 0.030
LAYER_LINE_HEIGHT   = 0.012
LAYER_LINE_STRENGTH = 0.35

BEVEL           = 0.030
CABLE_RADIUS    = 0.050
CABLE_SLACK     = 1.0
LID_LIFT        = 1.45
CUT_FRONT_WALL  = True
SHOW_BADGES     = True
SHOW_WIFI       = False

LIGHT_SCALE     = 1.0

BAKE_FOR_WEB    = True
BAKE_RES        = {'large': 2048, 'medium': 1024, 'small': 512}
BAKE_SAMPLES    = 16
EXPORT_GLB      = True
EXPORT_PATH     = os.path.join(os.path.expanduser("~"), "netfault_trainer.glb")
EXPORT_IMAGE_FORMAT = 'JPEG'

ENC_W, ENC_D, ENC_H = 3.0, 1.9, 1.2
WALL, FLOOR = 0.07, 0.06
JACK_Y = (0.6, 0.0, -0.6)
JACK_Z = 0.6
LAP_X = -5.9
LAP_Y = (2.7, 0.0, -2.7)
SW_X = 5.1
SW_W, SW_D, SW_H = 2.0, 1.4, 0.46
SW_FACE = SW_X - SW_W / 2
SW_Y = (0.52, 0.26, 0.0, -0.26, -0.52)
SW_PORT_Z = 0.20
PHONE_Y = -3.7

CUT = CUT_LAPTOP if CUT_LAPTOP in (0, 1, 2) else -1

R = random.Random(SEED)
COLL, MAT, BAKE_QUEUE = {}, {}, []
LAP_PORTS, SW_PORTS = {}, {}


def jit(a):
    return R.uniform(-a, a) * JITTER


def ME(x, y, z):
    return Vector((x, -z, y))


def srgb(hexstr, a=1.0):
    h = hexstr.lstrip('#')
    r, g, b = (int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))
    f = lambda c: c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4
    return (f(r), f(g), f(b), a)


def new_coll(name, parent=None):
    c = bpy.data.collections.new(name)
    (parent or bpy.context.scene.collection).children.link(c)
    return c


def new_empty(name, coll, loc=(0, 0, 0)):
    o = bpy.data.objects.new(name, None)
    o.empty_display_type = 'PLAIN_AXES'
    o.empty_display_size = 0.2
    coll.objects.link(o)
    o.location = loc
    return o


def eval_mesh(obj):
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    eo = obj.evaluated_get(dg)
    return bpy.data.meshes.new_from_object(eo, preserve_all_data_layers=False, depsgraph=dg)


def smooth_by_angle(mesh, deg):
    bm = bmesh.new()
    bm.from_mesh(mesh)
    lim = math.radians(deg)
    for e in bm.edges:
        e.smooth = True
        if len(e.link_faces) != 2 or e.calc_face_angle(0.0) > lim:
            e.smooth = False
    for f in bm.faces:
        f.smooth = True
    bm.to_mesh(mesh)
    bm.free()
    if hasattr(mesh, 'use_auto_smooth'):
        mesh.use_auto_smooth = True
        mesh.auto_smooth_angle = math.pi


class MB:

    def __init__(self):
        self.bm = bmesh.new()
        self.mats = []
        self.M0 = Matrix.Identity(4)

    def mi(self, mat):
        if mat is None:
            return 0
        if mat not in self.mats:
            self.mats.append(mat)
        return self.mats.index(mat)

    def _face(self, vs, mat):
        try:
            f = self.bm.faces.new(vs)
            f.material_index = self.mi(mat)
        except ValueError:
            pass

    def _m(self, loc, rot):
        return self.M0 @ Matrix.Translation(loc) @ Euler(rot, 'XYZ').to_matrix().to_4x4()

    def box(self, size, loc, rot=(0, 0, 0), mat=None):
        sx, sy, sz = (s / 2.0 for s in size)
        M = self._m(loc, rot)
        pts = [(-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz),
               (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz)]
        v = [self.bm.verts.new(M @ Vector(p)) for p in pts]
        for idx in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
            self._face([v[i] for i in idx], mat)

    def cyl(self, r, h, loc, rot=(0, 0, 0), seg=12, mat=None, r_top=None, cap_bottom=True):
        r2 = r if r_top is None else r_top
        M = self._m(loc, rot)

        def ring(rad, z):
            return [self.bm.verts.new(M @ Vector((rad * math.cos(2 * math.pi * i / seg),
                                                  rad * math.sin(2 * math.pi * i / seg), z)))
                    for i in range(seg)]
        b, t = ring(r, -h / 2.0), ring(r2, h / 2.0)
        for i in range(seg):
            j = (i + 1) % seg
            self._face([b[i], b[j], t[j], t[i]], mat)
        self._face(t, mat)
        if cap_bottom:
            self._face(b[::-1], mat)

    def blob(self, loc, r, squash=0.6, mat=None):
        t = (1 + 5 ** 0.5) / 2
        P = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t),
             (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
        F = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4),
             (11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8),
             (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
        M = self._m(loc, (0, 0, R.uniform(0, 6.28)))
        k = r / 1.902
        vs = [self.bm.verts.new(M @ Vector((p[0] * k * (1 + jit(0.18)), p[1] * k * (1 + jit(0.18)),
                                            p[2] * k * squash * (1 + jit(0.18))))) for p in P]
        for f in F:
            self._face([vs[i] for i in f], mat)

    def arc(self, r, w, a0, a1, segs=12, mat=None):
        M = self._m((0, 0, 0), (0, 0, 0))
        ro, ri = r + w / 2, r - w / 2
        outer, inner = [], []
        for i in range(segs + 1):
            a = a0 + (a1 - a0) * i / segs
            outer.append(self.bm.verts.new(M @ Vector((ro * math.cos(a), ro * math.sin(a), 0))))
            inner.append(self.bm.verts.new(M @ Vector((ri * math.cos(a), ri * math.sin(a), 0))))
        for i in range(segs):
            self._face([inner[i], outer[i], outer[i + 1], inner[i + 1]], mat)

    def add_mesh(self, me, matrix=None, mat=None):
        M = self.M0 @ (matrix if matrix is not None else Matrix.Identity(4))
        vm = [self.bm.verts.new(M @ v.co) for v in me.vertices]
        for p in me.polygons:
            try:
                f = self.bm.faces.new([vm[i] for i in p.vertices])
                f.material_index = self.mi(mat) if mat is not None else p.material_index
            except ValueError:
                pass

    def text(self, txt, size, loc, rot=(0, 0, 0), mat=None, align='CENTER'):
        me = text_mesh(txt, size, align)
        local = Matrix.Translation(loc) @ Euler(rot, 'XYZ').to_matrix().to_4x4()
        self.add_mesh(me, local, mat)
        bpy.data.meshes.remove(me)

    def finish(self, name, coll, bevel=0.0, bev_seg=2, smooth=None, parent=None):
        bm = self.bm
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
        me = bpy.data.meshes.new(name)
        bm.to_mesh(me)
        bm.free()
        for m in self.mats:
            me.materials.append(m)
        obj = bpy.data.objects.new(name, me)
        coll.objects.link(obj)
        if bevel > 0:
            mod = obj.modifiers.new('Bevel', 'BEVEL')
            mod.width = bevel
            mod.segments = bev_seg
            mod.limit_method = 'ANGLE'
            mod.angle_limit = math.radians(35)
            new = eval_mesh(obj)
            old = obj.data
            obj.modifiers.clear()
            obj.data = new
            new.name = name
            if len(new.materials) == 0:
                for m in self.mats:
                    new.materials.append(m)
            bpy.data.meshes.remove(old)
        if smooth:
            smooth_by_angle(obj.data, smooth)
        if parent is not None:
            obj.parent = parent
        return obj


def text_mesh(txt, size=0.1, align='CENTER'):
    cu = bpy.data.curves.new('tmp_txt', 'FONT')
    cu.body = txt
    cu.size = size
    cu.align_x = align
    cu.align_y = 'CENTER'
    cu.resolution_u = 2
    try:
        cu.fill_mode = 'FRONT'
    except Exception:
        pass
    obj = bpy.data.objects.new('tmp_txt_obj', cu)
    COLL['tmp'].objects.link(obj)
    me = eval_mesh(obj)
    bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.curves.remove(cu)
    return me


def curve_mesh(points, radius, res=6, bevel_res=1):
    cu = bpy.data.curves.new('tmp_cu', 'CURVE')
    cu.dimensions = '3D'
    cu.bevel_depth = radius
    cu.bevel_resolution = bevel_res
    cu.resolution_u = res
    try:
        cu.use_fill_caps = True
    except Exception:
        pass
    sp = cu.splines.new('BEZIER')
    sp.bezier_points.add(len(points) - 1)
    for bp, p in zip(sp.bezier_points, points):
        bp.co = p
        bp.handle_left_type = 'AUTO'
        bp.handle_right_type = 'AUTO'
    obj = bpy.data.objects.new('tmp_cu_obj', cu)
    COLL['tmp'].objects.link(obj)
    me = eval_mesh(obj)
    bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.curves.remove(cu)
    return me


def screen_plane(name, coll, size, center, right, up, mat, parent=None):
    w, h = size
    c = Vector(center)
    r = Vector(right).normalized() * (w / 2.0)
    u = Vector(up).normalized() * (h / 2.0)
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(p) for p in (c - r - u, c + r - u, c + r + u, c - r + u)], [], [(0, 1, 2, 3)])
    me.update()
    me.materials.append(mat)
    obj = bpy.data.objects.new(name, me)
    coll.objects.link(obj)
    if parent is not None:
        obj.parent = parent
    return obj


def cutter_box(coll, size, loc, rot=(0, 0, 0)):
    mb = MB()
    mb.box(size, loc, rot)
    return mb.finish('cut', coll)


def cutter_cyl(coll, r, h, loc, rot=(0, 0, 0), seg=16):
    mb = MB()
    mb.cyl(r, h, loc, rot, seg)
    return mb.finish('cut', coll)


def carve(obj, bevel=0.0, bev_seg=2, cut_coll=None):
    if bevel > 0:
        m = obj.modifiers.new('Bevel', 'BEVEL')
        m.width = bevel
        m.segments = bev_seg
        m.limit_method = 'ANGLE'
        m.angle_limit = math.radians(35)
    if cut_coll is not None:
        b = obj.modifiers.new('Cut', 'BOOLEAN')
        b.operation = 'DIFFERENCE'
        b.operand_type = 'COLLECTION'
        b.collection = cut_coll
        try:
            b.solver = 'EXACT'
        except Exception:
            pass
    return eval_mesh(obj)


def purge(*items):
    for it in items:
        if isinstance(it, bpy.types.Collection):
            for o in list(it.objects):
                me = o.data
                bpy.data.objects.remove(o, do_unlink=True)
                if me is not None and me.users == 0:
                    bpy.data.meshes.remove(me)
            bpy.data.collections.remove(it)
        else:
            me = it.data
            bpy.data.objects.remove(it, do_unlink=True)
            if me is not None and me.users == 0:
                bpy.data.meshes.remove(me)


def setup_scene():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)
    for blk in (bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.images,
                bpy.data.lights, bpy.data.cameras):
        for b in list(blk):
            try:
                blk.remove(b)
            except Exception:
                pass
    COLL.clear(); MAT.clear(); BAKE_QUEUE.clear(); LAP_PORTS.clear(); SW_PORTS.clear()
    root = new_coll('NetFault_Scene')
    COLL['root'] = root
    for key, nm in (('enc', 'Enclosure'), ('int', 'Internals'), ('cab', 'Cables'), ('lap', 'Laptops'),
                    ('sw', 'Switch'), ('phone', 'Phone'), ('env', 'Environment'), ('ann', 'Annotations')):
        COLL[key] = new_coll(nm, root)
    COLL['tmp'] = new_coll('TMP_helpers')
    scn = bpy.context.scene
    scn.render.engine = 'CYCLES'
    scn.render.resolution_x, scn.render.resolution_y = 1920, 1080
    try:
        scn.cycles.samples = 96
        scn.cycles.use_denoising = True
    except Exception:
        pass
    for vt in ('AgX', 'Filmic'):
        try:
            scn.view_settings.view_transform = vt
            break
        except Exception:
            continue


def set_in(node, names, value):
    if isinstance(names, str):
        names = (names,)
    for n in names:
        if n in node.inputs:
            node.inputs[n].default_value = value
            return True
    return False


def mk_mat(name, hexc, rough=0.5, metal=0.0, emit=None, emit_str=0.0, spec=0.3):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes['Principled BSDF']
    col = srgb(hexc)
    set_in(b, 'Base Color', col)
    set_in(b, 'Roughness', rough)
    set_in(b, 'Metallic', metal)
    set_in(b, ('Specular IOR Level', 'Specular'), spec)
    if emit:
        set_in(b, ('Emission Color', 'Emission'), srgb(emit))
        set_in(b, 'Emission Strength', emit_str)
    m.diffuse_color = col
    return m


def mk_wear_mat(name, base, wear_col, base_alt=None, rough=0.55, rough_wear=0.80, wear=1.0, dirt=1.0,
                dirt_col='#2a2823', noise_scale=7.0, grain=False, layers=False, spec=0.3):
    W = max(0.0, min(1.0, wear * WEAR_STRENGTH))
    D = max(0.0, min(1.0, dirt * DIRT_STRENGTH))
    K = min(1.0, W * 1.6)
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    N, L = nt.nodes, nt.links
    N.clear()

    def node(t, x, y):
        n = N.new(t)
        n.location = (x, y)
        return n

    def math_n(op, a=None, b=None, c=None, x=0, y=0, clamp=False):
        n = node('ShaderNodeMath', x, y)
        n.operation = op
        n.use_clamp = clamp
        for i, v in enumerate((a, b, c)):
            if v is None:
                continue
            if isinstance(v, (int, float)):
                n.inputs[i].default_value = v
            else:
                L.new(v, n.inputs[i])
        return n

    def ramp(fac, p0, p1, x, y):
        r = node('ShaderNodeValToRGB', x, y)
        r.color_ramp.elements[0].position = max(0.0, min(0.98, p0))
        r.color_ramp.elements[1].position = max(0.02, min(1.0, p1))
        L.new(fac, r.inputs['Fac'])
        return r

    out = node('ShaderNodeOutputMaterial', 1800, 0)
    bsdf = node('ShaderNodeBsdfPrincipled', 1500, 0)
    L.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    set_in(bsdf, 'Metallic', 0.0)
    set_in(bsdf, ('Specular IOR Level', 'Specular'), spec)

    tc = node('ShaderNodeTexCoord', -1400, 0)
    n1 = node('ShaderNodeTexNoise', -1100, 250)
    n1.inputs['Scale'].default_value = noise_scale * 3
    n1.inputs['Detail'].default_value = 6
    L.new(tc.outputs['Object'], n1.inputs['Vector'])
    n2 = node('ShaderNodeTexNoise', -1100, 0)
    n2.inputs['Scale'].default_value = noise_scale * 9
    n2.inputs['Detail'].default_value = 4
    L.new(tc.outputs['Object'], n2.inputs['Vector'])
    n3 = node('ShaderNodeTexNoise', -1100, -250)
    n3.inputs['Scale'].default_value = 1.3
    n3.inputs['Detail'].default_value = 3
    L.new(tc.outputs['Object'], n3.inputs['Vector'])

    bev = node('ShaderNodeBevel', -1100, 500)
    bev.samples = 6
    bev.inputs['Radius'].default_value = EDGE_WEAR_RADIUS
    geo = node('ShaderNodeNewGeometry', -1100, 650)
    dot = node('ShaderNodeVectorMath', -850, 560)
    dot.operation = 'DOT_PRODUCT'
    L.new(bev.outputs['Normal'], dot.inputs[0])
    L.new(geo.outputs['True Normal'], dot.inputs[1])
    inv = math_n('SUBTRACT', 1.0, dot.outputs['Value'], x=-650, y=560)
    edge = ramp(inv.outputs['Value'], 0.012, 0.11, -450, 560)
    e0 = 0.62 - 0.30 * W
    brk = ramp(n1.outputs['Fac'], e0, e0 + 0.10, -850, 250)
    edge_brk = math_n('MULTIPLY', edge.outputs['Color'], brk.outputs['Color'], x=-250, y=400)
    s0 = 0.72 - 0.10 * W
    scuff = ramp(n2.outputs['Fac'], s0, s0 + 0.04, -850, 0)
    scuff_s = math_n('MULTIPLY', scuff.outputs['Color'], 0.45, x=-650, y=0)
    comb = math_n('MAXIMUM', edge_brk.outputs['Value'], scuff_s.outputs['Value'], x=-50, y=300)
    wear_mask = math_n('MULTIPLY', comb.outputs['Value'], K, x=150, y=300, clamp=True)

    ao = node('ShaderNodeAmbientOcclusion', -1100, -550)
    ao.samples = 8
    ao.inputs['Distance'].default_value = 0.25
    cav = math_n('SUBTRACT', 1.0, ao.outputs['AO'], x=-850, y=-550)
    cav_s = math_n('MULTIPLY', cav.outputs['Value'], D * 2.2, x=-650, y=-550, clamp=True)
    smudge = ramp(n3.outputs['Fac'], 0.45, 0.75, -850, -250)
    smudge_s = math_n('MULTIPLY', smudge.outputs['Color'], D * 0.35, x=-650, y=-250)
    dirt_amt = math_n('ADD', cav_s.outputs['Value'], smudge_s.outputs['Value'], x=-400, y=-450, clamp=True)

    if base_alt is not None:
        if grain:
            mp = node('ShaderNodeMapping', -1200, -750)
            mp.inputs['Scale'].default_value = (0.35, 16.0, 0.35)
            L.new(tc.outputs['Object'], mp.inputs['Vector'])
            gn = node('ShaderNodeTexNoise', -1000, -750)
            gn.inputs['Scale'].default_value = 1.0
            gn.inputs['Detail'].default_value = 5
            L.new(mp.outputs['Vector'], gn.inputs['Vector'])
            var = ramp(gn.outputs['Fac'], 0.35, 0.65, -750, -750)
        else:
            var = ramp(n3.outputs['Fac'], 0.35, 0.65, -750, -750)
        bc = node('ShaderNodeMixRGB', -450, -750)
        bc.inputs['Color1'].default_value = srgb(base)
        bc.inputs['Color2'].default_value = srgb(base_alt)
        L.new(var.outputs['Color'], bc.inputs['Fac'])
        base_out = bc.outputs['Color']
    else:
        rgb = node('ShaderNodeRGB', -450, -750)
        rgb.outputs[0].default_value = srgb(base)
        base_out = rgb.outputs[0]
    m1 = node('ShaderNodeMixRGB', 400, 100)
    m1.inputs['Color2'].default_value = srgb(wear_col)
    L.new(wear_mask.outputs['Value'], m1.inputs['Fac'])
    L.new(base_out, m1.inputs['Color1'])
    m2 = node('ShaderNodeMixRGB', 700, 0)
    m2.inputs['Color2'].default_value = srgb(dirt_col)
    L.new(dirt_amt.outputs['Value'], m2.inputs['Fac'])
    L.new(m1.outputs['Color'], m2.inputs['Color1'])
    L.new(m2.outputs['Color'], bsdf.inputs['Base Color'])

    r1 = math_n('MULTIPLY_ADD', wear_mask.outputs['Value'], rough_wear - rough, rough, x=400, y=-300)
    r2 = math_n('MULTIPLY_ADD', smudge.outputs['Color'], 0.12, r1.outputs['Value'], x=700, y=-300, clamp=True)
    L.new(r2.outputs['Value'], bsdf.inputs['Roughness'])

    if layers and LAYER_LINE_STRENGTH > 0:
        wv = node('ShaderNodeTexWave', 400, -600)
        wv.wave_type = 'BANDS'
        wv.bands_direction = 'Z'
        wv.wave_profile = 'SAW'
        wv.inputs['Scale'].default_value = 1.0 / LAYER_LINE_HEIGHT
        wv.inputs['Distortion'].default_value = 1.2
        wv.inputs['Detail'].default_value = 2
        L.new(tc.outputs['Object'], wv.inputs['Vector'])
        bp = node('ShaderNodeBump', 900, -600)
        bp.inputs['Strength'].default_value = LAYER_LINE_STRENGTH
        bp.inputs['Distance'].default_value = 0.004
        L.new(wv.outputs['Fac'], bp.inputs['Height'])
        L.new(bp.outputs['Normal'], bsdf.inputs['Normal'])
    m.diffuse_color = srgb(base)
    return m


def build_materials():
    P = lambda n, h, r=0.5, mt=0.0, e=None, es=0.0: MAT.__setitem__(n, mk_mat(n, h, r, mt, e, es))
    P('pcb_green', '#14532d', 0.40)
    P('pcb_black', '#12151c', 0.40)
    P('pcb_blue', '#1a4aa0', 0.40)
    P('relay_blue', '#2a6fbf', 0.45)
    P('relay_on', '#EF9F27', 0.45)
    P('terminal', '#1e7a3c', 0.50)
    P('chip_black', '#1a1a1a', 0.60)
    P('header_black', '#101010', 0.55)
    P('brass', '#b08d3c', 0.35, 1.0)
    P('gold', '#d4af37', 0.30, 1.0)
    P('silver', '#c9c9c4', 0.35, 0.9)
    P('psu_case', '#2b2d31', 0.55)
    P('hotglue', '#e8d9a8', 0.25)
    P('jack_dark', '#171717', 0.55)
    P('cable_blue', '#378ADD', 0.45)
    P('cable_red', '#E24B4A', 0.45)
    P('wire_orange', '#EF9F27', 0.40)
    P('wire_black', '#1b1b1b', 0.45)
    P('plug_clear', '#c9d3d8', 0.20)
    P('led_g_on', '#2f5a14', 0.3, 0.0, '#97C459', 6.0)
    P('led_g_off', '#2f3d1c', 0.3)
    P('led_r_on', '#E24B4A', 0.3, 0.0, '#E24B4A', 6.0)
    P('led_r_off', '#4a2020', 0.3)
    P('oled_bg', '#06121f', 0.3, 0.0, '#0a2236', 0.6)
    P('oled_text', '#7fd0ff', 0.3, 0.0, '#7fd0ff', 3.0)
    P('scr_bg', '#0b1b2b', 0.3, 0.0, '#0b1b2b', 0.9)
    P('scr_green', '#97C459', 0.3, 0.0, '#97C459', 2.0)
    P('scr_pink', '#F09595', 0.3, 0.0, '#F09595', 2.0)
    P('bezel_black', '#0c0c0e', 0.35)
    P('ph_bg', '#E6F1FB', 0.3, 0.0, '#E6F1FB', 0.8)
    P('ph_title', '#0C447C', 0.3, 0.0, '#0C447C', 0.5)
    P('ph_btn', '#85B7EB', 0.3, 0.0, '#85B7EB', 0.8)
    P('ph_btn_text', '#042C53', 0.3, 0.0, '#042C53', 0.4)
    P('ph_red', '#E24B4A', 0.3, 0.0, '#E24B4A', 0.8)
    P('ph_green', '#0F6E56', 0.3, 0.0, '#0F6E56', 0.8)
    P('ph_white', '#ffffff', 0.3, 0.0, '#ffffff', 0.8)
    P('paper', '#e9e6dc', 0.85)
    P('tape', '#d6c39a', 0.80)
    P('ink', '#1b1b1b', 0.60)
    P('marker', '#1c2f7a', 0.50)
    P('sticker_y', '#e9bf3b', 0.45)
    P('sticker_w', '#e8e8e4', 0.45)
    P('stand_pla', '#d8d4c8', 0.60)
    P('key_well', '#141414', 0.60)
    P('key', '#222221', 0.55)
    P('trackpad', '#303030', 0.40)
    P('black_ox', '#242424', 0.45, 0.8)
    P('zinc', '#b5b5b0', 0.35, 0.9)
    P('badge_black', '#111111', 0.5)
    P('credit', '#f1ead6', 0.5)
    P('badge_white', '#f4f4f4', 0.5, 0.0, '#ffffff', 0.4)
    P('wifi', '#EF9F27', 0.4, 0.0, '#EF9F27', 3.0)
    MAT['enc_body'] = mk_wear_mat('enc_body', '#0F6E56', '#9BC2B3', '#13795e', rough=0.62, rough_wear=0.82,
                                  wear=0.9, dirt=1.0, layers=True)
    MAT['enc_lid'] = mk_wear_mat('enc_lid', '#1D9E75', '#80857f', '#168a65', rough=0.52, rough_wear=0.75,
                                 wear=1.1, dirt=0.8)
    MAT['sw_plastic'] = mk_wear_mat('sw_plastic', '#3f3f3c', '#6d6d69', '#464643', rough=0.50, wear=0.8)
    MAT['lap1'] = mk_wear_mat('lap1', '#4a4a47', '#7a7a76', '#53534f', rough=0.45, wear=1.0)
    MAT['lap2'] = mk_wear_mat('lap2', '#8d8f90', '#b9b9b3', '#a09c8c', rough=0.40, wear=0.9, dirt=1.3)
    MAT['lap3'] = mk_wear_mat('lap3', '#262626', '#555553', '#2c2c2b', rough=0.50, wear=1.2)
    MAT['phone_case'] = mk_wear_mat('phone_case', '#2c2c2a', '#5d5d59', None, rough=0.55, wear=1.0)
    MAT['desk'] = mk_wear_mat('desk', '#a68a64', '#c1a77f', '#8f7455', rough=0.65, wear=0.5, dirt=1.4,
                              noise_scale=2.0, grain=True)


def build_environment():
    mb = MB()
    mb.box((15.8, 9.8, 0.4), (-0.4, -0.4, -0.2), mat=MAT['desk'])
    desk = mb.finish('Desk', COLL['env'], bevel=0.03, bev_seg=1)
    BAKE_QUEUE.append((desk, 'large', False))


def build_enclosure():
    c = COLL['enc']
    base = MB()
    base.box((ENC_W, ENC_D, ENC_H), (0, 0, ENC_H / 2))
    tmp = base.finish('enc_tmp', COLL['tmp'])
    cuts = new_coll('cuts_enc', COLL['tmp'])
    cutter_box(cuts, (ENC_W - 2 * WALL, ENC_D - 2 * WALL, ENC_H), (0, 0, FLOOR + ENC_H / 2 + 0.001))
    for side in (-1, 1):
        for i in range(3):
            cutter_box(cuts, (0.40, 0.27 + jit(0.012), 0.255 + jit(0.012)),
                       (side * 1.47, JACK_Y[i] + jit(0.014), JACK_Z + jit(0.010)),
                       (jit(math.radians(1.3)), 0, 0))
    cutter_cyl(cuts, 0.062, 0.30, (0.30, 0.92, 0.20), (math.pi / 2, 0, 0), seg=14)
    for i in range(6):
        cutter_box(cuts, (0.05 + jit(0.008), 0.30, 0.30 + jit(0.04)),
                   (-1.2 + i * 0.13 + jit(0.014), 0.92, 0.60 + jit(0.02)),
                   (0, jit(math.radians(1.5)), 0))
    if XRAY and CUT_FRONT_WALL:
        cutter_box(cuts, (2.90, 0.60, 1.20), (0, -1.0, 0.34 + 0.60), (0, math.radians(0.6), 0))
    me = carve(tmp, BEVEL * 1.2, 2, cuts)
    purge(tmp, cuts)

    mb = MB()
    mb.add_mesh(me, None, MAT['enc_body'])
    bpy.data.meshes.remove(me)
    ph = ENC_H - FLOOR - 0.03
    for sx in (-1, 1):
        for sy in (-1, 1):
            mb.cyl(0.085, ph, (sx * 1.33, sy * 0.78, FLOOR + ph / 2), seg=10, mat=MAT['enc_body'])
    shell = mb.finish('Enclosure_Shell', c)
    BAKE_QUEUE.append((shell, 'large', True))
    if XRAY and CUT_FRONT_WALL:
        fw = MB()
        fw.box((2.92, WALL, 0.88), (0, -(ENC_D / 2 - WALL / 2), 0.76), mat=MAT['enc_body'])
        front = fw.finish('Enclosure_FrontWall', c, bevel=0.012, bev_seg=1)
        BAKE_QUEUE.append((front, 'medium', False))


def build_jacks():
    c = COLL['enc']
    hous = MB()
    for side in (-1, 1):
        for i in range(3):
            hous.box((0.31, 0.26, 0.24), (side * 1.555, JACK_Y[i], JACK_Z))
    tmp = hous.finish('jack_tmp', COLL['tmp'])
    cuts = new_coll('cuts_jack', COLL['tmp'])
    for side in (-1, 1):
        for i in range(3):
            cutter_box(cuts, (0.20, 0.17, 0.14), (side * 1.64, JACK_Y[i], JACK_Z))
            cutter_box(cuts, (0.20, 0.06, 0.05), (side * 1.64, JACK_Y[i], JACK_Z + 0.09))
    me = carve(tmp, 0.008, 1, cuts)
    purge(tmp, cuts)
    mb = MB()
    mb.add_mesh(me, None, MAT['jack_dark'])
    bpy.data.meshes.remove(me)
    for side in (-1, 1):
        for i in range(3):
            for k in range(8):
                mb.box((0.10, 0.008, 0.004), (side * 1.62, JACK_Y[i] - 0.07 + k * 0.02, JACK_Z + 0.062), mat=MAT['gold'])
    mb.finish('RJ45_Jacks', c)


def build_panel():
    c = COLL['enc']
    root = new_empty('Lid_Assembly', c)
    top = ENC_H + 0.08
    ox, oy = -0.7, -0.45

    slab = MB()
    slab.box((3.04, 1.94, 0.08), (0, 0, ENC_H + 0.04))
    tmp = slab.finish('lid_tmp', COLL['tmp'])
    cuts = new_coll('cuts_lid', COLL['tmp'])
    cutter_box(cuts, (0.86, 0.46, 0.30), (ox + jit(0.01), oy + jit(0.01), ENC_H + 0.04),
               (0, 0, math.radians(0.7) + jit(math.radians(0.4))))
    cutter_box(cuts, (0.08, 0.07, 0.30), (ox - 0.43, oy + 0.22, ENC_H + 0.04), (0, 0, 0.3))
    leds = []
    for i, x in enumerate((-0.9, 0.0, 0.9)):
        for kind, lx in (('g', x - 0.13), ('r', x + 0.13)):
            hx, hy = lx + jit(0.012), 0.6 + jit(0.012)
            cutter_cyl(cuts, 0.063 + jit(0.004), 0.30, (hx, hy, ENC_H + 0.04), seg=14)
            leds.append((kind, i, hx, hy))
    me = carve(tmp, 0.018, 2, cuts)
    purge(tmp, cuts)
    mb = MB()
    mb.add_mesh(me, None, MAT['enc_lid'])
    bpy.data.meshes.remove(me)
    lid = mb.finish('Enclosure_Lid', c, parent=root)
    BAKE_QUEUE.append((lid, 'large', False))

    mb = MB()
    mb.box((0.98, 0.64, 0.03), (ox, oy, ENC_H - 0.035), mat=MAT['pcb_blue'])
    mb.box((0.84, 0.44, 0.02), (ox, oy, ENC_H - 0.012), mat=MAT['chip_black'])
    mb.box((0.80, 0.38, 0.004), (ox, oy, ENC_H + 0.002), mat=MAT['oled_bg'])
    for sx, sy in ((-0.47, -0.29), (0.45, 0.30)):
        mb.blob((ox + sx, oy + sy, ENC_H - 0.065), 0.05, 0.6, MAT['hotglue'])
    mb.finish('OLED_Module', c, parent=root)
    screen_plane('OLED_Screen', c, (0.80, 0.38), (ox, oy, ENC_H + 0.006), (1, 0, 0), (0, 1, 0),
                 mk_mat('Mat_OLED_Screen', '#06121f', 0.3, 0.0, '#0a2236', 0.6), root)

    led_look = {('g', True): ('#2f5a14', '#97C459'), ('g', False): ('#2f3d1c', None),
                ('r', True): ('#E24B4A', '#E24B4A'), ('r', False): ('#4a2020', None)}
    for kind, i, hx, hy in leds:
        on = (i != CUT) if kind == 'g' else (i == CUT)
        base_hex, emit_hex = led_look[(kind, on)]
        led_name = 'LED_L%d_%s' % (i + 1, 'green' if kind == 'g' else 'red')
        mat = mk_mat('Mat_' + led_name, base_hex, 0.3, 0.0, emit_hex, 6.0 if emit_hex else 0.0)
        mb = MB()
        x, y = hx + jit(0.01), hy + jit(0.01)
        rot = (jit(0.04), jit(0.04), 0)
        mb.cyl(0.055, 0.09, (x, y, ENC_H + 0.05), rot, seg=12, mat=mat)
        mb.cyl(0.055, 0.03, (x, y, ENC_H + 0.11), rot, seg=12, mat=mat, r_top=0.035)
        for dx in (-0.02, 0.02):
            mb.box((0.008, 0.008, 0.12), (x + dx, y, ENC_H - 0.045), mat=MAT['gold'])
        mb.finish(led_name, c, smooth=50, parent=root)

    mb = MB()
    corners = [(-1, -1), (1, -1), (-1, 1), (1, 1)]
    silver = R.randrange(4)
    for n, (sx, sy) in enumerate(corners):
        x, y = sx * 1.38 + jit(0.015), sy * 0.82 + jit(0.015)
        mat = MAT['zinc'] if n == silver else MAT['black_ox']
        mb.cyl(0.06, 0.035, (x, y, top + 0.0175), seg=10, mat=mat, r_top=0.052)
        mb.box((0.085, 0.014, 0.004), (x, y, top + 0.035), (0, 0, R.uniform(0, math.pi)), MAT['chip_black'])
    mb.finish('Lid_Screws', c, smooth=50, parent=root)

    mb = MB()
    mb.M0 = (Matrix.Translation((0.78 + jit(0.03), -0.52 + jit(0.03), top))
             @ Euler((jit(0.006), jit(0.006), math.radians(-2.6)), 'XYZ').to_matrix().to_4x4())
    mb.box((1.0, 0.5, 0.004), (0, 0, 0.002), mat=MAT['paper'])
    mb.text('NETFAULT', 0.15, (0, 0.10, 0.0045), mat=MAT['ink'])
    mb.text('TRAINER  v0.3', 0.075, (0, -0.07, 0.0045), mat=MAT['ink'])
    mb.text('do not open', 0.055, (0.08, -0.17, 0.0045), (0, 0, math.radians(2)), MAT['marker'])
    mb.finish('Lid_Label', c, parent=root)

    mb = MB()
    for side in (-1, 1):
        for i in range(3):
            mb.M0 = (Matrix.Translation((side * (1.30 + jit(0.03)), JACK_Y[i] + jit(0.03), top))
                     @ Matrix.Rotation(jit(0.07), 4, 'Z'))
            mb.box((0.30, 0.15, 0.003), (0, 0, 0.0015), mat=MAT['tape'])
            mb.text(('PC%d' if side < 0 else 'SW%d') % (i + 1), 0.075, (0, 0, 0.0035), mat=MAT['marker'])
    mb.finish('Lid_TapeLabels', c, parent=root)

    if XRAY:
        root.location = (0.10, 0.22, LID_LIFT)
        root.rotation_euler = (math.radians(1.2), math.radians(-2.0), math.radians(2.5))


def standoffs(mb, pts, z, mat):
    for (sx, sy) in pts:
        mb.cyl(0.03, 0.025, (sx, sy, z), seg=8, mat=mat)


def build_internals():
    if not XRAY:
        return
    c = COLL['int']
    intr = new_empty('Internals_Group', c)
    for k in range(3):
        mb = MB()
        mb.M0 = (Matrix.Translation((-0.7 + jit(0.012), JACK_Y[k] + jit(0.015), 0.10))
                 @ Matrix.Rotation(jit(math.radians(1.3)), 4, 'Z'))
        rel = mk_mat('Mat_Relay_%d' % (k + 1), '#EF9F27' if k == CUT else '#2a6fbf', 0.45)
        mb.box((1.0, 0.4, 0.03), (0, 0, 0), mat=MAT['pcb_green'])
        for r in range(8):
            x = -0.42 + r * 0.115 + jit(0.004)
            mb.box((0.10, 0.15, 0.13), (x, jit(0.004), 0.08 + jit(0.003)), (0, 0, jit(math.radians(0.8))), rel)
            mb.box((0.095, 0.10, 0.11), (x, 0.14, 0.07), mat=MAT['terminal'])
            mb.box((0.075, 0.045, 0.03), (x, -0.10, 0.03), mat=MAT['chip_black'])
        mb.box((0.95, 0.025, 0.06), (0, -0.165, 0.045), mat=MAT['header_black'])
        for sx, sy in ((-0.46, -0.17), (0.46, -0.17), (-0.46, 0.17), (0.46, 0.17)):
            if k == 0 and sx < 0 and sy > 0:
                mb.blob((sx, sy, -0.028), 0.045, 0.55, MAT['hotglue'])
            else:
                mb.cyl(0.03, 0.025, (sx, sy, -0.0275), seg=8, mat=MAT['brass'])
        mb.finish('RelayModule_%d' % (k + 1), c, bevel=0.004, bev_seg=1, parent=intr)

    mb = MB()
    mb.M0 = (Matrix.Translation(ME(0.8, 0.14, -0.55) + Vector((jit(0.01), jit(0.01), 0)))
             @ Matrix.Rotation(jit(math.radians(2.0)), 4, 'Z'))
    mb.box((0.80, 0.38, 0.03), (0, 0, 0), mat=MAT['pcb_black'])
    mb.box((0.30, 0.26, 0.04), (-0.12, 0, 0.035), mat=MAT['silver'])
    mb.box((0.08, 0.12, 0.045), (0.40, 0, 0.035), mat=MAT['silver'])
    for sy in (-0.16, 0.16):
        mb.box((0.70, 0.03, 0.06), (0, sy, 0.045), mat=MAT['header_black'])
    mb.box((0.10, 0.20, 0.004), (-0.34, 0, 0.017), mat=MAT['gold'])
    for sy in (-0.1, 0.1):
        mb.box((0.035, 0.03, 0.025), (0.30, sy, 0.028), mat=MAT['chip_black'])
    standoffs(mb, ((-0.36, -0.15), (0.36, -0.15), (-0.36, 0.15), (0.36, 0.15)), -0.0275, MAT['brass'])
    mb.finish('ESP32', c, bevel=0.004, bev_seg=1, parent=intr)

    for j, x in enumerate((0.55, 1.05)):
        mb = MB()
        mb.M0 = (Matrix.Translation(ME(x, 0.09, 0.1) + Vector((jit(0.012), jit(0.012), 0)))
                 @ Matrix.Rotation(jit(math.radians(2.5)), 4, 'Z'))
        mb.box((0.42, 0.36, 0.03), (0, 0, 0), mat=MAT['pcb_green'])
        mb.box((0.28, 0.10, 0.04), (0, 0, 0.035), mat=MAT['chip_black'])
        for sy in (-0.145, 0.145):
            mb.box((0.38, 0.025, 0.06), (0, sy, 0.045), mat=MAT['header_black'])
        standoffs(mb, ((-0.18, -0.13), (0.18, -0.13), (-0.18, 0.13), (0.18, 0.13)), -0.0275, MAT['brass'])
        mb.finish('IOExpander_%d' % (j + 1), c, bevel=0.004, bev_seg=1, parent=intr)

    mb = MB()
    mb.box((0.32, 0.30, 0.14), ME(0.05, 0.14, -0.62), (0, 0, jit(math.radians(3))), MAT['psu_case'])
    pc = ME(0.05, 0.14, -0.62)
    mb.box((0.24, 0.12, 0.003), (pc.x, pc.y, pc.z + 0.0715), (0, 0, jit(math.radians(4))), MAT['paper'])
    mb.text('5V 3A', 0.08, (pc.x, pc.y, pc.z + 0.074), mat=MAT['ink'])
    for dx in (-0.08, 0.08):
        mb.box((0.10, 0.08, 0.07), (pc.x + dx, pc.y - 0.19, pc.z - 0.035), mat=MAT['terminal'])
    mb.box((0.20, 0.14, 0.16), (0.30, 0.88, 0.20), mat=MAT['jack_dark'])
    mb.cyl(0.055, 0.12, (0.30, 0.97, 0.20), (math.pi / 2, 0, 0), seg=12, mat=MAT['jack_dark'])
    for p in ((0.19, 0.84, 0.12), (0.41, 0.85, 0.13), (0.14, 0.72, 0.06)):
        mb.blob(p, 0.05, 0.5, MAT['hotglue'])
    mb.finish('PowerSupply_5V3A', c, bevel=0.006, bev_seg=1, parent=intr)

    def pts(mock):
        return [ME(*p) + Vector((jit(0.015), jit(0.015), jit(0.01))) for p in mock]

    mb = MB()

    def wire(mock, rad, mat):
        me = curve_mesh(pts(mock), rad, 6, 1)
        mb.add_mesh(me, None, mat)
        bpy.data.meshes.remove(me)

    jz = (-0.6, 0.0, 0.6)
    O, B = MAT['wire_orange'], MAT['wire_black']
    wire([(0.7, 0.17, -0.4), (0.62, 0.15, -0.15), (0.55, 0.17, 0.04)], 0.018, O)
    wire([(0.9, 0.17, -0.4), (0.95, 0.15, -0.15), (1.05, 0.17, 0.04)], 0.018, O)
    wire([(0.45, 0.16, 0.1), (0.1, 0.18, (0.1 + jz[0]) / 2), (-0.2, 0.2, jz[0])], 0.018, O)
    wire([(0.45, 0.16, 0.1), (0.1, 0.18, 0.05), (-0.2, 0.2, jz[1])], 0.018, O)
    wire([(1.05, 0.17, 0.12), (0.5, 0.26, 0.4), (0.1, 0.2, 0.55), (-0.2, 0.2, jz[2])], 0.018, O)
    wire([(0.3, 0.2, -0.84), (0.22, 0.19, -0.76), (0.15, 0.18, -0.70)], 0.022, B)
    wire([(0.2, 0.2, -0.58), (0.32, 0.2, -0.56), (0.42, 0.19, -0.55)], 0.022, B)
    wire([(-0.1, 0.19, -0.62), (-0.19, 0.15, -0.6), (-0.23, 0.14, -0.3), (-0.24, 0.15, 0.0),
          (-0.23, 0.14, 0.3), (-0.24, 0.15, 0.6)], 0.022, B)
    for k in range(3):
        col = '#E24B4A' if k == CUT else '#378ADD'
        for tag, mock in (('in', [(-1.4, 0.6, jz[k]), (-1.3, 0.4, jz[k]), (-1.2, 0.22, jz[k])]),
                          ('out', [(-0.2, 0.22, jz[k]), (0.5, 0.45, jz[k]), (1.1, 0.58, jz[k]), (1.4, 0.6, jz[k])])):
            pm = MB()
            me = curve_mesh(pts(mock), 0.03, 6, 1)
            pm.add_mesh(me, None, mk_mat('Mat_Path_L%d_%s' % (k + 1, tag), col, 0.45))
            bpy.data.meshes.remove(me)
            pm.finish('Path_L%d_%s' % (k + 1, tag), c, smooth=50, parent=intr)
    mb.finish('Internal_Wires', c, smooth=50, parent=intr)


def build_laptops():
    c = COLL['lap']
    cfg = [
        dict(W=2.00, D=1.40, T=0.13, tilt=17, rz=-90 - 2.5, mat='lap1', sticker='round', miss=None, port=0.55),
        dict(W=1.95, D=1.36, T=0.12, tilt=24, rz=-90 + 1.8, mat='lap2', sticker=None, miss=None, port=-0.45),
        dict(W=2.10, D=1.46, T=0.14, tilt=12, rz=-90 + 4.0, mat='lap3', sticker='rect', miss=(1, 4), port=0.35),
    ]
    for i, g in enumerate(cfg):
        W, D, T = g['W'], g['D'], g['T']
        emp = new_empty('Laptop_%d' % (i + 1), c, (LAP_X + jit(0.12), LAP_Y[i] + jit(0.10), 0))
        emp.rotation_euler = (0, 0, math.radians(g['rz'] + jit(0.6)))
        chassis = MAT[g['mat']]

        mb = MB()
        mb.box((W, D, T), (0, 0, T / 2), mat=chassis)
        mb.cyl(0.05, W * 0.8, (0, D / 2 - 0.035, T + 0.015), (0, math.pi / 2, 0), seg=10, mat=chassis)
        base = mb.finish('Laptop%d_Base' % (i + 1), c, bevel=0.02, bev_seg=2, parent=emp)
        BAKE_QUEUE.append((base, 'medium', False))

        mb = MB()
        kw = W * 0.86
        ytop = D / 2 - 0.26
        mb.box((kw + 0.06, 0.62, 0.008), (0, ytop - 0.215, T + 0.004), mat=MAT['key_well'])
        rows = [(13, 0.06), (13, 0.095), (12, 0.095), (11, 0.095)]
        for r, (n, kh) in enumerate(rows):
            pitch = kw / n
            for k in range(n):
                if g['miss'] == (r, k):
                    continue
                mb.box((pitch * 0.86, kh, 0.03 + jit(0.004)),
                       ((k - (n - 1) / 2) * pitch + jit(0.003), ytop - r * 0.12 + jit(0.003), T + 0.02),
                       (0, 0, jit(math.radians(0.8))), MAT['key'])
        widths = [0.14, 0.14, 0.14, 0.80, 0.14, 0.14, 0.14]
        total = sum(widths) + 0.02 * (len(widths) - 1)
        xx = -total / 2
        for w in widths:
            mb.box((w, 0.095, 0.03 + jit(0.004)), (xx + w / 2 + jit(0.003), ytop - 4 * 0.12, T + 0.02),
                   (0, 0, jit(math.radians(0.8))), MAT['key'])
            xx += w + 0.02
        mb.box((0.62, 0.40, 0.004), (0, -D / 2 + 0.30, T + 0.002), mat=MAT['trackpad'])
        px = g['port']
        mb.box((0.22, 0.05, 0.075), (px, D / 2 - 0.01, 0.075), mat=MAT['jack_dark'])
        mb.finish('Laptop%d_Keys' % (i + 1), c, bevel=0.004, bev_seg=1, parent=emp)

        Hl = W * 0.62
        mb = MB()
        mb.box((W, 0.05, Hl), (0, 0, Hl / 2 + 0.02), mat=chassis)
        lid = mb.finish('Laptop%d_Lid' % (i + 1), c, bevel=0.02, bev_seg=2, parent=emp)
        lid.location = (0, D / 2 - 0.035, T + 0.015)
        lid.rotation_euler = (-math.radians(g['tilt'] + jit(1.5)), 0, 0)
        BAKE_QUEUE.append((lid, 'medium', False))

        mb = MB()
        zc = Hl / 2 + 0.02
        mb.box((W - 0.05, 0.004, Hl - 0.05), (0, -0.027, zc), mat=MAT['bezel_black'])
        sw, sh = W - 0.20, Hl - 0.22
        mb.cyl(0.012, 0.004, (0, -0.029, Hl - 0.02), (math.pi / 2, 0, 0), seg=8, mat=MAT['chip_black'])
        if g['sticker'] == 'round':
            mb.cyl(0.14, 0.003, (jit(0.2) + 0.3, 0.0265, Hl * 0.55), (math.pi / 2, 0, 0), seg=16, mat=MAT['sticker_y'])
        elif g['sticker'] == 'rect':
            mb.box((0.38, 0.003, 0.22), (-0.4 + jit(0.1), 0.0265, Hl * 0.4), (0, math.radians(7), 0), MAT['sticker_w'])
        mb.finish('Laptop%d_Bezel' % (i + 1), c, parent=lid)
        screen_plane('Laptop%d_Screen' % (i + 1), c, (sw, sh), (0, -0.0305, zc + 0.01), (1, 0, 0), (0, 0, 1),
                     mk_mat('Mat_Screen_L%d' % (i + 1), '#0b1b2b', 0.3, 0.0, '#0b1b2b', 0.9), lid)

        Mw = Matrix.Translation(emp.location) @ Matrix.Rotation(emp.rotation_euler.z, 4, 'Z')
        LAP_PORTS[i] = (Mw @ Vector((px, D / 2, 0.075)), (Mw.to_3x3() @ Vector((0, 1, 0))).normalized())


def build_switch():
    c = COLL['sw']
    body = MB()
    body.box((SW_W, SW_D, SW_H), (SW_X, 0, SW_H / 2))
    tmp = body.finish('sw_tmp', COLL['tmp'])
    cuts = new_coll('cuts_sw', COLL['tmp'])
    for y in SW_Y:
        cutter_box(cuts, (0.25, 0.17 + jit(0.004), 0.14), (SW_FACE + 0.05, y + jit(0.006), SW_PORT_Z))
    me = carve(tmp, 0.02, 2, cuts)
    purge(tmp, cuts)
    mb = MB()
    mb.add_mesh(me, None, MAT['sw_plastic'])
    bpy.data.meshes.remove(me)
    sw = mb.finish('Switch_Body', c)
    BAKE_QUEUE.append((sw, 'medium', False))

    mb = MB()
    for i, y in enumerate(SW_Y):
        mb.box((0.02, 0.15, 0.115), (SW_FACE + 0.265, y, SW_PORT_Z), mat=MAT['chip_black'])
        if i < 3:
            on = i != CUT
            lm = MB()
            lm.box((0.012, 0.05, 0.03), (SW_FACE - 0.004, y, 0.37),
                   mat=mk_mat('Mat_SwitchLED_%d' % (i + 1), '#2f5a14' if on else '#2f3d1c', 0.3, 0.0,
                              '#97C459' if on else None, 6.0 if on else 0.0))
            lm.finish('SwitchLED_%d' % (i + 1), c, parent=sw)
        else:
            mb.box((0.012, 0.05, 0.03), (SW_FACE - 0.004, y, 0.37), mat=MAT['led_g_off'])
        SW_PORTS[i] = (Vector((SW_FACE, y, SW_PORT_Z)), Vector((-1, 0, 0)))
    mb.box((0.012, 0.05, 0.03), (SW_FACE - 0.004, 0.62, 0.37), mat=MAT['led_g_on'])
    mb.M0 = Matrix.Translation((5.3, -0.25, SW_H)) @ Matrix.Rotation(math.radians(4), 4, 'Z')
    mb.box((0.9, 0.3, 0.003), (0, 0, 0.0015), mat=MAT['paper'])
    mb.text('5-PORT SWITCH', 0.07, (0, 0.04, 0.0035), mat=MAT['ink'])
    mb.text('10/100', 0.05, (0, -0.07, 0.0035), mat=MAT['ink'])
    mb.M0 = Matrix.Identity(4)
    for k in range(8):
        mb.box((0.5, 0.035, 0.003), (5.65, -0.5 + k * 0.14 + jit(0.006), SW_H + 0.0015), mat=MAT['chip_black'])
    mb.box((0.06, 0.12, 0.10), (SW_X + SW_W / 2 + 0.03, 0.3, 0.2), mat=MAT['chip_black'])
    mb.finish('Switch_Details', c, parent=sw)


def plug(mb, mouth, d, cable_mat):
    a = math.atan2(d.y, d.x)
    c1 = mouth + d * 0.11
    mb.box((0.22, 0.15, 0.125), (c1.x, c1.y, c1.z), (0, 0, a), MAT['plug_clear'])
    t = mouth + d * 0.10
    mb.box((0.07, 0.07, 0.02), (t.x, t.y, t.z + 0.07), (0, 0, a), MAT['plug_clear'])
    b = mouth + d * 0.30
    mb.cyl(0.078, 0.16, (b.x, b.y, b.z), (0, math.pi / 2, a), seg=10, mat=cable_mat, r_top=0.052)
    return mouth + d * 0.38


def leave(p, u, z0):
    pts = [p]
    h = p.z
    for dist, frac in ((0.07, 0.15), (0.20, 0.52), (0.38, 0.85), (0.60, 1.0)):
        q = p + u * dist
        q.z = h - (h - z0) * frac
        pts.append(q)
    return pts


def route(s, e, wander):
    z0 = CABLE_RADIUS + 0.004
    u = e - s
    u.z = 0
    u.normalize()
    v = Vector((-u.y, u.x, 0))
    A = leave(s, u, z0)
    B = leave(e, -u, z0)
    a, b = A[-1], B[-1]
    mids = []
    for t, off in ((0.33, wander), (0.66, -0.7 * wander)):
        p = a.lerp(b, t) + v * off * CABLE_SLACK
        p.z = z0 + jit(0.004)
        mids.append(p)
    return A + mids + B[::-1]


def make_net_cable(name, ma, da, mb_, db, mat, wander):
    mb = MB()
    s = plug(mb, ma, da, mat)
    e = plug(mb, mb_, db, mat)
    me = curve_mesh(route(s, e, wander), CABLE_RADIUS, 6, 1)
    mb.add_mesh(me, None, mat)
    bpy.data.meshes.remove(me)
    mb.finish(name, COLL['cab'], smooth=50)


def build_cables():
    for i in range(3):
        col = '#E24B4A' if i == CUT else '#378ADD'
        lp, ld = LAP_PORTS[i]
        jl = Vector((-1.71, JACK_Y[i], JACK_Z))
        make_net_cable('Cable_L%d_in' % (i + 1), lp, ld, jl, Vector((-1, 0, 0)),
                       mk_mat('Mat_Cable_L%d_in' % (i + 1), col, 0.45), R.choice((-1, 1)) * R.uniform(0.25, 0.5))
        jr = Vector((1.71, JACK_Y[i], JACK_Z))
        sp, sd = SW_PORTS[i]
        make_net_cable('Cable_L%d_out' % (i + 1), jr, Vector((1, 0, 0)), sp, sd,
                       mk_mat('Mat_Cable_L%d_out' % (i + 1), col, 0.45), R.choice((-1, 1)) * R.uniform(0.15, 0.35))


def build_phone():
    c = COLL['phone']
    W, H, T = 0.78, 1.55, 0.085
    root = new_empty('Phone', c, (0.05 + jit(0.1), PHONE_Y, 0))
    root.rotation_euler = (-math.radians(20), 0, math.radians(4 + jit(2)))

    mb = MB()
    mb.box((W, T, H), (0, 0, H / 2), mat=MAT['phone_case'])
    mb.box((0.22, 0.02, 0.22), (-0.2, T / 2 + 0.01, H - 0.2), mat=MAT['chip_black'])
    body = mb.finish('Phone_Body', c, bevel=0.045, bev_seg=3, parent=root)
    BAKE_QUEUE.append((body, 'small', False))

    mb = MB()
    Ws, Hs = W - 0.09, H - 0.20
    mb.box((W - 0.02, 0.004, H - 0.03), (0, -(T / 2 + 0.001), H / 2), mat=MAT['bezel_black'])
    mb.finish('Phone_Bezel', c, parent=root)
    screen_plane('Phone_Screen', c, (Ws, Hs), (0, -(T / 2 + 0.005), H / 2), (1, 0, 0), (0, 0, 1),
                 mk_mat('Mat_Phone_Screen', '#E6F1FB', 0.3, 0.0, '#E6F1FB', 0.8), root)

    Mw = Matrix.Translation(root.location) @ root.rotation_euler.to_matrix().to_4x4()
    P = Mw @ Vector((0, T / 2, 0.95))
    F = Vector((P.x, P.y + 0.78, 0.0))
    dy, dz = F.y - P.y, F.z - P.z
    Ln = math.hypot(dy, dz)
    mid = (P + F) / 2
    mb = MB()
    mb.box((0.55, 0.03, Ln), (mid.x, mid.y, mid.z), (math.atan2(-dy, dz), 0, 0), MAT['stand_pla'])
    mb.finish('Phone_Stand', c, bevel=0.006, bev_seg=1)

    if SHOW_WIFI:
        mb = MB()
        mb.M0 = Matrix.Translation((0, -3.55, 1.6)) @ Matrix.Rotation(0.35, 4, 'X')
        for r in (0.45, 0.80, 1.15):
            mb.arc(r, 0.06, math.radians(45), math.radians(135), 14, MAT['wifi'])
        mb.finish('WiFi_Arcs', COLL['ann'])


def build_badges():
    if not (XRAY and SHOW_BADGES):
        return
    mb = MB()
    L = LID_LIFT
    spots = [(1, ME(-1.85, 0.95, 0)), (2, ME(-0.7, 0.5, 0)), (3, ME(1.85, 0.95, 0)), (4, ME(0.8, 0.4, -0.55)),
             (5, ME(0.8, 0.4, 0.1)), (6, ME(-0.7, 1.55, 0.45) + Vector((0, 0, L))),
             (7, ME(0, 1.6, -0.6) + Vector((0, 0, L))), (8, ME(0.3, 0.55, -0.9))]
    for n, p in spots:
        mb.cyl(0.11, 0.02, (p.x, p.y, p.z), (math.pi / 2, 0, 0), seg=20, mat=MAT['badge_black'])
        mb.text(str(n), 0.14, (p.x, p.y - 0.013, p.z), (math.pi / 2, 0, 0), MAT['badge_white'])
    mb.finish('Badges_1-8', COLL['ann'])


def build_credit():
    cu = bpy.data.curves.new('credit_txt', 'FONT')
    cu.body = 'made by Kenchi Hilario'
    cu.size = 0.30
    cu.align_x = 'CENTER'
    cu.align_y = 'CENTER'
    cu.extrude = 0.012
    cu.resolution_u = 3
    tmp = bpy.data.objects.new('credit_tmp', cu)
    COLL['tmp'].objects.link(tmp)
    me = eval_mesh(tmp)
    bpy.data.objects.remove(tmp, do_unlink=True)
    bpy.data.curves.remove(cu)
    me.materials.append(MAT['credit'])
    smooth_by_angle(me, 40)
    obj = bpy.data.objects.new('MadeBy_KenchiHilario', me)
    COLL['env'].objects.link(obj)
    obj.location = (-0.4, -4.9, 0.0)


def aim(obj, target):
    d = Vector(target) - obj.location
    obj.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()


def build_lighting():
    c = COLL['env']
    scn = bpy.context.scene
    w = bpy.data.worlds.new('NF_World')
    w.use_nodes = True
    bg = w.node_tree.nodes['Background']
    bg.inputs['Color'].default_value = srgb('#a9aeb6')
    bg.inputs['Strength'].default_value = 0.35
    scn.world = w

    def area(name, loc, energy, size, target, color=(1, 1, 1)):
        L = bpy.data.lights.new(name, 'AREA')
        L.energy = energy * LIGHT_SCALE
        L.size = size
        L.color = color
        o = bpy.data.objects.new(name, L)
        c.objects.link(o)
        o.location = loc
        aim(o, target)
    area('Key', (-5, -9, 10), 1800, 6, (0, 0, 0.5), (1.0, 0.95, 0.88))
    area('Fill', (7, -7, 5), 500, 8, (0, 0, 0.5), (0.85, 0.92, 1.0))
    area('Rim', (4, 8, 7), 900, 5, (0, 0, 0.5))
    area('Xray_Top', (0, -2, 9), 500, 3, (0, 0, 0.8))


def build_camera():
    c = COLL['env']
    scn = bpy.context.scene

    def cam(name, loc, target, lens):
        cd = bpy.data.cameras.new(name)
        cd.lens = lens
        cd.clip_end = 300
        o = bpy.data.objects.new(name, cd)
        c.objects.link(o)
        o.location = loc
        aim(o, target)
        return o
    overview = cam('Cam_Overview', (-3.2, -14.0, 6.6), (0.0, 0.2, 0.5), 35)
    xray = cam('Cam_Xray', (-1.8, -6.6, 4.4), (0.0, 0.0, 0.9), 50)
    scn.camera = xray if XRAY else overview


def unwrap(obj):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.uv.smart_project(angle_limit=math.radians(66), island_margin=0.004)
    bpy.ops.object.mode_set(mode='OBJECT')


def baked_material(name, ic, ir, inn=None):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    N, L = m.node_tree.nodes, m.node_tree.links
    b = N['Principled BSDF']
    set_in(b, ('Specular IOR Level', 'Specular'), 0.3)
    tc = N.new('ShaderNodeTexImage'); tc.image = ic; tc.location = (-600, 200)
    L.new(tc.outputs['Color'], b.inputs['Base Color'])
    tr = N.new('ShaderNodeTexImage'); tr.image = ir; tr.location = (-800, -100)
    sep = N.new('ShaderNodeSeparateColor'); sep.location = (-500, -100)
    L.new(tr.outputs['Color'], sep.inputs['Color'])
    L.new(sep.outputs['Green'], b.inputs['Roughness'])
    if inn is not None:
        tn = N.new('ShaderNodeTexImage'); tn.image = inn; tn.location = (-800, -400)
        nm = N.new('ShaderNodeNormalMap'); nm.location = (-500, -400)
        L.new(tn.outputs['Color'], nm.inputs['Color'])
        L.new(nm.outputs['Normal'], b.inputs['Normal'])
    return m


def bake_object(obj, res, with_normal):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    unwrap(obj)
    mats = [s.material for s in obj.material_slots if s.material]

    def target(img):
        for m in mats:
            N = m.node_tree.nodes
            n = N.get('BAKE_TARGET') or N.new('ShaderNodeTexImage')
            n.name = 'BAKE_TARGET'
            n.image = img
            N.active = n

    def new_img(suffix, is_srgb):
        im = bpy.data.images.new('%s_%s' % (obj.name, suffix), res, res, alpha=False)
        im.colorspace_settings.name = 'sRGB' if is_srgb else 'Non-Color'
        return im
    ic = new_img('color', True); target(ic)
    bpy.ops.object.bake(type='DIFFUSE', pass_filter={'COLOR'}, margin=8, use_clear=True)
    ir = new_img('rough', False); target(ir)
    bpy.ops.object.bake(type='ROUGHNESS', margin=8, use_clear=True)
    inn = None
    if with_normal:
        inn = new_img('normal', False); target(inn)
        bpy.ops.object.bake(type='NORMAL', normal_space='TANGENT', margin=8, use_clear=True)
    new_mat = baked_material('%s_baked' % obj.name, ic, ir, inn)
    obj.data.materials.clear()
    obj.data.materials.append(new_mat)


def bake_all():
    scn = bpy.context.scene
    scn.render.engine = 'CYCLES'
    try:
        scn.cycles.device = 'CPU'
    except Exception:
        pass
    old = scn.cycles.samples
    scn.cycles.samples = BAKE_SAMPLES
    for n, (obj, size, with_normal) in enumerate(BAKE_QUEUE):
        print('[NetFault] baking %d/%d  %s' % (n + 1, len(BAKE_QUEUE), obj.name))
        try:
            bake_object(obj, BAKE_RES[size], with_normal)
        except Exception:
            print('[NetFault] bake FAILED for', obj.name)
            traceback.print_exc()
    scn.cycles.samples = old
    for m in list(bpy.data.materials):
        if m.users == 0:
            bpy.data.materials.remove(m)


def verify_names():
    need = ['MadeBy_KenchiHilario', 'Lid_Assembly', 'OLED_Screen', 'Phone_Screen']
    mats = []
    for n in range(1, 4):
        need += ['Cable_L%d_in' % n, 'Cable_L%d_out' % n, 'Laptop_%d' % n, 'Laptop%d_Screen' % n, 'SwitchLED_%d' % n]
        mats += ['Mat_Cable_L%d_in' % n, 'Mat_Cable_L%d_out' % n, 'Mat_SwitchLED_%d' % n]
        if XRAY:
            need += ['Path_L%d_in' % n, 'Path_L%d_out' % n, 'RelayModule_%d' % n,
                     'LED_L%d_green' % n, 'LED_L%d_red' % n]
            mats += ['Mat_Path_L%d_in' % n, 'Mat_Path_L%d_out' % n, 'Mat_Relay_%d' % n,
                     'Mat_LED_L%d_green' % n, 'Mat_LED_L%d_red' % n]
    if XRAY:
        need.append('Internals_Group')
        if CUT_FRONT_WALL:
            need.append('Enclosure_FrontWall')
    bad = [n for n in need if n not in bpy.data.objects] + [n for n in mats if n not in bpy.data.materials]
    if bad:
        print('[NetFault] MISSING names the web page expects:', ', '.join(bad))
    else:
        print('[NetFault] all names the web page expects are present')


def export_glb():
    bpy.ops.object.select_all(action='DESELECT')
    skip = set(COLL['ann'].objects)
    for o in bpy.context.scene.objects:
        if o in skip or o.type not in ('MESH', 'EMPTY'):
            continue
        try:
            o.select_set(True)
        except Exception:
            pass
    kw = dict(filepath=EXPORT_PATH, export_format='GLB', use_selection=True, export_apply=True, export_yup=True,
              export_image_format=EXPORT_IMAGE_FORMAT, export_jpeg_quality=90,
              export_cameras=False, export_lights=False)
    try:
        bpy.ops.export_scene.gltf(**kw)
    except TypeError:
        print('[NetFault] exporter rejected an option, retrying with minimal arguments')
        bpy.ops.export_scene.gltf(filepath=EXPORT_PATH, export_format='GLB', use_selection=True)
    print('[NetFault] exported', EXPORT_PATH)


def step(name, fn):
    try:
        fn()
        print('[NetFault] ok     ', name)
    except Exception:
        print('[NetFault] FAILED ', name)
        traceback.print_exc()


def main():
    setup_scene()
    step('materials', build_materials)
    step('environment', build_environment)
    step('enclosure', build_enclosure)
    step('jacks', build_jacks)
    step('panel', build_panel)
    step('internals', build_internals)
    step('laptops', build_laptops)
    step('switch', build_switch)
    step('cables', build_cables)
    step('phone', build_phone)
    step('badges', build_badges)
    step('credit', build_credit)
    step('lighting', build_lighting)
    step('camera', build_camera)
    try:
        purge(COLL['tmp'])
    except Exception:
        pass
    if BAKE_FOR_WEB:
        step('bake', bake_all)
    if EXPORT_GLB:
        if not BAKE_FOR_WEB:
            print('[NetFault] WARNING: exporting without bake -> procedural wear will NOT appear in the glb')
        step('export', export_glb)
    step('verify', verify_names)
    tris = 0
    for o in bpy.data.objects:
        if o.type == 'MESH':
            o.data.calc_loop_triangles()
            tris += len(o.data.loop_triangles)
    print('[NetFault] done. objects: %d   triangles: %d' % (len(bpy.data.objects), tris))


main()
