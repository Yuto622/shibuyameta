# generate_shibuya_mvp.py
# UE Editor Python (Unreal Engine 5.7) - Shibuya-like walkable MVP generator (idempotent)
 
import math
import random
import traceback
import unreal
 
# =========================
# Parameters (edit here)
# =========================
RANDOM_SEED = 1337
 
CITY_SIZE = 60000.0  # cm, half-extent (radius-ish) of city footprint
 
ROAD_W = 1800.0      # cm
SIDEWALK_W = 500.0   # cm
BLOCK_SIZE = 12000.0 # cm (guideline; blocks are derived from roads)
 
BUILDING_COUNT_MIN = 100
 
BUILDING_HEIGHT_RANGE = (600.0, 8000.0)   # cm (6m - 80m)
FOOTPRINT_RANGE = (450.0, 1800.0)         # cm (4.5m - 18m)
 
# Optional feel tweaks
PLAZA_SIZE = 9000.0                        # cm (central scramble-like space)
GROUND_Z = 0.0
ROAD_Z = 1.5
SIDEWALK_Z = 2.0
 
BUILDING_MARGIN_TO_ROAD = 250.0            # cm
BUILDING_OVERLAP_PADDING = 60.0            # cm
ROTATION_YAW_MAX_DEG = 12.0
 
CENTER_HEIGHT_BOOST = 1.25                 # higher near center
CENTER_DENSITY_BOOST = 1.20                # more attempts near center
 
AUTO_TAG = "AUTO_SHIBUYA_MVP"
 
FOLDER_GROUND = "AUTO_Shared/Ground"
FOLDER_ROADS = "AUTO_Shared/Roads"
FOLDER_BUILDINGS = "AUTO_Shared/Buildings"
FOLDER_LIGHTS = "AUTO_Shared/Lights"
FOLDER_GAMEPLAY = "AUTO_Shared/Gameplay"
 
MAP_FOLDER = "/Game/Maps"
MAP_PATH = "/Game/Maps/Shibuya_MVP"
 
AUTO_MAT_FOLDER = "/Game/Auto/Materials"
MAT_GROUND_PATH = "/Game/Auto/Materials/M_Ground"
MAT_ROAD_PATH = "/Game/Auto/Materials/M_Road"
MAT_BUILDING_PATH = "/Game/Auto/Materials/M_Building"
 
 
# =========================
# Utilities
# =========================
def _log(msg: str):
    unreal.log(msg)
 
 
def _warn(msg: str):
    unreal.log_warning(msg)
 
 
def _err(msg: str):
    unreal.log_error(msg)
 
 
def _hasattr(obj, name: str) -> bool:
    try:
        getattr(obj, name)
        return True
    except Exception:
        return False
 
 
def _safe_set_editor_property(obj, prop: str, value):
    try:
        obj.set_editor_property(prop, value)
        return True
    except Exception:
        return False
 
 
def _plugin_check():
    # Assumes plugins are enabled; warn if not.
    try:
        if hasattr(unreal, "Plugins") and hasattr(unreal.Plugins, "is_plugin_enabled"):
            if not unreal.Plugins.is_plugin_enabled("PythonEditorScriptPlugin"):
                _warn("[WARN] PythonEditorScriptPlugin seems disabled. Enable it for Editor Python scripting.")
            if not unreal.Plugins.is_plugin_enabled("EditorScriptingUtilities"):
                _warn("[WARN] EditorScriptingUtilities seems disabled. Enable it for Editor scripting helpers.")
        else:
            _warn("[WARN] Could not verify plugin enablement (unreal.Plugins API not found).")
    except Exception:
        _warn("[WARN] Plugin check failed (continuing).")
 
 
def _ensure_directory(long_package_dir: str):
    try:
        if not unreal.EditorAssetLibrary.does_directory_exist(long_package_dir):
            unreal.EditorAssetLibrary.make_directory(long_package_dir)
        return True
    except Exception as e:
        _err(f"[ERROR] Failed to ensure directory {long_package_dir}: {e}")
        return False
 
 
def _asset_exists(asset_path: str) -> bool:
    try:
        return unreal.EditorAssetLibrary.does_asset_exist(asset_path)
    except Exception:
        return False
 
 
def _load_asset(asset_path: str):
    try:
        return unreal.EditorAssetLibrary.load_asset(asset_path)
    except Exception:
        return None
 
 
def _save_asset(asset_path: str) -> bool:
    try:
        return unreal.EditorAssetLibrary.save_asset(asset_path)
    except Exception:
        return False
 
 
def _open_or_create_level(map_path: str) -> bool:
    # Robust-ish across UE versions
    try:
        if _asset_exists(map_path):
            _log(f"Map exists, loading: {map_path}")
            if hasattr(unreal, "EditorLoadingAndSavingUtils") and hasattr(unreal.EditorLoadingAndSavingUtils, "load_map"):
                unreal.EditorLoadingAndSavingUtils.load_map(map_path)
                return True
            if hasattr(unreal.EditorLevelLibrary, "load_level"):
                unreal.EditorLevelLibrary.load_level(map_path)
                return True
            _warn("[WARN] No load_map/load_level API found; map may already be open.")
            return True
 
        _log(f"Creating new map: {map_path}")
        if hasattr(unreal.EditorLevelLibrary, "new_level"):
            unreal.EditorLevelLibrary.new_level(map_path)
            return True
 
        # Fallback: create a World asset
        asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
        factory = unreal.WorldFactory()
        name = map_path.split("/")[-1]
        pkg = "/".join(map_path.split("/")[:-1])
        world = asset_tools.create_asset(name, pkg, unreal.World, factory)
        if not world:
            _err("[ERROR] Failed to create world asset via WorldFactory.")
            return False
        _save_asset(map_path)
        if hasattr(unreal, "EditorLoadingAndSavingUtils") and hasattr(unreal.EditorLoadingAndSavingUtils, "load_map"):
            unreal.EditorLoadingAndSavingUtils.load_map(map_path)
            return True
        if hasattr(unreal.EditorLevelLibrary, "load_level"):
            unreal.EditorLevelLibrary.load_level(map_path)
            return True
        return True
    except Exception as e:
        _err(f"[ERROR] Failed to open/create level {map_path}: {e}")
        _err(traceback.format_exc())
        return False
 
 
def _save_current_level() -> bool:
    try:
        if hasattr(unreal, "EditorLoadingAndSavingUtils") and hasattr(unreal.EditorLoadingAndSavingUtils, "save_current_level"):
            return bool(unreal.EditorLoadingAndSavingUtils.save_current_level())
        if hasattr(unreal.EditorLevelLibrary, "save_current_level"):
            return bool(unreal.EditorLevelLibrary.save_current_level())
        _warn("[WARN] No save_current_level API found; please save manually.")
        return False
    except Exception as e:
        _err(f"[ERROR] Failed to save current level: {e}")
        _err(traceback.format_exc())
        return False
 
 
def _get_all_level_actors():
    try:
        return unreal.EditorLevelLibrary.get_all_level_actors()
    except Exception:
        return []
 
 
def _destroy_actor(actor) -> bool:
    try:
        unreal.EditorLevelLibrary.destroy_actor(actor)
        return True
    except Exception:
        return False
 
 
def _delete_tagged_actors(tag: str):
    actors = _get_all_level_actors()
    to_delete = []
    for a in actors:
        try:
            tags = list(a.tags) if hasattr(a, "tags") else []
            if tag in [str(t) for t in tags]:
                to_delete.append(a)
        except Exception:
            continue
 
    if to_delete:
        _log(f"Deleting {len(to_delete)} existing actors with tag '{tag}'...")
    deleted = 0
    for a in to_delete:
        if _destroy_actor(a):
            deleted += 1
    _log(f"Deleted {deleted}/{len(to_delete)} actors.")
 
 
def _set_common_actor_metadata(actor, folder_path: str, tag: str, label: str = None):
    try:
        # Tags
        try:
            tags = list(actor.tags)
            if tag not in [str(t) for t in tags]:
                tags.append(tag)
                actor.tags = tags
        except Exception:
            pass
 
        # Outliner folder
        try:
            actor.set_folder_path(folder_path)
        except Exception:
            pass
 
        # Label
        if label:
            try:
                actor.set_actor_label(label, mark_dirty=False)
            except Exception:
                pass
    except Exception:
        pass
 
 
def _spawn_actor(actor_class, location, rotation):
    try:
        return unreal.EditorLevelLibrary.spawn_actor_from_class(actor_class, location, rotation)
    except Exception as e:
        _err(f"[ERROR] Failed to spawn actor {actor_class}: {e}")
        return None
 
 
def _degrees_to_rotator(yaw_deg: float, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> unreal.Rotator:
    return unreal.Rotator(pitch_deg, yaw_deg, roll_deg)
 
 
def _vec(x, y, z) -> unreal.Vector:
    return unreal.Vector(float(x), float(y), float(z))
 
 
def _clamp(v, a, b):
    return max(a, min(b, v))
 
 
def _lerp(a, b, t):
    return a + (b - a) * t
 
 
def _merge_intervals(intervals):
    if not intervals:
        return []
    intervals = sorted(intervals, key=lambda p: p[0])
    out = [list(intervals[0])]
    for s, e in intervals[1:]:
        if s <= out[-1][1]:
            out[-1][1] = max(out[-1][1], e)
        else:
            out.append([s, e])
    return [(a, b) for a, b in out]
 
 
def _subtract_intervals(base_min, base_max, forbidden_intervals):
    # returns list of allowed intervals in [base_min, base_max]
    forbidden = []
    for a, b in forbidden_intervals:
        a2 = max(base_min, a)
        b2 = min(base_max, b)
        if b2 > a2:
            forbidden.append((a2, b2))
    forbidden = _merge_intervals(forbidden)
    allowed = []
    cur = base_min
    for a, b in forbidden:
        if a > cur:
            allowed.append((cur, a))
        cur = max(cur, b)
    if cur < base_max:
        allowed.append((cur, base_max))
    return allowed
 
 
def _point_line_distance_2d(px, py, ax, ay, bx, by) -> float:
    # distance from P to line AB (infinite line) in 2D
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-6:
        return math.hypot(px - ax, py - ay)
    t = (wx * vx + wy * vy) / denom
    projx = ax + t * vx
    projy = ay + t * vy
    return math.hypot(px - projx, py - projy)
 
 
def _aabb_overlap(a, b, pad=0.0) -> bool:
    # a,b: (minx,miny,maxx,maxy)
    return not (
        (a[2] + pad) <= (b[0] - pad) or
        (a[0] - pad) >= (b[2] + pad) or
        (a[3] + pad) <= (b[1] - pad) or
        (a[1] - pad) >= (b[3] + pad)
    )
 
 
def _rotated_aabb_half_extents(w, d, yaw_deg):
    # Given rectangle w (x), d (y) in cm, rotated around Z by yaw, return half-extents of resulting AABB.
    yaw = math.radians(yaw_deg)
    c = abs(math.cos(yaw))
    s = abs(math.sin(yaw))
    hx = 0.5 * (w * c + d * s)
    hy = 0.5 * (w * s + d * c)
    return hx, hy
 
 
# =========================
# Materials
# =========================
def _create_simple_material(mat_path: str, base_color: unreal.LinearColor, roughness: float, metallic: float = 0.0) -> object:
    try:
        _ensure_directory(AUTO_MAT_FOLDER)
 
        if _asset_exists(mat_path):
            mat = _load_asset(mat_path)
            if mat:
                return mat
 
        asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
        factory = unreal.MaterialFactoryNew()
        name = mat_path.split("/")[-1]
        pkg = "/".join(mat_path.split("/")[:-1])
 
        mat = asset_tools.create_asset(name, pkg, unreal.Material, factory)
        if not mat:
            return None
 
        try:
            unreal.MaterialEditingLibrary.delete_all_material_expressions(mat)
        except Exception:
            pass
 
        # BaseColor
        c3 = unreal.MaterialEditingLibrary.create_material_expression(
            mat, unreal.MaterialExpressionConstant3Vector, -420, -40
        )
        try:
            c3.constant = base_color
        except Exception:
            _safe_set_editor_property(c3, "constant", base_color)
 
        # Roughness
        r1 = unreal.MaterialEditingLibrary.create_material_expression(
            mat, unreal.MaterialExpressionConstant, -420, 120
        )
        try:
            r1.r = float(roughness)
        except Exception:
            _safe_set_editor_property(r1, "r", float(roughness))
 
        # Metallic
        m1 = unreal.MaterialEditingLibrary.create_material_expression(
            mat, unreal.MaterialExpressionConstant, -420, 220
        )
        try:
            m1.r = float(metallic)
        except Exception:
            _safe_set_editor_property(m1, "r", float(metallic))
 
        unreal.MaterialEditingLibrary.connect_material_property(c3, "", unreal.MaterialProperty.MP_BASE_COLOR)
        unreal.MaterialEditingLibrary.connect_material_property(r1, "", unreal.MaterialProperty.MP_ROUGHNESS)
        unreal.MaterialEditingLibrary.connect_material_property(m1, "", unreal.MaterialProperty.MP_METALLIC)
 
        try:
            unreal.MaterialEditingLibrary.recompile_material(mat)
        except Exception:
            pass
 
        _save_asset(mat_path)
        return mat
    except Exception as e:
        _warn(f"[WARN] Failed to create material {mat_path}: {e}")
        _warn(traceback.format_exc())
        return None
 
 
def _build_materials():
    mats = {"ground": None, "road": None, "building": None}
    try:
        mats["ground"] = _create_simple_material(
            MAT_GROUND_PATH,
            unreal.LinearColor(0.45, 0.45, 0.45, 1.0),
            roughness=0.92,
            metallic=0.0,
        )
        mats["road"] = _create_simple_material(
            MAT_ROAD_PATH,
            unreal.LinearColor(0.10, 0.10, 0.10, 1.0),
            roughness=0.95,
            metallic=0.0,
        )
        mats["building"] = _create_simple_material(
            MAT_BUILDING_PATH,
            unreal.LinearColor(0.55, 0.55, 0.57, 1.0),
            roughness=0.88,
            metallic=0.0,
        )
        return mats
    except Exception as e:
        _warn(f"[WARN] Material creation failed (continuing): {e}")
        _warn(traceback.format_exc())
        return mats
 
 
# =========================
# Mesh helpers
# =========================
def _load_basic_shape_meshes():
    plane = _load_asset("/Engine/BasicShapes/Plane.Plane")
    cube = _load_asset("/Engine/BasicShapes/Cube.Cube")
    cylinder = _load_asset("/Engine/BasicShapes/Cylinder.Cylinder")
    if not plane or not cube:
        _err("[ERROR] Failed to load /Engine/BasicShapes meshes. Ensure Engine content is available.")
    return plane, cube, cylinder
 
 
def _apply_static_mesh(actor, static_mesh, material=None, collision_profile="BlockAll"):
    try:
        smc = None
        if hasattr(actor, "static_mesh_component"):
            smc = actor.static_mesh_component
        if not smc:
            smc = actor.get_component_by_class(unreal.StaticMeshComponent)
        if not smc:
            return False
 
        smc.set_static_mesh(static_mesh)
 
        if material:
            try:
                smc.set_material(0, material)
            except Exception:
                pass
 
        try:
            smc.set_mobility(unreal.ComponentMobility.STATIC)
        except Exception:
            pass
 
        try:
            smc.set_collision_profile_name(collision_profile)
        except Exception:
            pass
 
        return True
    except Exception:
        return False
 
 
def _spawn_mesh_actor(static_mesh, location, rotation, scale3d, folder, tag, label, material=None, collision_profile="BlockAll"):
    actor = _spawn_actor(unreal.StaticMeshActor, location, rotation)
    if not actor:
        return None
    _set_common_actor_metadata(actor, folder, tag, label)
    try:
        actor.set_actor_scale3d(scale3d)
    except Exception:
        pass
    _apply_static_mesh(actor, static_mesh, material=material, collision_profile=collision_profile)
    return actor
 
 
# =========================
# Generation
# =========================
def _spawn_ground(plane_mesh, mats):
    size = CITY_SIZE * 2.2  # slightly larger than city
    sx = size / 100.0
    sy = size / 100.0
    loc = _vec(0, 0, GROUND_Z)
    rot = _degrees_to_rotator(0)
    scale = unreal.Vector(sx, sy, 1.0)
    _spawn_mesh_actor(
        plane_mesh,
        loc,
        rot,
        scale,
        FOLDER_GROUND,
        AUTO_TAG,
        "AUTO_Ground",
        material=mats.get("ground"),
        collision_profile="BlockAll",
    )
 
 
def _spawn_road_strip(plane_mesh, mats, center_xy, yaw_deg, length_cm, width_cm, z_cm, folder, label_prefix, idx, material_key="road"):
    sx = max(1e-3, length_cm / 100.0)
    sy = max(1e-3, width_cm / 100.0)
    loc = _vec(center_xy[0], center_xy[1], z_cm)
    rot = _degrees_to_rotator(yaw_deg)
    scale = unreal.Vector(sx, sy, 1.0)
    mat = mats.get(material_key)
    return _spawn_mesh_actor(
        plane_mesh,
        loc,
        rot,
        scale,
        folder,
        AUTO_TAG,
        f"{label_prefix}_{idx:02d}",
        material=mat,
        collision_profile="BlockAll",
    )
 
 
def _spawn_road_with_sidewalks(plane_mesh, mats, center_xy, yaw_deg, length_cm, road_w, sidewalk_w, z_road, z_sidewalk, idx_base):
    # Road
    _spawn_road_strip(
        plane_mesh, mats,
        center_xy=center_xy,
        yaw_deg=yaw_deg,
        length_cm=length_cm,
        width_cm=road_w,
        z_cm=z_road,
        folder=FOLDER_ROADS,
        label_prefix="AUTO_Road",
        idx=idx_base,
        material_key="road",
    )
 
    # Sidewalks (use ground material but slightly brighter via ground mat; acceptable)
    # Offset along "right" of road
    yaw = math.radians(yaw_deg)
    rx = -math.sin(yaw)
    ry = math.cos(yaw)
    offset = (road_w * 0.5) + (sidewalk_w * 0.5)
    for j, sign in enumerate([1.0, -1.0]):
        cx = center_xy[0] + rx * offset * sign
        cy = center_xy[1] + ry * offset * sign
        _spawn_road_strip(
            plane_mesh, mats,
            center_xy=(cx, cy),
            yaw_deg=yaw_deg,
            length_cm=length_cm,
            width_cm=sidewalk_w,
            z_cm=z_sidewalk,
            folder=FOLDER_ROADS,
            label_prefix="AUTO_Sidewalk",
            idx=idx_base * 10 + j,
            material_key="ground",
        )
 
 
def _road_layout():
    # A minimal Shibuya-ish network: 2 main axes + auxiliary offsets + diagonal.
    # Returns:
    # - axis_roads: dict with vertical x positions and horizontal y positions (centers)
    # - diagonal: (ax,ay,bx,by, half_corridor)
    # - segments_to_spawn: list of (center_xy, yaw_deg, length_cm, road_w)
    aux_x = [-0.32 * CITY_SIZE, 0.42 * CITY_SIZE]
    aux_y = [-0.33 * CITY_SIZE, 0.36 * CITY_SIZE]
    main_x = [0.0]
    main_y = [0.0]
 
    # Axis road spawn list
    segs = []
 
    # Main cross
    segs.append(((0.0, 0.0), 0.0, CITY_SIZE * 2.25, ROAD_W))     # horizontal (along X)
    segs.append(((0.0, 0.0), 90.0, CITY_SIZE * 2.25, ROAD_W))    # vertical (along Y)
 
    # Aux roads (slightly shorter for asymmetry)
    for i, x in enumerate(aux_x):
        segs.append(((x, -0.10 * CITY_SIZE), 90.0, CITY_SIZE * 1.95, ROAD_W * 0.90))
    for i, y in enumerate(aux_y):
        segs.append(((-0.08 * CITY_SIZE, y), 0.0, CITY_SIZE * 1.95, ROAD_W * 0.90))
 
    # Add one short "knot" road near center to suggest complexity
    segs.append(((0.18 * CITY_SIZE, 0.12 * CITY_SIZE), 25.0, CITY_SIZE * 0.85, ROAD_W * 0.70))
    segs.append(((-0.22 * CITY_SIZE, -0.05 * CITY_SIZE), -20.0, CITY_SIZE * 0.78, ROAD_W * 0.65))
 
    # Diagonal artery through/near center
    diag_yaw = 45.0
    diag_len = CITY_SIZE * 2.6
    diag_center = (-0.06 * CITY_SIZE, 0.05 * CITY_SIZE)
    segs.append((diag_center, diag_yaw, diag_len, ROAD_W * 0.85))
 
    # Plaza (scramble-like)
    plaza_center = (0.0, 0.0)
    segs.append((plaza_center, 0.0, PLAZA_SIZE, PLAZA_SIZE))  # square
 
    axis = {
        "vertical_x": sorted(main_x + aux_x),
        "horizontal_y": sorted(main_y + aux_y),
    }
 
    # Diagonal line endpoints for avoidance checks
    yaw = math.radians(diag_yaw)
    dx = math.cos(yaw) * (diag_len * 0.5)
    dy = math.sin(yaw) * (diag_len * 0.5)
    ax = diag_center[0] - dx
    ay = diag_center[1] - dy
    bx = diag_center[0] + dx
    by = diag_center[1] + dy
    diag_half = (ROAD_W * 0.5 + SIDEWALK_W) * 1.10
 
    return axis, (ax, ay, bx, by, diag_half), segs
 
 
def _spawn_roads(plane_mesh, mats):
    axis, diag, segs = _road_layout()
    _log(f"Road definitions: {len(segs)} segments")
 
    for i, (center_xy, yaw_deg, length_cm, road_w) in enumerate(segs):
        # Plaza is encoded as length==width (square) and yaw_deg==0; treat as road strip without sidewalks
        if abs(length_cm - road_w) < 1e-3 and length_cm >= PLAZA_SIZE * 0.95:
            _spawn_road_strip(
                plane_mesh, mats,
                center_xy=center_xy,
                yaw_deg=0.0,
                length_cm=length_cm,
                width_cm=road_w,
                z_cm=ROAD_Z,
                folder=FOLDER_ROADS,
                label_prefix="AUTO_Plaza",
                idx=i,
                material_key="road",
            )
        else:
            _spawn_road_with_sidewalks(
                plane_mesh, mats,
                center_xy=center_xy,
                yaw_deg=yaw_deg,
                length_cm=length_cm,
                road_w=road_w,
                sidewalk_w=SIDEWALK_W,
                z_road=ROAD_Z,
                z_sidewalk=SIDEWALK_Z,
                idx_base=i + 1,
            )
 
    return axis, diag
 
 
def _derive_blocks(axis_roads):
    # Compute block rectangles from axis-aligned road corridors only
    corridor_half = (ROAD_W * 0.5 + SIDEWALK_W)
 
    xs_forbidden = []
    for x in axis_roads["vertical_x"]:
        xs_forbidden.append((x - corridor_half, x + corridor_half))
    ys_forbidden = []
    for y in axis_roads["horizontal_y"]:
        ys_forbidden.append((y - corridor_half, y + corridor_half))
 
    # Also exclude central plaza region by treating it as forbidden in both axes (approx square)
    plaza_half = PLAZA_SIZE * 0.5 + SIDEWALK_W
    xs_forbidden.append((-plaza_half, plaza_half))
    ys_forbidden.append((-plaza_half, plaza_half))
 
    x_allowed = _subtract_intervals(-CITY_SIZE, CITY_SIZE, xs_forbidden)
    y_allowed = _subtract_intervals(-CITY_SIZE, CITY_SIZE, ys_forbidden)
 
    blocks = []
    min_dim = max(FOOTPRINT_RANGE[1] * 3.0, BLOCK_SIZE * 0.35)
    for xa, xb in x_allowed:
        for ya, yb in y_allowed:
            w = xb - xa
            h = yb - ya
            if w < min_dim or h < min_dim:
                continue
            # keep some variety: skip some outer tiny-ish slivers
            blocks.append((xa, ya, xb, yb))
    return blocks
 
 
def _spawn_buildings(cube_mesh, mats, axis_roads, diag, rng):
    blocks = _derive_blocks(axis_roads)
    if not blocks:
        _warn("[WARN] No blocks derived; placing buildings in fallback scatter.")
        blocks = [(-CITY_SIZE, -CITY_SIZE, CITY_SIZE, CITY_SIZE)]
 
    ax, ay, bx, by, diag_half = diag
 
    placed_aabbs = []
    building_count = 0
 
    def center_factor(x, y):
        # 0 at edge -> 1 at center
        d = math.hypot(x, y)
        t = _clamp(d / CITY_SIZE, 0.0, 1.0)
        return 1.0 - t
 
    def try_place_in_block(block, desired, max_attempts_base):
        nonlocal building_count
        xa, ya, xb, yb = block
 
        # Slightly boost attempts if closer to center
        cx = 0.5 * (xa + xb)
        cy = 0.5 * (ya + yb)
        cf = center_factor(cx, cy)
        attempts = int(max_attempts_base * (1.0 + cf * (CENTER_DENSITY_BOOST - 1.0)))
 
        success = 0
        for _ in range(attempts):
            if success >= desired:
                break
 
            # footprint
            w = rng.uniform(FOOTPRINT_RANGE[0], FOOTPRINT_RANGE[1])
            d = rng.uniform(FOOTPRINT_RANGE[0], FOOTPRINT_RANGE[1])
 
            yaw = rng.uniform(-ROTATION_YAW_MAX_DEG, ROTATION_YAW_MAX_DEG)
 
            # expanded AABB for rotated bounds
            hx, hy = _rotated_aabb_half_extents(w, d, yaw)
 
            margin = BUILDING_MARGIN_TO_ROAD
            minx = xa + margin + hx
            maxx = xb - margin - hx
            miny = ya + margin + hy
            maxy = yb - margin - hy
 
            if maxx <= minx or maxy <= miny:
                continue
 
            x = rng.uniform(minx, maxx)
            y = rng.uniform(miny, maxy)
 
            # avoid diagonal road corridor
            dist_diag = _point_line_distance_2d(x, y, ax, ay, bx, by)
            if dist_diag < diag_half:
                continue
 
            # height distribution: higher near center
            cf = center_factor(x, y)  # 0..1
            h_min, h_max = BUILDING_HEIGHT_RANGE
            base_h = rng.uniform(h_min, h_max)
            boost = 1.0 + cf * CENTER_HEIGHT_BOOST
            height = _clamp(base_h * boost, h_min, h_max * (1.0 + CENTER_HEIGHT_BOOST))
 
            # AABB for overlap checks (use rotated AABB)
            aabb = (x - hx, y - hy, x + hx, y + hy)
 
            ok = True
            for other in placed_aabbs:
                if _aabb_overlap(aabb, other, pad=BUILDING_OVERLAP_PADDING):
                    ok = False
                    break
            if not ok:
                continue
 
            # spawn
            loc = _vec(x, y, height * 0.5)
            rot = _degrees_to_rotator(yaw_deg=yaw)
            scale = unreal.Vector(w / 100.0, d / 100.0, height / 100.0)
 
            label = f"AUTO_Building_{building_count:04d}"
            a = _spawn_mesh_actor(
                cube_mesh, loc, rot, scale,
                FOLDER_BUILDINGS, AUTO_TAG, label,
                material=mats.get("building"),
                collision_profile="BlockAll",
            )
            if not a:
                continue
 
            placed_aabbs.append(aabb)
            building_count += 1
            success += 1
 
        return success
 
    # First pass: each block gets 3..10 buildings
    rng.shuffle(blocks)
    for b in blocks:
        desired = rng.randint(3, 10)
        try_place_in_block(b, desired=desired, max_attempts_base=90)
 
    # Ensure minimum count
    if building_count < BUILDING_COUNT_MIN:
        _log(f"Building count {building_count} < {BUILDING_COUNT_MIN}, adding more...")
        # Additional passes: bias toward larger / central blocks
        scored = []
        for b in blocks:
            xa, ya, xb, yb = b
            area = (xb - xa) * (yb - ya)
            cx = 0.5 * (xa + xb)
            cy = 0.5 * (ya + yb)
            cf = center_factor(cx, cy)
            score = area * (1.0 + cf * 0.65)
            scored.append((score, b))
        scored.sort(key=lambda t: t[0], reverse=True)
        blocks_sorted = [b for _, b in scored]
 
        safety = 0
        while building_count < BUILDING_COUNT_MIN and safety < 40:
            for b in blocks_sorted[: max(6, len(blocks_sorted) // 2)]:
                if building_count >= BUILDING_COUNT_MIN:
                    break
                try_place_in_block(b, desired=rng.randint(2, 6), max_attempts_base=80)
            safety += 1
 
    _log(f"Buildings placed: {building_count}")
    return building_count
 
 
def _spawn_lighting_and_post():
    # Directional Light
    dl = _spawn_actor(unreal.DirectionalLight, _vec(0, 0, 12000), _degrees_to_rotator(yaw_deg=35.0, pitch_deg=-55.0))
    if dl:
        _set_common_actor_metadata(dl, FOLDER_LIGHTS, AUTO_TAG, "AUTO_DirectionalLight")
        try:
            comp = dl.get_component_by_class(unreal.DirectionalLightComponent)
            if comp:
                _safe_set_editor_property(comp, "intensity", 10.0)
                _safe_set_editor_property(comp, "light_color", unreal.LinearColor(1.0, 0.98, 0.95, 1.0))
        except Exception:
            pass
 
    # Sky Light
    sl = _spawn_actor(unreal.SkyLight, _vec(0, 0, 8000), _degrees_to_rotator(0))
    if sl:
        _set_common_actor_metadata(sl, FOLDER_LIGHTS, AUTO_TAG, "AUTO_SkyLight")
        try:
            comp = sl.get_component_by_class(unreal.SkyLightComponent)
            if comp:
                _safe_set_editor_property(comp, "intensity", 1.2)
                # Ensure it captures if possible
                try:
                    sl.recapture_sky()
                except Exception:
                    pass
        except Exception:
            pass
 
    # Post Process Volume (Unbound)
    pp = _spawn_actor(unreal.PostProcessVolume, _vec(0, 0, 200), _degrees_to_rotator(0))
    if pp:
        _set_common_actor_metadata(pp, FOLDER_LIGHTS, AUTO_TAG, "AUTO_PostProcess")
        _safe_set_editor_property(pp, "b_unbound", True)
        try:
            # Try common property path
            settings = pp.get_editor_property("settings") if _hasattr(pp, "get_editor_property") else None
            if settings:
                # Mild Shibuya-ish: slightly desaturated, more contrast
                _safe_set_editor_property(settings, "b_override_color_saturation", True)
                _safe_set_editor_property(settings, "color_saturation", unreal.Vector4(0.88, 0.88, 0.88, 1.0))
                _safe_set_editor_property(settings, "b_override_color_contrast", True)
                _safe_set_editor_property(settings, "color_contrast", unreal.Vector4(1.10, 1.10, 1.10, 1.0))
                _safe_set_editor_property(pp, "settings", settings)
        except Exception:
            # Fallback: attempt direct set on volume
            try:
                _safe_set_editor_property(pp, "b_unbound", True)
            except Exception:
                pass
 
 
def _spawn_gameplay(axis_roads):
    # PlayerStart near center plaza edge on a road (slightly above road surface)
    ps = _spawn_actor(unreal.PlayerStart, _vec(900.0, -600.0, ROAD_Z + 120.0), _degrees_to_rotator(0))
    if ps:
        _set_common_actor_metadata(ps, FOLDER_GAMEPLAY, AUTO_TAG, "AUTO_PlayerStart")
 
    # NavMeshBoundsVolume covering the whole city
    nav = _spawn_actor(unreal.NavMeshBoundsVolume, _vec(0, 0, 50), _degrees_to_rotator(0))
    if nav:
        _set_common_actor_metadata(nav, FOLDER_GAMEPLAY, AUTO_TAG, "AUTO_NavMeshBounds")
        # Scale heuristic: default brush is typically a small cube; scaling should expand bounds.
        scale_xy = max(50.0, (CITY_SIZE * 2.4) / 200.0)  # assuming ~200cm default brush extent
        nav.set_actor_scale3d(unreal.Vector(scale_xy, scale_xy, 20.0))
 
 
# =========================
# Main
# =========================
def main():
    step = 0
    try:
        _plugin_check()
 
        step = 1
        _log("[1/8] Create/Open map & ensure folders...")
        _ensure_directory(MAP_FOLDER)
        if not _open_or_create_level(MAP_PATH):
            raise RuntimeError("Failed to create/open target level.")
 
        step = 2
        _log("[2/8] Cleanup previous AUTO_SHIBUYA_MVP actors...")
        _delete_tagged_actors(AUTO_TAG)
 
        step = 3
        _log("[3/8] Load basic shapes...")
        plane_mesh, cube_mesh, _ = _load_basic_shape_meshes()
        if not plane_mesh or not cube_mesh:
            raise RuntimeError("BasicShapes meshes not loaded; cannot continue.")
 
        step = 4
        _log("[4/8] Create/Load simple materials...")
        mats = _build_materials()
        if not (mats.get("ground") and mats.get("road") and mats.get("building")):
            _warn("[WARN] Some materials could not be created/loaded. Continuing with default materials where needed.")
 
        step = 5
        _log("[5/8] Spawn ground...")
        _spawn_ground(plane_mesh, mats)
 
        step = 6
        _log("[6/8] Spawn roads (with intersections & central plaza)...")
        axis_roads, diag = _spawn_roads(plane_mesh, mats)
 
        step = 7
        _log("[7/8] Spawn blocks & buildings (100+)...")
        rng = random.Random(RANDOM_SEED)
        building_count = _spawn_buildings(cube_mesh, mats, axis_roads, diag, rng)
        if building_count < BUILDING_COUNT_MIN:
            _warn(f"[WARN] Only {building_count} buildings placed (target min {BUILDING_COUNT_MIN}). Consider increasing CITY_SIZE or loosening margins.")
 
        step = 8
        _log("[8/8] Spawn lighting, post process, gameplay actors...")
        _spawn_lighting_and_post()
        _spawn_gameplay(axis_roads)
 
        _log("Saving map...")
        _save_current_level()
 
        unreal.log("Generated Shibuya MVP ✅")
 
    except Exception as e:
        _err(f"[FAILED] Step {step}/8: {e}")
        _err(traceback.format_exc())
 
 
if __name__ == "__main__":
    main()
