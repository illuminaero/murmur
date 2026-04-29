"""
Drone Colorizer — distance-based LED coloring for Skybrush Studio.

Two modes are supported:

1. Skybrush "Color Ramp" custom expression (recommended):
     Light Effect → Output: Custom
     Path:     /path/to/colorizer.py
     Function: distance_factor

   `distance_factor` returns a float in [0.0, 1.0] for each drone, based on the
   distance to its nearest neighbor. Skybrush samples the user-configured color
   ramp (red → orange → yellow → green → cyan) at that output value. Nothing
   runs at import time, so Skybrush can safely re-evaluate the module per frame.

2. Standalone keyframe baking (legacy):
     Open this file in the Text Editor and run it, or:
       python3 util/send_to_blender.py colorizer.py
     This calls bake_keyframes(), which writes emission-color keyframes onto
     per-drone materials. Note: the custom-expression mode is preferred for
     Skybrush — keyframed materials don't drive Skybrush's LED output.
"""

import bpy

# ============================================================================
# CONFIGURATION
# ============================================================================

# Source collection for the standalone bake mode (`bake_keyframes`). The
# Skybrush callback path ignores this and queries Skybrush's own Drones
# collection — see _skybrush_drone_objects().
DRONE_COLLECTION = "Swarm_Drones"

# Distance window for the normalized output (meters).
DISTANCE_MIN = 1.42  # at or below -> 0.0 (left end of color ramp)
DISTANCE_MAX = 3  # at or above -> 1.0 (right end of color ramp)

# ============================================================================
# SKYBRUSH CUSTOM-EXPRESSION ENTRY POINT
# ============================================================================

# Per-frame cache of drone world positions. Skybrush calls the entry point
# once per drone per frame; without caching this would be O(N^2) lookups
# through bpy.data on every frame.
_pos_cache_frame = None
_pos_cache = None


def _skybrush_drone_objects():
    """
    Return the drone objects Skybrush is actually animating.

    Skybrush keeps its rendered drones in `scene.skybrush.settings.drone_collection`
    (default name "Drones") — these are distinct from the murmur source drones
    in `Swarm_Drones`. The `position` argument Skybrush passes to a custom
    expression comes from this collection, so distances must be computed
    against the same set of objects.
    """
    scene = bpy.context.scene
    skybrush = getattr(scene, "skybrush", None) if scene is not None else None
    coll = getattr(skybrush.settings, "drone_collection", None) if skybrush else None
    if coll is None:
        coll = bpy.data.collections.get("Drones")
    if coll is None:
        return []
    # Match Skybrush's iteration exactly — it does not filter by type, so
    # filtering here would offset drone_index against our cache.
    return list(coll.objects)


def _snapshot_positions():
    return [obj.matrix_world.translation.copy() for obj in _skybrush_drone_objects()]


def _positions_for(frame):
    global _pos_cache_frame, _pos_cache
    if _pos_cache_frame != frame or _pos_cache is None:
        _pos_cache_frame = frame
        _pos_cache = _snapshot_positions()
    return _pos_cache


def distance_factor(
    *, frame, time_fraction, drone_index, formation_index, position, drone_count
):
    """
    Skybrush custom-expression callback.

    Returns the drone's minimum-neighbor distance, normalized to
    [DISTANCE_MIN, DISTANCE_MAX] and clamped to [0, 1]. Skybrush samples the
    light-effect color ramp at this output value.
    """
    positions = _positions_for(frame)
    if len(positions) < 2:
        return 1.0

    px, py, pz = position[0], position[1], position[2]
    min_d_sq = float("inf")
    for i, p in enumerate(positions):
        if i == drone_index:
            continue
        dx = p[0] - px
        dy = p[1] - py
        dz = p[2] - pz
        d_sq = dx * dx + dy * dy + dz * dz
        if d_sq < min_d_sq:
            min_d_sq = d_sq

    min_dist = min_d_sq**0.5
    span = DISTANCE_MAX - DISTANCE_MIN
    if span <= 0:
        return 1.0
    t = (min_dist - DISTANCE_MIN) / span
    if t < 0.0:
        return 0.0
    if t > 1.0:
        return 1.0
    return t


# ============================================================================
# STANDALONE KEYFRAME BAKER (legacy mode)
# ============================================================================

# Local color gradient — only used by bake_keyframes(). Skybrush's custom
# expression uses Skybrush's own color ramp UI instead of this table.
COLOR_GRADIENT = [
    (0.0, (1.0, 0.0, 0.0)),
    (0.25, (1.0, 0.5, 0.0)),
    (0.5, (1.0, 1.0, 0.0)),
    (0.75, (0.0, 1.0, 0.0)),
    (1.0, (0.0, 1.0, 1.0)),
]


def _lerp(a, b, t):
    return a + (b - a) * t


def _lerp_color(c1, c2, t):
    return (_lerp(c1[0], c2[0], t), _lerp(c1[1], c2[1], t), _lerp(c1[2], c2[2], t))


def _factor_to_color(t):
    prev = COLOR_GRADIENT[0]
    for stop in COLOR_GRADIENT:
        if stop[0] >= t:
            if stop[0] == prev[0]:
                return stop[1]
            local_t = (t - prev[0]) / (stop[0] - prev[0])
            return _lerp_color(prev[1], stop[1], local_t)
        prev = stop
    return COLOR_GRADIENT[-1][1]


def _get_or_create_emission_material(drone):
    mat_name = f"{drone.name}_LED"
    if mat_name in bpy.data.materials:
        return bpy.data.materials[mat_name]

    mat = bpy.data.materials.new(name=mat_name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    emission = nodes.new("ShaderNodeEmission")
    emission.location = (0, 0)
    emission.inputs["Strength"].default_value = 1.0
    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (200, 0)
    links.new(emission.outputs["Emission"], output.inputs["Surface"])

    if drone.type == "MESH":
        if drone.data.materials:
            drone.data.materials[0] = mat
        else:
            drone.data.materials.append(mat)
    return mat


def _set_drone_color(drone, color, frame):
    mat = _get_or_create_emission_material(drone)
    if not mat.use_nodes:
        return
    emission_node = next((n for n in mat.node_tree.nodes if n.type == "EMISSION"), None)
    if emission_node is None:
        return
    emission_node.inputs["Color"].default_value = (color[0], color[1], color[2], 1.0)
    emission_node.inputs["Color"].keyframe_insert(
        data_path="default_value", frame=frame
    )


def bake_keyframes(start_frame=None, end_frame=None, frame_step=1):
    """
    Bake distance-based emission-color keyframes onto each drone's material.

    This is the legacy mode. For Skybrush Studio LED output, prefer the
    custom-expression flow using `distance_factor` instead.
    """
    print("=" * 70)
    print("  DRONE COLORIZER — keyframe bake")
    print("=" * 70)

    coll = bpy.data.collections.get(DRONE_COLLECTION)
    if coll is None:
        raise ValueError(f"Collection '{DRONE_COLLECTION}' not found")
    drones = [obj for obj in coll.objects if obj.type in ("MESH", "EMPTY")]
    print(f"  Found {len(drones)} drones in '{DRONE_COLLECTION}'")
    if len(drones) < 2:
        print("  ERROR: Need at least 2 drones to calculate distances")
        return

    scene = bpy.context.scene
    start = start_frame if start_frame is not None else scene.frame_start
    end = end_frame if end_frame is not None else scene.frame_end
    total_frames = (end - start) // frame_step + 1
    print(f"  Frame range: {start} - {end} (step: {frame_step}, total: {total_frames})")
    print(f"  Distance window: {DISTANCE_MIN}m - {DISTANCE_MAX}m")
    print("=" * 70)

    frame_count = 0
    for frame in range(start, end + 1, frame_step):
        frame_count += 1
        if frame_count % 100 == 0 or frame == start or frame == end:
            print(
                f"  Processing frame {frame} ({frame_count / total_frames * 100:.1f}%)"
            )

        scene.frame_set(frame)
        positions = [d.matrix_world.translation.copy() for d in drones]

        for i, drone in enumerate(drones):
            px, py, pz = positions[i]
            min_d_sq = float("inf")
            for j, p in enumerate(positions):
                if j == i:
                    continue
                dx, dy, dz = p[0] - px, p[1] - py, p[2] - pz
                d_sq = dx * dx + dy * dy + dz * dz
                if d_sq < min_d_sq:
                    min_d_sq = d_sq
            min_dist = min_d_sq**0.5

            span = DISTANCE_MAX - DISTANCE_MIN
            t = (
                0.0
                if span <= 0
                else max(0.0, min(1.0, (min_dist - DISTANCE_MIN) / span))
            )
            _set_drone_color(drone, _factor_to_color(t), frame)

    # Finalize fcurves (Blender 4.4+ slotted Actions: walk layers/strips/channelbags).
    print("  Finalizing color animation...")
    for drone in drones:
        mat = _get_or_create_emission_material(drone)
        if not (
            mat.node_tree
            and mat.node_tree.animation_data
            and mat.node_tree.animation_data.action
        ):
            continue
        action = mat.node_tree.animation_data.action
        for layer in action.layers:
            for strip in layer.strips:
                if strip.type != "KEYFRAME":
                    continue
                for channelbag in strip.channelbags:
                    for fcurve in channelbag.fcurves:
                        fcurve.update()

    print("=" * 70)
    print(f"  Bake complete: {len(drones)} drones over {total_frames} frames")
    print("=" * 70)


# ============================================================================
# Explicit-run entry point
# ============================================================================
# Only run the bake when invoked directly (Text Editor "Run Script" sets
# __name__ == "__main__"; remote_bpy uses exec() which also sets __main__).
# Importing this file as a module — including by Skybrush as a custom-output
# expression — must NOT trigger the bake, otherwise the per-frame depsgraph
# update inside bake_keyframes() recursively re-evaluates the light effect
# and re-imports this module until the Python recursion limit is hit.

if __name__ == "__main__":
    bake_keyframes()
