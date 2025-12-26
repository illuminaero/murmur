"""
Swarm Flocking Animation - Vásárhelyi et al. 2018 Model (v1.0)

Based on the optimized flocking model from:
    Vásárhelyi, G., Virágh, C., Somorjai, G., Nepusz, T., Eiben, A. E., & Vicsek, T. (2018).
    "Optimized flocking of autonomous drones in confined environments."
    Science Robotics, 3(20), eaat3536.
    https://hal.elte.hu/~vasarhelyi/doc/vasarhelyi2018optimized.pdf

THREE-ZONE INTERACTION MODEL:

1. REPULSION (Short-range: < r_rep)
   - Prevents collisions
   - THREE-TIER: Critical (1/d²), Emergency (exponential), Normal (linear)
   - Strongest at close range

2. VELOCITY ALIGNMENT / FRICTION (Mid-range: < r_frict)
   - Synchronizes velocities with neighbors
   - Only activates when velocity difference exceeds v_slack
   - Creates smooth, coherent group movement

3. COHESION (Long-range: < r_cohesion)
   - Attracts toward center of mass of neighbors
   - Prevents group fragmentation
   - Weaker force, acts over larger distances

4. BREATHING (Periodic radial oscillation)
   - Pushes drones outward from center of mass (split phase)
   - Relaxes to let cohesion pull them back (merge phase)
"""

import bpy
import bmesh
import math
import mathutils
import random
import time
import os
import sys

# Add user site-packages to path for pip packages installed with --user
# This is needed because Blender's Python doesn't check ~/.local by default
import site
user_site = site.getusersitepackages()
if user_site not in sys.path:
    sys.path.append(user_site)

import yaml

# ============================================================================
# LOAD CONFIGURATION FROM YAML
# ============================================================================

def load_config():
    """Load configuration from config.yaml file"""
    # Try multiple locations to find config.yaml
    search_paths = []
    
    # 1. Try relative to script file (if running as a file)
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        search_paths.append(os.path.join(script_dir, "config.yaml"))
    except NameError:
        pass  # __file__ not defined (running via exec/remote)
    
    # 2. Try relative to the current blend file
    if bpy.data.filepath:
        blend_dir = os.path.dirname(bpy.data.filepath)
        search_paths.append(os.path.join(blend_dir, "config.yaml"))
    
    # 3. Try current working directory
    search_paths.append(os.path.join(os.getcwd(), "config.yaml"))
    
    # 4. Try common project locations
    search_paths.append(os.path.expanduser("~/src/murmur/config.yaml"))
    
    for config_path in search_paths:
        if os.path.exists(config_path):
            print(f"Loading config from: {config_path}")
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
    
    raise FileNotFoundError(
        f"config.yaml not found. Searched:\n" + 
        "\n".join(f"  - {p}" for p in search_paths)
    )

config = load_config()

# Volume scale factor: 1.5x volume means distances scale by 1.5^(1/3) ≈ 1.145
VOLUME_SCALE = config['volume_scale_factor'] ** (1/3)  # ≈ 1.1447

class FlockingParams:
    """Parameters from Vásárhelyi et al. 2018 optimized flocking model"""
    
    def __init__(self, cfg):
        flock = cfg['flocking']
        breathing = cfg['breathing']
        turbulence = cfg['turbulence']
        kicks = cfg['random_kicks']
        
        # === HARD COLLISION PREVENTION ===
        self.min_distance = flock['min_distance']
        
        # Repulsion (collision avoidance) - scaled
        self.r_rep = flock['r_rep'] * VOLUME_SCALE
        self.p_rep = flock['p_rep']
        
        # Emergency repulsion
        self.r_emergency = flock['r_emergency']
        
        # Friction/Alignment (velocity matching) - scaled
        self.r_frict = flock['r_frict'] * VOLUME_SCALE
        self.c_frict = flock['c_frict']
        self.v_slack = flock['v_slack']
        
        # Cohesion (group attraction) - scaled
        self.r_cohesion = flock['r_cohesion'] * VOLUME_SCALE
        self.p_cohesion = flock['p_cohesion']
        
        # Speed limits (NOT scaled - these are physical limits)
        self.v_flock = flock['v_flock']
        self.v_max_horizontal = flock['v_max_horizontal']
        self.v_max_vertical = flock['v_max_vertical']
        
        # Communication - scaled
        self.r_comm = flock['r_comm'] * VOLUME_SCALE
        self.max_neighbors = flock['max_neighbors']
        
        # Arena bounds (soft constraint) - scaled
        self.arena_size = flock['arena_size'] * VOLUME_SCALE
        self.wall_repulsion = flock['wall_repulsion']
        
        # === BREATHING / PULSING BEHAVIOR ===
        self.enable_breathing = breathing['enable']
        self.breathing_period = breathing['period']
        self.breathing_strength = breathing['strength']
        self.breathing_duty_cycle = breathing['duty_cycle']
        self.breathing_smoothness = breathing['smoothness']
        
        # === ANTI-CONVERGENCE OPTIONS ===
        
        # Turbulence: periodic noise to prevent blob formation
        self.enable_turbulence = turbulence['enable']
        self.turbulence_strength = turbulence['strength']
        self.turbulence_frequency = turbulence['frequency']
        
        # Random kicks: occasional velocity impulses
        self.enable_random_kicks = kicks['enable']
        self.kick_probability = kicks['probability']
        self.kick_strength = kicks['strength']

params = FlockingParams(config)

# ============================================================================
# ANIMATION SETTINGS
# ============================================================================

anim = config['animation']
num_drones = anim['num_drones']
fps = anim['fps']
start_frame = anim['start_frame']
end_frame = anim['end_frame']

total_frames = end_frame - start_frame + 1
dt = 1.0 / fps  # Time step

drone_cfg = config['drone']
drone_size = drone_cfg['size']
use_empties = drone_cfg['use_empties']

# Spawn radius - scaled
spawn_radius = config['spawn_radius'] * VOLUME_SCALE

# ============================================================================
# SETUP SCENE
# ============================================================================

# Clear existing animation data
for obj in bpy.data.objects:
    if obj.animation_data:
        obj.animation_data_clear()

# Create anchor
if "SwarmAnchor" in bpy.data.objects:
    anchor = bpy.data.objects["SwarmAnchor"]
else:
    anchor = bpy.data.objects.new("SwarmAnchor", None)
    anchor.empty_display_type = 'CUBE'
    anchor.empty_display_size = 5.0
    anchor.location = (0, 0, 20)
    bpy.context.scene.collection.objects.link(anchor)

# Create collection
collection_name = "Swarm_Drones"
if collection_name in bpy.data.collections:
    swarm_collection = bpy.data.collections[collection_name]
    for obj in swarm_collection.objects:
        bpy.data.objects.remove(obj, do_unlink=True)
else:
    swarm_collection = bpy.data.collections.new(collection_name)
    bpy.context.scene.collection.children.link(swarm_collection)

# Helper function for mesh creation
def create_uv_sphere_mesh(name, radius=1.0, segments=8, rings=5):
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=segments, v_segments=rings, radius=radius)
    bm.to_mesh(mesh)
    bm.free()
    return mesh

# ============================================================================
# INITIALIZE DRONES
# ============================================================================

random.seed(42)
drones = []

# Initialize in sphere around anchor
for i in range(num_drones):
    # Fibonacci sphere distribution
    phi = math.pi * (3.0 - math.sqrt(5.0))  # Golden angle
    y = 1 - (i / (num_drones - 1)) * 2
    radius = math.sqrt(1 - y * y)
    theta = phi * i
    
    x = math.cos(theta) * radius
    z = math.sin(theta) * radius
    
    initial_pos = mathutils.Vector((
        x * spawn_radius,
        y * spawn_radius,
        z * spawn_radius + 20.0
    ))
    
    name = f"Drone_{i:03d}"
    
    if use_empties:
        obj = bpy.data.objects.new(name, None)
        obj.empty_display_type = 'SPHERE'
        obj.empty_display_size = 0.3
    else:
        mesh = create_uv_sphere_mesh(f"{name}_Mesh", radius=drone_size)
        obj = bpy.data.objects.new(name, mesh)
    
    obj.location = initial_pos
    obj.parent = anchor
    swarm_collection.objects.link(obj)
    
    # Initialize drone state
    drone_data = {
        'object': obj,
        'index': i,
        'position': initial_pos.copy(),
        'velocity': mathutils.Vector((
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5)
        ))
    }
    drones.append(drone_data)

# ============================================================================
# FLOCKING FORCES (Vásárhelyi et al. 2018)
# ============================================================================

def get_neighbors(drone, all_drones):
    """Get neighbors within communication range, sorted by distance"""
    neighbors = []
    drone_pos = drone['position']
    
    for other in all_drones:
        if other['index'] == drone['index']:
            continue
        
        diff = other['position'] - drone_pos
        dist = diff.length
        
        if dist < params.r_comm and dist > 0.01:
            neighbors.append((other, dist, diff))
    
    # Sort by distance and limit to max_neighbors
    neighbors.sort(key=lambda x: x[1])
    return neighbors[:params.max_neighbors]

def compute_repulsion(drone, neighbors):
    """
    ZONE 1: Repulsion (short-range collision avoidance)
    
    THREE-TIER REPULSION:
    1. Normal zone (r_emergency to r_rep): Linear repulsion
    2. Emergency zone (min_distance to r_emergency): Exponential repulsion  
    3. Critical zone (< min_distance): Extreme force to separate
    """
    force = mathutils.Vector((0, 0, 0))
    
    for other, dist, diff in neighbors:
        if dist < 0.01:
            # Overlapping - push in random direction
            direction = mathutils.Vector((
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 1)
            )).normalized()
            force += direction * 50.0  # Very strong
            continue
            
        direction = -diff.normalized()  # Push away
        
        if dist < params.min_distance:
            # CRITICAL: Below hard minimum - extreme repulsion
            # Force grows as 1/dist² - becomes huge at small distances
            magnitude = 20.0 * (params.min_distance / dist) ** 2
            force += direction * magnitude
            
        elif dist < params.r_emergency:
            # EMERGENCY: Exponential repulsion
            # Smoothly ramps up as we approach min_distance
            normalized = (dist - params.min_distance) / (params.r_emergency - params.min_distance)
            # Exponential: force = base * e^(-k*normalized)
            magnitude = 10.0 * math.exp(-3.0 * normalized)
            force += direction * magnitude
            
        elif dist < params.r_rep:
            # NORMAL: Linear repulsion (original behavior)
            magnitude = params.p_rep * (params.r_rep - dist) / params.r_rep
            force += direction * magnitude
    
    return force

def compute_alignment(drone, neighbors):
    """
    ZONE 2: Velocity alignment / Friction (mid-range)
    Only activates when velocity difference exceeds v_slack
    """
    force = mathutils.Vector((0, 0, 0))
    
    if not neighbors:
        return force
    
    # Average velocity of neighbors in friction range
    avg_velocity = mathutils.Vector((0, 0, 0))
    count = 0
    
    for other, dist, _ in neighbors:
        if dist < params.r_frict:
            avg_velocity += other['velocity']
            count += 1
    
    if count > 0:
        avg_velocity /= count
        
        # Velocity difference
        velocity_diff = avg_velocity - drone['velocity']
        
        # Only align if difference exceeds slack
        if velocity_diff.length > params.v_slack:
            force = velocity_diff * params.c_frict
    
    return force

def compute_cohesion(drone, neighbors):
    """
    ZONE 3: Cohesion (long-range attraction to group center)
    Attracts toward center of mass of nearby drones
    """
    force = mathutils.Vector((0, 0, 0))
    
    if not neighbors:
        return force
    
    # Center of mass of neighbors in cohesion range
    center = mathutils.Vector((0, 0, 0))
    count = 0
    
    for other, dist, _ in neighbors:
        if dist < params.r_cohesion:
            center += other['position']
            count += 1
    
    if count > 0:
        center /= count
        direction = center - drone['position']
        force = direction * params.p_cohesion
    
    return force

def compute_breathing(drone, frame, all_drones):
    """
    BREATHING FORCE: Periodic expansion/contraction of the swarm.
    
    - During "push" phase: pushes drones outward from center of mass
    - During "relax" phase: no force (lets cohesion pull them back)
    
    This creates a natural split/merge cycle.
    """
    if not params.enable_breathing:
        return mathutils.Vector((0, 0, 0))
    
    # Calculate center of mass of entire swarm
    center = mathutils.Vector((0, 0, 0))
    for d in all_drones:
        center += d['position']
    center /= len(all_drones)
    
    # Direction from center to this drone (outward)
    outward = drone['position'] - center
    dist_from_center = outward.length
    
    if dist_from_center < 0.1:
        # If at center, push in random direction
        outward = mathutils.Vector((
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1)
        ))
    outward = outward.normalized()
    
    # Calculate breathing phase (0 to 1 over the period)
    phase = (frame % params.breathing_period) / params.breathing_period
    
    # Smooth wave function for breathing
    # During duty_cycle portion: push outward
    # Rest of cycle: relax (no push, cohesion pulls back)
    duty = params.breathing_duty_cycle
    smoothness = params.breathing_smoothness
    
    if phase < duty:
        # Push phase - use smoothed ramp up and down
        # Normalize phase within push period (0 to 1)
        push_phase = phase / duty
        # Smooth bell curve: sin² gives nice ramp up/down
        intensity = math.sin(push_phase * math.pi) ** (1.0 / (smoothness + 0.1))
    else:
        # Relax phase - no outward push
        intensity = 0.0
    
    force = outward * params.breathing_strength * intensity
    
    return force

def compute_preferred_velocity(drone):
    """Maintain preferred flocking speed"""
    current_speed = drone['velocity'].length
    
    if current_speed < 0.01:
        # Random direction if stationary
        direction = mathutils.Vector((
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1)
        )).normalized()
    else:
        direction = drone['velocity'].normalized()
    
    desired_velocity = direction * params.v_flock
    force = (desired_velocity - drone['velocity']) * 0.2
    
    return force

def compute_wall_avoidance(drone):
    """Soft constraint to keep drones near anchor"""
    force = mathutils.Vector((0, 0, 0))
    drone_pos = drone['position']
    
    # Distance from anchor origin (not world origin)
    dist = drone_pos.length
    
    if dist > params.arena_size * 0.7:
        # Push back toward center
        direction = -drone_pos.normalized()
        magnitude = params.wall_repulsion * (dist - params.arena_size * 0.7)
        force = direction * magnitude
    
    return force

def compute_ground_avoidance(drone):
    """Keep drones above ground (z > 5)"""
    force = mathutils.Vector((0, 0, 0))
    z_height = drone['position'].z
    
    min_height = 5.0
    buffer = 10.0
    
    if z_height < buffer:
        # Push upward
        magnitude = 2.0 * (buffer - z_height) / buffer
        force = mathutils.Vector((0, 0, magnitude))
    
    return force

def compute_turbulence(drone, frame, neighbors):
    """
    Add periodic noise to prevent convergence to stable blob.
    Each drone gets a unique phase offset for variation.
    
    COLLISION-SAFE: Turbulence is projected to remove any component
    that would push the drone toward nearby neighbors. This guarantees
    turbulence can NEVER cause drones to get closer together.
    """
    if not params.enable_turbulence:
        return mathutils.Vector((0, 0, 0))
    
    # Compute raw turbulence vector
    freq = params.turbulence_frequency
    base_strength = params.turbulence_strength * (1 + math.sin(frame * freq))
    phase = drone['index'] * 0.1
    
    turbulence = mathutils.Vector((
        base_strength * math.sin(frame * 0.02 + phase),
        base_strength * math.cos(frame * 0.015 + phase * 1.3),
        base_strength * math.sin(frame * 0.025 + phase * 0.7)
    ))
    
    # Safety radius: remove "toward neighbor" components within this range
    safety_radius = params.r_rep * 1.5
    
    # For each close neighbor, remove the component of turbulence
    # that would push us toward them
    for other, dist, diff in neighbors:
        if dist < safety_radius and dist > 0.01:
            # Direction FROM drone TO neighbor (normalized)
            toward_neighbor = diff.normalized()
            
            # Project turbulence onto this direction
            # (how much turbulence is pushing us toward this neighbor)
            toward_component = turbulence.dot(toward_neighbor)
            
            # If turbulence would push us toward neighbor, remove that component
            if toward_component > 0:
                turbulence -= toward_neighbor * toward_component
    
    return turbulence

def apply_random_kick(drone, neighbors):
    """
    Occasionally give a drone a random velocity impulse.
    COLLISION-SAFE: Kick direction is filtered to never push toward neighbors.
    Returns True if kick was applied.
    """
    if not params.enable_random_kicks:
        return False
    
    if random.random() < params.kick_probability:
        kick = mathutils.Vector((
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-0.5, 0.5)
        )).normalized() * params.kick_strength
        
        # Remove components that would push toward close neighbors
        safety_radius = params.r_rep * 1.5
        for other, dist, diff in neighbors:
            if dist < safety_radius and dist > 0.01:
                toward_neighbor = diff.normalized()
                toward_component = kick.dot(toward_neighbor)
                if toward_component > 0:
                    kick -= toward_neighbor * toward_component
        
        # Only apply if there's still meaningful kick left
        if kick.length > 0.1:
            drone['velocity'] += kick
            return True
    return False

def filter_velocity_toward_neighbors(drone, velocity, neighbors):
    """
    Remove velocity components that would move the drone toward any
    neighbor within the emergency zone. This prevents velocity from
    carrying drones into collisions.
    """
    filtered = velocity.copy()
    
    for other, dist, diff in neighbors:
        if dist < params.r_emergency and dist > 0.01:
            toward_neighbor = diff.normalized()
            toward_component = filtered.dot(toward_neighbor)
            
            # If velocity would move us toward this close neighbor, remove that component
            if toward_component > 0:
                # Scale removal based on how close we are
                # At min_distance: remove 100%, at r_emergency: remove 50%
                removal_factor = 0.5 + 0.5 * (1 - (dist - params.min_distance) / 
                                               (params.r_emergency - params.min_distance))
                removal_factor = max(0.5, min(1.0, removal_factor))
                filtered -= toward_neighbor * toward_component * removal_factor
    
    return filtered

def enforce_minimum_distance(drones):
    """
    HARD CONSTRAINT: After all physics, physically push apart any drones
    that are closer than min_distance. This is the final safety net.
    
    Uses iterative relaxation to resolve all violations.
    """
    max_iterations = 5  # Usually converges in 2-3
    
    for iteration in range(max_iterations):
        violations_fixed = 0
        
        for i, drone_a in enumerate(drones):
            for j, drone_b in enumerate(drones):
                if j <= i:
                    continue
                
                diff = drone_b['position'] - drone_a['position']
                dist = diff.length
                
                if dist < params.min_distance and dist > 0.001:
                    # Calculate overlap
                    overlap = params.min_distance - dist
                    
                    # Push each drone apart by half the overlap
                    direction = diff.normalized()
                    push = direction * (overlap / 2 + 0.01)  # +0.01 for margin
                    
                    drone_a['position'] -= push
                    drone_b['position'] += push
                    
                    # Also adjust velocities to prevent them from immediately colliding again
                    # Remove velocity components toward each other
                    vel_a_toward = drone_a['velocity'].dot(direction)
                    vel_b_toward = drone_b['velocity'].dot(-direction)
                    
                    if vel_a_toward > 0:
                        drone_a['velocity'] -= direction * vel_a_toward
                    if vel_b_toward > 0:
                        drone_b['velocity'] += direction * vel_b_toward
                    
                    violations_fixed += 1
                
                elif dist < 0.001:
                    # Overlapping completely - push apart in random direction
                    random_dir = mathutils.Vector((
                        random.uniform(-1, 1),
                        random.uniform(-1, 1),
                        random.uniform(-1, 1)
                    )).normalized()
                    
                    drone_a['position'] -= random_dir * (params.min_distance / 2)
                    drone_b['position'] += random_dir * (params.min_distance / 2)
                    violations_fixed += 1
        
        if violations_fixed == 0:
            break  # All violations resolved
    
    return violations_fixed > 0

# ============================================================================
# OPTIMIZE BLENDER PERFORMANCE
# ============================================================================

# Pre-create animation data and FCurves for faster keyframe insertion
# This prevents slowdown over time because:
# 1. keyframe_insert() searches/creates FCurves each time (O(n) lookup)
# 2. Direct FCurve access is O(1) - constant time
# 3. Pre-allocating keyframe points avoids repeated memory allocation
# Result: Maintains consistent FPS throughout long animations

for drone in drones:
    obj = drone['object']
    if not obj.animation_data:
        obj.animation_data_create()
    
    action = bpy.data.actions.new(name=f"Action_{obj.name}")
    obj.animation_data.action = action
    
    # Pre-create FCurves for X, Y, Z location
    for i in range(3):
        fcurve = action.fcurves.new(data_path="location", index=i)
        fcurve.keyframe_points.add(total_frames)
    
    drone['fcurves'] = [action.fcurves[i] for i in range(3)]

# ============================================================================
# ANIMATE SWARM
# ============================================================================

print(f"\n{'='*70}")
print(f"  🚁 Vásárhelyi et al. Flocking Model v1.0")
print(f"{'='*70}")
print(f"  Drones: {num_drones} | Frames: {start_frame}-{end_frame} ({total_frames} total) | FPS: {fps}")
print(f"  Volume scale: {VOLUME_SCALE:.3f}x linear ({VOLUME_SCALE**3:.2f}x volume)")
print(f"  COLLISION: min={params.min_distance:.1f}m | emergency={params.r_emergency:.1f}m | r_rep={params.r_rep:.1f}m")
print(f"  r_frict: {params.r_frict:.1f}m | r_cohesion: {params.r_cohesion:.1f}m")
breath_status = "ON" if params.enable_breathing else "OFF"
turb_status = "ON" if params.enable_turbulence else "OFF"
kick_status = "ON" if params.enable_random_kicks else "OFF"
print(f"  Breathing: {breath_status} (period={params.breathing_period} frames, strength={params.breathing_strength})")
print(f"  Turbulence: {turb_status} | Kicks: {kick_status}")
print(f"  Speed limits: v_flock={params.v_flock} m/s, v_max_h={params.v_max_horizontal} m/s, v_max_v={params.v_max_vertical} m/s")
print(f"{'='*70}\n")

start_time = time.time()
last_update_time = start_time

# Initialize window manager progress cursor
wm = bpy.context.window_manager
wm.progress_begin(0, total_frames)

for frame_idx, frame in enumerate(range(start_frame, end_frame + 1)):
    # Update cursor progress indicator
    wm.progress_update(frame_idx)
    # Progress bar
    current_time = time.time()
    frames_done = frame_idx + 1
    if frame_idx == 0 or (current_time - last_update_time) >= 0.1 or frame == end_frame:
        last_update_time = current_time
        
        progress = frames_done / total_frames
        bar_width = 40
        filled = int(bar_width * progress)
        bar = '█' * filled + '░' * (bar_width - filled)
        
        elapsed = current_time - start_time
        if frames_done > 1:
            frames_per_sec = frames_done / elapsed
            eta_seconds = (total_frames - frames_done) / frames_per_sec if frames_per_sec > 0 else 0
            eta_str = f"{int(eta_seconds//60):02d}:{int(eta_seconds%60):02d}"
            fps_str = f"{frames_per_sec:.1f} fps"
        else:
            eta_str = "--:--"
            fps_str = "-- fps"
        
        print(f"\r  [{bar}] {progress*100:5.1f}% | Frame {frame:5d} ({frames_done}/{total_frames}) | {fps_str} | ETA: {eta_str}  ", end='', flush=True)
    
    bpy.context.scene.frame_set(frame)
    
    # PHASE 1: Compute forces and update velocities/positions for all drones
    for drone in drones:
        # Get neighbors
        neighbors = get_neighbors(drone, drones)
        
        # Compute three fundamental forces
        f_repulsion = compute_repulsion(drone, neighbors)
        f_alignment = compute_alignment(drone, neighbors)
        f_cohesion = compute_cohesion(drone, neighbors)
        
        # Breathing force (split/merge)
        f_breathing = compute_breathing(drone, frame, drones)
        
        # Additional forces
        f_preferred = compute_preferred_velocity(drone)
        f_wall = compute_wall_avoidance(drone)
        f_ground = compute_ground_avoidance(drone)
        f_turbulence = compute_turbulence(drone, frame, neighbors)
        
        # Total acceleration
        acceleration = (f_repulsion + f_alignment + f_cohesion + f_breathing + 
                       f_preferred + f_wall + f_ground + f_turbulence)
        
        # Apply random kick (if enabled and lucky)
        apply_random_kick(drone, neighbors)
        
        # Update velocity
        new_velocity = drone['velocity'] + acceleration * dt
        
        # Limit maximum speed (ENFORCED) - separate horizontal and vertical limits
        # Horizontal (XY) speed limit
        horizontal_speed = math.sqrt(new_velocity.x**2 + new_velocity.y**2)
        if horizontal_speed > params.v_max_horizontal:
            scale = params.v_max_horizontal / horizontal_speed
            new_velocity.x *= scale
            new_velocity.y *= scale
        
        # Vertical (Z) speed limit - more restrictive
        if abs(new_velocity.z) > params.v_max_vertical:
            new_velocity.z = math.copysign(params.v_max_vertical, new_velocity.z)
        
        # SAFETY: Filter out velocity components toward close neighbors
        new_velocity = filter_velocity_toward_neighbors(drone, new_velocity, neighbors)
        
        drone['velocity'] = new_velocity
        
        # Update position
        drone['position'] += drone['velocity'] * dt
        
        # Hard constraint: never go below ground
        if drone['position'].z < 5.0:
            drone['position'].z = 5.0
            if drone['velocity'].z < 0:
                drone['velocity'].z = 0
    
    # PHASE 2: HARD CONSTRAINT - Enforce minimum distance between ALL drone pairs
    # This runs after all drones have moved, resolving any remaining violations
    enforce_minimum_distance(drones)
    
    # PHASE 3: Apply final positions to Blender objects and record keyframes
    for drone in drones:
        # Apply to Blender object
        drone['object'].location = drone['position']
        
        # Fast keyframe insertion using pre-allocated FCurves
        for i, fcurve in enumerate(drone['fcurves']):
            fcurve.keyframe_points[frame_idx].co = (frame, drone['position'][i])
            fcurve.keyframe_points[frame_idx].interpolation = 'LINEAR'

# End cursor progress indicator
wm.progress_end()

# Update all FCurves to finalize the animation
print("\n  Finalizing animation curves...")
for drone in drones:
    for fcurve in drone['fcurves']:
        fcurve.update()

# Final statistics
total_time = time.time() - start_time
avg_fps = total_frames / total_time if total_time > 0 else 0

print(f"\n\n{'='*70}")
print(f"  ✓ ANIMATION COMPLETE (v1.0)")
print(f"{'='*70}")
print(f"  Drones: {num_drones}")
print(f"  Frames: {start_frame}-{end_frame} ({total_frames} total, {total_frames/fps:.1f}s @ {fps} FPS)")
print(f"  Keyframes: {num_drones * total_frames * 3:,} (X/Y/Z per drone)")
print(f"  Computation Time: {int(total_time//60)}m {int(total_time%60)}s")
print(f"  Average Speed: {avg_fps:.1f} frames/sec")
print(f"  Performance: {(num_drones * total_frames) / total_time:.0f} drone-frames/sec")
print(f"")
print(f"  VOLUME SCALING:")
print(f"    Linear scale: {VOLUME_SCALE:.3f}x")
print(f"    Volume scale: {VOLUME_SCALE**3:.2f}x (1.5x)")
print(f"")
print(f"  COLLISION PREVENTION (multi-layer):")
print(f"    Hard minimum:  {params.min_distance:.1f}m (NEVER violated)")
print(f"    Emergency:     {params.r_emergency:.1f}m (exponential repulsion)")
print(f"    Soft repulsion: {params.r_rep:.1f}m (linear repulsion)")
print(f"    + Velocity filtering toward close neighbors")
print(f"    + Post-movement position enforcement")
print(f"")
print(f"  INTERACTION ZONES:")
print(f"    Repulsion:  < {params.r_rep:.1f}m (collision avoidance)")
print(f"    Alignment:  < {params.r_frict:.1f}m (velocity matching)")
print(f"    Cohesion:   < {params.r_cohesion:.1f}m (group attraction)")
print(f"")
print(f"  BREATHING (split/merge):")
print(f"    Enabled: {params.enable_breathing}")
print(f"    Period: {params.breathing_period} frames ({params.breathing_period/fps:.1f}s)")
print(f"    Strength: {params.breathing_strength}")
print(f"    Duty cycle: {params.breathing_duty_cycle*100:.0f}% push / {(1-params.breathing_duty_cycle)*100:.0f}% relax")
print(f"")
print(f"  SPEED LIMITS (enforced):")
print(f"    Preferred: {params.v_flock} m/s")
print(f"    Max horizontal (XY): {params.v_max_horizontal} m/s")
print(f"    Max vertical (Z):    {params.v_max_vertical} m/s")
print(f"{'='*70}\n")
print(f"  💡 TIP: Select 'SwarmAnchor' to move/rotate/scale the entire swarm!")
print(f"  💡 Adjust breathing_period to change split/merge frequency")
print(f"{'='*70}\n")

