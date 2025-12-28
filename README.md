# Murmur

A swarm flocking simulation for Blender and Skybrush Studio, based on the Vásárhelyi et al. 2018 optimized flocking model.

https://github.com/user-attachments/assets/5a9eec63-972c-48b6-b54c-4bcb24765280


## ⚠️ Disclaimer

**THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.**

**YOU ARE SOLELY RESPONSIBLE FOR VALIDATING ALL TRAJECTORIES, SPEEDS, AND BEHAVIORS PRODUCED BY THIS SOFTWARE BEFORE FLYING ANY DRONE SHOWS OR OPERATING ANY REAL AIRCRAFT.**

The authors and contributors of this software accept **NO LIABILITY** for any damages, injuries, property damage, or other consequences arising from the use of this software or the trajectories it generates. Always:

- Validate all generated flight paths against your specific hardware limitations
- Verify speed limits match your aircraft capabilities
- Test in simulation before any real-world deployment
- Comply with all local aviation regulations and safety requirements
- Maintain appropriate safety margins and fail-safes

---

## Overview

Murmur implements the optimized flocking model from:

> Vásárhelyi, G., Virágh, C., Somorjai, G., Nepusz, T., Eiben, A. E., & Vicsek, T. (2018).  
> "Optimized flocking of autonomous drones in confined environments."  
> *Science Robotics*, 3(20), eaat3536.  
> https://hal.elte.hu/~vasarhelyi/doc/vasarhelyi2018optimized.pdf

The model creates realistic, collision-free swarm behavior through a combination of local interaction rules that each drone follows independently.

## Method

### Three-Zone Interaction Model

Each drone interacts with its neighbors through three fundamental forces based on distance:

#### 1. Repulsion (Short-range: < `r_rep`)
Prevents collisions through a three-tier system:
- **Critical zone** (< `min_distance`): Extreme 1/d² repulsion force
- **Emergency zone** (< `r_emergency`): Exponential repulsion
- **Normal zone** (< `r_rep`): Linear repulsion

#### 2. Velocity Alignment / Friction (Mid-range: < `r_frict`)
Synchronizes velocities with nearby neighbors to create smooth, coherent group movement. Only activates when the velocity difference exceeds `v_slack`, preventing unnecessary corrections.

#### 3. Cohesion (Long-range: < `r_cohesion`)
Attracts drones toward the center of mass of their neighbors, preventing group fragmentation. This is a weaker force that acts over larger distances.

### Additional Behaviors

#### Breathing (Split/Merge)
A periodic radial oscillation that pushes drones outward from the swarm's center of mass during the "push" phase, then relaxes to let cohesion pull them back together. This creates organic split/merge dynamics.

#### Anti-Convergence Mechanisms
To prevent the swarm from settling into a static blob:
- **Turbulence**: Periodic noise added to each drone's movement (collision-safe: filtered to never push toward neighbors)
- **Random Kicks**: Occasional velocity impulses (also collision-safe)

### Multi-Layer Collision Prevention

The system uses multiple safeguards to prevent collisions:
1. Three-tier repulsion forces (critical/emergency/normal zones)
2. Velocity filtering: removes velocity components that would move a drone toward close neighbors
3. Post-movement position enforcement: iteratively pushes apart any drones that end up too close

## Installation

### 1. Add Project Files

Place `murmur.py` and `config.toml` in the same directory (or in the same directory as your `.blend` file).

No external dependencies are required—the script uses Python's built-in `tomllib` module (Python 3.11+, included with Blender 4.0+).

### 2. Run the Script

Open `murmur.py` in Blender's Text Editor and run the script (Alt+P or the Run button).

### Remote Execution (Optional)

The `util/` directory contains optional utilities for executing scripts in Blender remotely from your terminal or IDE. This is useful for faster iteration without switching windows.

#### How It Works

1. **`util/remote_bpy.py`** — A server that runs inside Blender. It listens on `localhost:5678` for incoming Python code and executes it on Blender's main thread. Output (print statements, errors) is streamed back to the client in real-time.

2. **`util/send_to_blender.py`** — A client script you run from your terminal. It reads a Python file, sends its contents to the remote_bpy server, and streams the output back to your terminal.

#### Setup

1. Open `util/remote_bpy.py` in Blender's Text Editor and run it once. You'll see:
   ```
   [remote_bpy] listening on 127.0.0.1:5678
   [remote_bpy] streaming server started
   ```

2. From your terminal, send scripts to Blender:
   ```bash
   python3 util/send_to_blender.py murmur.py
   ```

3. The script executes in Blender and output streams back to your terminal.

#### Notes

- The server persists a namespace between executions, so you can define helper functions once and reuse them.
- The server runs on a background thread but executes code on Blender's main thread via `bpy.app.timers`, ensuring thread-safety with Blender's API.
- You only need to run `remote_bpy.py` once per Blender session.
- This is entirely optional—you can always run scripts directly in Blender's Text Editor.

## Configuration

All parameters are configured in `config.toml`. The configuration is organized into sections:

### Volume Scaling

```toml
volume_scale_factor = 1.5
```

Scales all distance-based parameters to adjust the overall swarm volume. A factor of 1.5 means 1.5× the volume, which translates to distances being multiplied by 1.5^(1/3) ≈ 1.145.

---

### Flocking Parameters

```toml
[flocking]
min_distance = 1.0      # Hard minimum distance (meters) - NEVER violated
r_rep = 3.0             # Repulsion range (soft zone)
p_rep = 1.0             # Repulsion force gain
r_emergency = 1.5       # Emergency zone with exponential repulsion
r_frict = 6.0           # Velocity alignment range
c_frict = 0.5           # Friction/alignment coefficient
v_slack = 0.5           # Velocity difference threshold before alignment kicks in
r_cohesion = 15.0       # Cohesion attraction range
p_cohesion = 0.5        # Cohesion force gain
v_flock = 2.5           # Preferred flocking speed (m/s)
v_max_horizontal = 3.7  # Maximum horizontal (XY) speed (m/s)
v_max_vertical = 1.2    # Maximum vertical (Z) speed (m/s)
r_comm = 50.0           # Communication/perception range
max_neighbors = 10      # Maximum neighbors each drone considers
arena_size = 30.0       # Soft boundary radius
wall_repulsion = 1.0    # Boundary avoidance strength
```

| Parameter | Description | Scaled by Volume |
|-----------|-------------|------------------|
| `min_distance` | Absolute minimum distance between any two drones. This is a hard constraint that is never violated. | No |
| `r_rep` | Radius of the soft repulsion zone. Drones within this distance experience linear repulsion. | Yes |
| `p_rep` | Gain/strength of the repulsion force. | No |
| `r_emergency` | Radius of the emergency zone. Drones closer than this experience exponential repulsion. | No |
| `r_frict` | Radius within which drones align their velocities. | Yes |
| `c_frict` | How strongly drones match their neighbors' velocities. | No |
| `v_slack` | Minimum velocity difference before alignment activates. Prevents jitter. | No |
| `r_cohesion` | Radius within which drones are attracted to their neighbors' center of mass. | Yes |
| `p_cohesion` | Strength of the cohesion attraction. | No |
| `v_flock` | Target cruising speed for the swarm. | No |
| `v_max_horizontal` | Maximum speed in the XY plane. | No |
| `v_max_vertical` | Maximum speed in Z (typically more restricted for real drones). | No |
| `r_comm` | Maximum distance at which drones can "see" each other. | Yes |
| `max_neighbors` | Limits computation by only considering the N closest neighbors. | No |
| `arena_size` | Soft boundary—drones beyond 70% of this radius are pushed back. | Yes |
| `wall_repulsion` | Strength of the boundary avoidance force. | No |

---

### Breathing (Split/Merge Behavior)

```toml
[breathing]
enable = true
period = 200        # Frames per full cycle
strength = 2.5      # Outward push strength
duty_cycle = 0.35   # Fraction of cycle spent pushing out
smoothness = 0.3    # Transition smoothness (0=sharp, 1=very smooth)
```

| Parameter | Description |
|-----------|-------------|
| `enable` | Toggle breathing behavior on/off. |
| `period` | Number of frames for one complete push/relax cycle. At 25 FPS, 200 frames = 8 seconds. |
| `strength` | How strongly drones are pushed outward during the push phase. |
| `duty_cycle` | Fraction of the cycle spent actively pushing outward (0.35 = 35% push, 65% relax). |
| `smoothness` | Controls how sharply the push ramps up/down. Lower = more abrupt, higher = gentler. |

---

### Turbulence

```toml
[turbulence]
enable = true
strength = 1.5      # Base turbulence magnitude
frequency = 0.003   # Oscillation frequency
```

| Parameter | Description |
|-----------|-------------|
| `enable` | Toggle turbulence on/off. |
| `strength` | Magnitude of the turbulent forces. |
| `frequency` | How rapidly the turbulence pattern oscillates. Higher = faster changes. |

---

### Random Kicks

```toml
[random_kicks]
enable = true
probability = 0.005  # Chance per drone per frame
strength = 1.5       # Kick magnitude
```

| Parameter | Description |
|-----------|-------------|
| `enable` | Toggle random kicks on/off. |
| `probability` | Probability that any given drone receives a kick on any given frame. |
| `strength` | Magnitude of the velocity impulse. |

---

### Animation Settings

```toml
[animation]
num_drones = 100
fps = 25
start_frame = 1000
end_frame = 20000
```

| Parameter | Description |
|-----------|-------------|
| `num_drones` | Number of drones in the swarm. |
| `fps` | Frames per second (affects physics timestep). |
| `start_frame` | First frame of the animation. |
| `end_frame` | Last frame of the animation. |

---

### Drone Appearance

```toml
[drone]
size = 0.15         # Sphere radius for mesh drones
use_empties = false # Use empties instead of mesh spheres
```

| Parameter | Description |
|-----------|-------------|
| `size` | Radius of the sphere mesh representing each drone. |
| `use_empties` | If `true`, uses Blender empties instead of mesh objects (faster for large swarms). |

---

### Spawn Settings

```toml
spawn_radius = 15.0   # Initial spawn area radius
```

| Parameter | Description | Scaled by Volume |
|-----------|-------------|------------------|
| `spawn_radius` | Radius of the sphere in which drones are initially distributed (Fibonacci sphere pattern). | Yes |

## Using with Skybrush Studio

To use the generated animation with [Skybrush Studio](https://skybrush.io/) for drone show planning:

1. **Run the script** to generate the swarm animation

2. **Select the drone collection hierarchy:**
   - In the Outliner, expand the `Swarm_Drones` collection
   - Select all drone objects within the collection (you can click the first drone, then Shift+click the last to select all)

3. **Create a Skybrush formation:**
   - Open the Skybrush Studio panel
   - Go to the **Formations** section
   - Click the **Create Formation (+)** button
   - Choose **Append** to add the selected objects as a new formation

4. **Verify the import:**
   - The drone positions and keyframed animation should now be available in Skybrush
   - Use Skybrush's validation tools to check for any trajectory or speed limit violations

> **Important:** Always validate the generated trajectories against your specific drone hardware capabilities before exporting for a real show. See the disclaimer at the top of this document.

## Usage Tips

- Select the `SwarmAnchor` object to move, rotate, or scale the entire swarm
- Adjust `breathing.period` to change the frequency of split/merge behavior
- Reduce `num_drones` or use `use_empties: true` for faster viewport performance
- Increase `min_distance` if your real drones have larger safety requirements
- The `v_max_vertical` is typically set lower than horizontal to match real drone capabilities

## License

See LICENSE file for details.

