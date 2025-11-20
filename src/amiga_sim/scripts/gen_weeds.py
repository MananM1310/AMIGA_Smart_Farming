#!/usr/bin/env python
import os
import random
import json

# Define weed layout
rows, cols = 20, 10
row_spacing = 1.18    # space between rows (Y-direction) 22 inches
col_spacing = 1.87    # space between columns (X-direction) 35 inches
quadrants = ["top_left", "top_right", "bottom_left", "bottom_right"]

WEED_SCALE_THRESHOLD = 0.3  # <= this is a "weed", otherwise "plant"

def get_signs_for_quadrant(q):
    return {
        "top_left": (-1, 1),
        "top_right": (1, 1),
        "bottom_left": (-1, -1),
        "bottom_right": (1, -1)
    }[q]

sdf = '<?xml version="1.0" ?>\n<sdf version="1.6">\n  <model name="farm_weeds">\n    <static>true</static>\n'

# Collect only true weeds (small ones) for JSON export
weed_list = []

for q in quadrants:
    x_sign, y_sign = get_signs_for_quadrant(q)
    for i in range(rows):
        for j in range(cols):
            x = (j + 1) * col_spacing * x_sign
            y = (i + 1) * row_spacing * y_sign
            name = f"{q}_{i}_{j}"
            scale = round(random.uniform(0.2, 0.7), 3)

            # SDF entry (plants + weeds, all together)
            sdf += f'''
    <link name="weed_{name}">
      <pose>{x} {y} 0 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://weed/meshes/weed.dae</uri>
            <scale>{scale} {scale} {scale}</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.1 0.5 0.1 1</ambient>
          <diffuse>0.1 0.6 0.1 1</diffuse>
        </material>
      </visual>
    </link>
'''

            # If it's small enough, treat it as a weed and record it
            if scale <= WEED_SCALE_THRESHOLD:
                weed_list.append({
                    "name": name,
                    "quadrant": q,
                    "row_index": i,
                    "col_index": j,
                    "x": x,
                    "y": y,
                    "scale": scale
                })

sdf += '  </model>\n</sdf>\n'

# Get absolute path to package directory
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_root = os.path.abspath(os.path.join(script_dir, ".."))  # amiga_sim/
model_dir = os.path.join(pkg_root, "models", "farm_weeds")
os.makedirs(model_dir, exist_ok=True)

# Write SDF
sdf_path = os.path.join(model_dir, "model.sdf")
with open(sdf_path, "w") as f:
    f.write(sdf)

# Write JSON with only real weeds (scale <= threshold)
weed_json_path = os.path.join(model_dir, "weed_positions.json")
with open(weed_json_path, "w") as f:
    json.dump(weed_list, f, indent=2)

print(f"✅ Generated: {sdf_path}")
print(f"✅ Generated weed JSON: {weed_json_path} (num_weeds={len(weed_list)})")
