#!/usr/bin/env python3
"""
Generate an empty occupancy grid map for Nav2.
Creates a 20m x 20m map at 0.05m resolution (400x400 pixels).
All cells are free space.
"""

from PIL import Image
import os

# Map size: 
size = 2000  # 2000 pixels × 0.05 m/pixel = 100 m × 100 m
resolution = 0.05

# Create a white image (255 = free in trinary mode with free_thresh=0.196)
img = Image.new('L', (size, size), color=254)  # 254 = clearly free

# Save PGM
output_dir = os.path.dirname(os.path.abspath(__file__))
pgm_path = os.path.join(output_dir, 'empty_map.pgm')
img.save(pgm_path)
print(f'Saved PGM: {pgm_path}')

# Save YAML
yaml_content = f"""image: empty_map.pgm
resolution: {resolution}
origin: [-{size * resolution / 2}, -{size * resolution / 2}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary
"""

yaml_path = os.path.join(output_dir, 'empty_map.yaml')
with open(yaml_path, 'w') as f:
    f.write(yaml_content)
print(f'Saved YAML: {yaml_path}')
half = size * resolution / 2
print(f'Map covers x: [-{half}, {half}], y: [-{half}, {half}] meters')
