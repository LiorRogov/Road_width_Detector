from glob import glob
import json
import os
import subprocess
from tqdm import tqdm

f = open('config.json')
config = json.load(f)


IMAGES_FOLDER = config['IMAGES_DIR']
SHAPEFILE_PATH = config['ROADS_SHAPEFILE_FILE']
OUTPUT_FOLDER = config['OUTPUT_FOLDER']

images_path_list = sorted(glob(os.path.join(IMAGES_FOLDER, '*.tif')))

for images_path in images_path_list:
    if not (os.path.exists(os.path.join(OUTPUT_FOLDER, os.path.basename(images_path)))):
        #analyze every road's width in image stared in 'image_path' path
        subprocess.run(['python', 'adaptive_road_width_lib.py', f'{images_path}', f'{SHAPEFILE_PATH}'])



