# Road_Width_Detector

Python function to extract road widths from aerial imagery.

## Width Detector and Coloring

An application for analyzing road widths in aerial imagery, generating road outlines, masks with widths, and color-enhanced images.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Requirements]
- [Getting Started](#getting-started)
- [Config File](#Config File)
- [Usage](#usage)

## Overview

The Width Detector and Coloring project are designed to analyze road widths in aerial imagery using a provided shapefile of roads. The program takes a TIFF format aerial image and the corresponding shapefile as inputs and outputs a set of processed images, including road outlines, masks with widths, the original TIFF image, and a new TIFF image where roads are colored to maintain similar road coloring.

The initial need for this program was to address challenges such as removing parked cars from the roads and eliminating trees that intersect with the roads.

## Features

- **Road Width Analysis:** Utilizes the provided shapefile to analyze and determine road widths in the aerial imagery.
- **Output Images:**
  - **Road Outlines:** kind of edge detection.
  - **Road Masks with Widths:** Masks highlighting the roads with their corresponding widths.
  - **Original TIFF Image:** The unaltered aerial imagery.
  - **Colored Roads TIFF Image:** A new TIFF image with roads colored to maintain similar road coloring.

## Requirements
  you have to run the following command to install the required libraries
   ```bash
  pip install -r requirements.txt
  '''
## Getting Started

Before running the program, a config file must be set.

1. Run a single image using the following command:

    ```bash
    python adaptive_road_width_lib.py "your_aerial_image.tiff" "shapefile.shp"
    ```

2. Run a folder of images using the following command (config file must be defined):

    ```bash
    python main.py
    ```

## Config File

The config file is written in a JSON file.

```json
{
  "ROADS_SHAPEFILE_FILE": "C:\\Users\\Simlat\\Desktop\\simlat lior\\material solver\\material solver data\\Vectors\\Roads\\alaro_roads.shp",
  "IMAGES_DIR": "C:\\Users\\Simlat\\Desktop\\simlat lior\\material solver\\material solver data\\Imagery\\10",
  "OUTPUT_FOLDER": "C:\\Users\\Simlat\\Desktop\\simlat lior\\width_road_detector\\road_width_output",

  "PIXEL_DISTANCE_TRESH" : 1, // Max distance for pixel color to be considered a road pixel
  "PREPENDICULAR_LENGTH" : 1.5, // Defined length of perpendicular lines, perpendicular to the roads' polylines, for pixel sampling
  "SPACES_BETWEEN_PREPENDICULAR" : 0.25, // Distance between two perpendicular lines in meters.
  "ROAD_SORT_SIZE": 1, // An integer to control the length of roads to be analyzed. For instance, if the variable is one, the polylines will be separated into different polylines, each one meter in length.
  "PROCESS_BATCH_SIZE": 10 // Number of roads to be processed together in the same batch.
}



