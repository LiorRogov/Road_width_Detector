import rasterio.control
import rasterio.crs
import rasterio.sample
import rasterio.vrt
import rasterio._features

from geopy.distance import geodesic
from shapely.geometry import Point, LineString, shape, Polygon, box
import geopandas as gpd
import rasterio
from rasterio import features
import numpy as np
import cv2
import math
import multiprocessing
import os
from tqdm import tqdm
import glob
import pickle
import argparse
import json
import shutil
from rasterio.features import shapes
import shapely._geos
from shutil import copy
import tifffile
import numpy as np
import road_coloring_lib as paint
from PIL import Image
import pandas  as pd

import buldings_support





f = open('config.json')
# returns JSON object as 
# a dictionary
config = json.load(f)

IMAGES_DIR =  config['IMAGES_DIR']
OUTPUT_FOLDER = config['OUTPUT_FOLDER']

PIXEL_DISTANCE_TRESH, PREPENDICULAR_LENGTH, SPACES_BETWEEN_PREPENDICULAR = config['PIXEL_DISTANCE_TRESH'], config['PREPENDICULAR_LENGTH'], config['SPACES_BETWEEN_PREPENDICULAR']
ROAD_SORT_SIZE =  config['ROAD_SORT_SIZE']
PROCESS_BATCH_SIZE =  config['PROCESS_BATCH_SIZE']

BAD_GUESS_POLYGON_BUFFER = config['BAD_GUESS_POLYGON_BUFFER']
KERNEL_SIZE = tuple(config['KERNEL_SIZE'])

AVRAGE_DISTANCE_MAX = config['AVRAGE_DISTANCE_MAX']
AVRAGE_DISTANCE_MAX_DISTANCE_SUBSTITUTE = config['AVRAGE_DISTANCE_MAX_DISTANCE_SUBSTITUTE']
AVRAGE_DISTANCE_FAILIURE  = config['AVRAGE_DISTANCE_FAILIURE']



def draw_clean_roads_on_image(image, mask):
    return paint.create_image(image, mask)


def copy_geodata_to(source_tiff: str, destination_tiff:str):
    """
    Copies geospatial metadata, including the geotransformation matrix and coordinate reference system, from a source GeoTIFF image to a destination GeoTIFF image.

    Parameters:
        source_tiff (str): Path to the source GeoTIFF image.
        destination_tiff (str): Path to the destination GeoTIFF image.

    Returns:
        None
    """
    with rasterio.open(source_tiff) as src:
        t = src.transform
        c = src.crs
    
    with rasterio.open(destination_tiff, 'r+') as dst:
        dst.transform = t
        dst.crs = c

def create_intermidiate_points_between_true_points(PointA, PointB, distance):
    """
    Generates intermediate points along a geodesic line between two given points based on a specified distance.

    Parameters:
        PointA (Point): First point with 'x' and 'y' attributes representing coordinates.
        PointB (Point): Second point with 'x' and 'y' attributes representing coordinates.
        distance (float): Desired distance between consecutive intermediate points in meters.

    Returns:
        List[Point]: List of intermediate points along the geodesic line.
    """

    # Define the two original points as Point objects
    point1 = PointA  # Example coordinates within valid ranges
    point2 = PointB  # Example coordinates within valid ranges

    # Calculate the geodesic distance between the two points in meters
    distance_between_points = geodesic((point1.y, point1.x), (point2.y, point2.x)).meters

    # Create a LineString object representing the line between the two points
    line = LineString([point1, point2])

    if distance_between_points > distance:
        # Calculate the point at the specified fraction of the LineString
        intermediate_point = line.interpolate(0.5, normalized=True)

        # Recursively call the function on two subsegments of the line
        list_1 = create_intermidiate_points_between_true_points(point1, intermediate_point, distance)
        list_2 = create_intermidiate_points_between_true_points(intermediate_point, point2, distance)

        # Combine the intermediate points from both subsegments with the current intermediate point
        return (list_1 + [intermediate_point] + list_2)
    else:
        # If the distance between points is smaller than the specified distance, return an empty list
        return (list())
    


def make_list_from_polyline(polyline: LineString) -> list:
    """
    Converts a LineString polyline into a list of coordinates.

    Parameters:
        polyline (LineString): LineString object representing a polyline.

    Returns:
        List[tuple]: List of coordinate tuples (x, y) extracted from the LineString.
    """
    # Initialize an empty list to store the coordinates
    coordinates = []

    # Iterate through the coordinates in the polyline
    for i in range(0, len(polyline.coords)):
        # Append each coordinate to the list
        coordinates.append(polyline.coords[i])

    return coordinates

def create_appropriate_polyline(polyline):
    """
    Generates an appropriate LineString polyline with additional intermediate points.

    Parameters:
        polyline: LineString object representing the original polyline.

    Returns:
        LineString: Modified LineString with added intermediate points.
    """
    # Convert the input polyline to a list of coordinates
    poly_list = make_list_from_polyline(polyline)

    # Initialize a list to store intermediate points, starting with the first point
    intermediate_points = [Point(poly_list[0])]

    # Iterate through the list of coordinates
    for i in range(1, len(poly_list)):
        # Create intermediate points between the current point and the next point
        intermediate_points += create_intermidiate_points_between_true_points(Point(poly_list[i - 1]), Point(poly_list[i]))

        # Append the current point to the list of intermediate points
        intermediate_points.append(Point(poly_list[i]))

    # Create a LineString from the list of intermediate points
    return LineString(intermediate_points)


def calculate_distance_of_polyline(polyline):
    """
    Calculates the geodesic distance of a LineString polyline between its first and last points (the polyline contains two points only).

    Parameters:
        polyline: LineString object representing the polyline.

    Returns:
        float: Geodesic distance in meters between the first and last points of the polyline.
    """
    # Get the first and last points of the polyline
    point1 = Point(polyline.coords[0])
    point2 = Point(polyline.coords[-1])

    # Calculate the geodesic distance between the first and last points in meters
    distance = geodesic((point1.y, point1.x), (point2.y, point2.x)).meters

    return distance
    

def modify_polyline(original_line, distance):
    """
    Modifies a LineString by adjusting the distances between its original points.

    Parameters:
        original_line: LineString object representing the original polyline.
        distance: Desired distance between the original center point and the adjusted points.

    Returns:
        LineString: Modified LineString with adjusted points.
    """
    # Extract the original points from the input LineString
    pointA, center, pointB = original_line.coords[0], original_line.coords[1], original_line.coords[2]

    # Calculate the distance between the center and pointB
    distance_vector = calculate_distance_of_polyline(LineString([Point(center), Point(pointB)]))

    # Calculate the vector from center to pointB
    vector_center_to_pointB = np.array([pointB[0] - center[0], pointB[1] - center[1]])

    # Calculate the multiplication factor to adjust the new vector length
    multiplication_factor = distance * 0.5 / distance_vector
   

    # Calculate the new vector from center to the adjusted pointB
    new_vector_center_to_pointB = vector_center_to_pointB * multiplication_factor

    # Calculate the new pointB coordinates
    new_pointB = Point(center[0] + new_vector_center_to_pointB[0], center[1] + new_vector_center_to_pointB[1])

    # Calculate the vector from center to pointA
    vector_center_to_pointA = np.array([pointA[0] - center[0], pointA[1] - center[1]])

    # Calculate the new vector from center to the adjusted pointA
    new_vector_center_to_pointA = vector_center_to_pointA * multiplication_factor

    # Calculate the new pointA coordinates
    new_pointA = Point(center[0] + new_vector_center_to_pointA[0], center[1] + new_vector_center_to_pointA[1])

    # Create a new LineString with the adjusted points
    return LineString([new_pointA, Point(center), new_pointB])

   
def calculate_line_vector_direction(original_line : LineString):
    """
    Calculates the direction vector of a LineString.

    Parameters:
        original_line: LineString object.

    Returns:
        Point: Direction vector as a Point object.
    """
    # Calculate the direction vector by rotating the original line by 90 degrees
    direction_vector = Point(original_line.coords[1][1] - original_line.coords[0][1], original_line.coords[0][0] - original_line.coords[1][0])

    return direction_vector


def create_perpendicular_line(original_line, point_of_contact, direction_vector):
    """
    Creates a perpendicular LineString to the original line from a given point of contact.

    Parameters:
        original_line: LineString object representing the original line.
        point_of_contact: Point object representing the contact point for the perpendicular line.
        direction_vector: Point object representing the direction vector of the original line.

    Returns:
        LineString: Perpendicular LineString created from the point of contact.
    """

    closest_point = original_line.interpolate(original_line.project(point_of_contact))

    final_point = Point(closest_point.x + direction_vector.x, closest_point.y + direction_vector.y)
    initial_point = Point(closest_point.x - direction_vector.x, closest_point.y - direction_vector.y)

    #Create the perpendicular line by extending from the closest_point in both directions:
    perpendicular_line = LineString([initial_point, closest_point, final_point])
    
    return perpendicular_line

def get_pixels_intersects_with_polylines(src: rasterio.open, image: np.array, list_of_perpendicular_line: list) -> np.array:
    """
    Extracts pixel values from an image that intersect with given perpendicular lines.

    Parameters:
        src: Rasterio dataset object representing the source image.
        image: NumPy array representing the image.
        list_of_perpendicular_line: List of LineString objects representing perpendicular lines.

    Returns:
        numpy.ndarray: Array of pixel values that intersect with the perpendicular lines.
    """    
   
    # Load the polyline (GeoDataFrame or shapefile) and Reproject the polyline to the same CRS as the image
    polyline_gdf = gpd.GeoDataFrame(geometry= list_of_perpendicular_line, crs = src.crs)

    #polyline_gdf = polyline_gdf.to_crs(crs)

    # Rasterize the polyline into a binary mask
    shapes = [(geom, 1) for geom in polyline_gdf.geometry]
   
    # Create a masked array to obtain the pixel values that overlap with the polyline
    polyline_mask = features.geometry_mask(shapes, out_shape=(src.height, src.width), transform=src.transform, invert=True).astype('uint8') * 255

    # You can access the pixel values that overlap with the polyline using masked_image
    overlap_pixels = image[(polyline_mask != 0)]

    return overlap_pixels


def estimate_material(rgb_values, materials):
    """
    Estimates material based on squared Euclidean distances between pixel's RGB values and each material's RGB values.

    Parameters:
        rgb_values (numpy.ndarray): Unmapped pixel's RGB values as a numpy array [R, G, B].
        materials (numpy.ndarray): Array of material RGB values.

    Returns:
        numpy.ndarray: Squared Euclidean distances between pixel's RGB values and each material's RGB values.
    """

    # Calculate squared Euclidean distances between pixel's RGB values and each material's RGB values
    distRGB = (materials[:,0] - np.double(rgb_values[0]))**2 + (materials[:,1] - np.double(rgb_values[1]))**2 + (materials[:,2] - np.double(rgb_values[2]))**2
    
    # returns the squared Euclidean distances between the pixel's RGB values and each material's RGB values.
    return np.sqrt(distRGB)

def calculate_to_lines_to_center_line(intern_poly: list, distance: float, a_side: bool = True) -> LineString:
    """
    Calculates parallel lines to the center line of an internal polyline.

    Parameters:
        intern_poly (list): List of (longitude, latitude) coordinates representing the internal polyline.
        distance (float): Desired distance in meters for creating parallel lines.
        a_side (bool): If True, creates lines on the A-side; if False, creates lines on the B-side.

    Returns:
        LineString: LineString representing parallel lines to the center line.
    """
    # Convert the internal polyline to a LineString
    line_intern_poly = LineString(intern_poly)

    # Calculate the vector (vec) from the first point to the last point of the original polyline
    vec = Point(line_intern_poly.coords[-1][0] - line_intern_poly.coords[0][0], line_intern_poly.coords[-1][1] - line_intern_poly.coords[0][1])
    
    # Calculate the angle (m_angle_with_x_axis) of the vector with respect to the x-axis
    if (vec.coords[0][0] != 0):
        m_angle_with_x_axis = math.degrees(math.atan(vec.coords[0][1] / vec.coords[0][0]))
    else:
        m_angle_with_x_axis = 0
    
    # Original polyline as a list of (longitude, latitude) coordinates
    original_polyline = LineString(intern_poly)  

    # Specify the desired distance in meters
    distance_in_meters = distance  # Example: 1,000 meters

    # Create an empty list for the new polyline
    new_polyline = []

    # Iterate over the vertices of the original polyline
    for vertex in original_polyline.coords:
        longitude, latitude = vertex

        if a_side:
            # Calculate the destination point at the specified distance in the direction of m_angle_with_x_axis
            destination = geodesic(meters= distance_in_meters).destination((latitude, longitude), -m_angle_with_x_axis)  
        else:
            # Calculate the destination point at the specified distance in the opposite direction
            destination = geodesic(meters= distance_in_meters).destination((latitude, longitude), -m_angle_with_x_axis + 180)  

        # Add the new vertex to the new polyline
        new_vertex = (destination.longitude, destination.latitude)
        new_polyline.append(new_vertex)

    # Create a LineString from the new polyline vertices
    return (LineString(new_polyline))

def find_biggest_component(image, mask):
    """
    Finds and returns the largest connected component in a binary mask.

    Parameters:
        image (numpy.ndarray): Input image.
        mask (numpy.ndarray): Binary mask.

    Returns:
        numpy.ndarray: Mask of the largest connected component.
    """
    # Apply connected components labeling
    _ , labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    # Calculate the area of each connected component
    component_areas = stats[1:, cv2.CC_STAT_AREA]  # Exclude the background component

    #Find the index of the largest component (by area)
    largest_component_index = np.argmax(component_areas)

    # Create a mask for the largest connected component
    largest_component_mask = np.zeros(image.shape[:2])
    largest_component_mask[labels == largest_component_index + 1] = 255  # Set the largest component pixels to white (255)
    
    return (largest_component_mask.astype('uint8'))

def create_mask_of_straigh_road_from_road_mask(src: rasterio.open, road_mask: np.array, list_of_perpendicular_line: list) -> (float, float):
    """
    Creates a mask of a straight road based on a road mask and perpendicular lines.

    Parameters:
        src: Rasterio dataset object representing the source image.
        road_mask: Binary mask representing the road.
        list_of_perpendicular_line: List of LineString objects representing perpendicular lines.

    Returns:
        float: Average distance on the A-side perpendicular lines.
        float: Average distance on the B-side perpendicular lines.
    """
    prependicular_length = 100

    # Convert the mask array to polygons using rasterio's shapes function.
    shapes_ = shapes(road_mask, mask= (road_mask > 0), transform = src.transform)
    polygon = [shape(geometry) for geometry, _ in shapes_][0]
    
    
    for i in range(len(list_of_perpendicular_line)):
        perpendicular_line = list_of_perpendicular_line[i]
        list_of_perpendicular_line[i] = modify_polyline(perpendicular_line, prependicular_length)

    one_side_perpendicular_lines_lengths = []
    second_side_perpendicular_lines_lengths = []

    A_lines = []
    B_lines = []
    for i in range(len(list_of_perpendicular_line)):
        # Calculate the intersection of the manually drawn line with the polygon's exterior
        center_point = Point(list_of_perpendicular_line[i].coords[1])
        lineA , lineB = LineString(list_of_perpendicular_line[i].coords[:2]), LineString(list_of_perpendicular_line[i].coords[1:])

        if lineA.intersects(polygon.exterior) and lineB.intersects(polygon.exterior):
            intersection = polygon.exterior.intersection(lineA)
            if intersection.geom_type == 'Point':
                lineA = LineString([center_point, intersection])
                A_lines.append(lineA)
                one_side_perpendicular_lines_lengths.append(calculate_distance_of_polyline(lineA))

            elif intersection.geom_type == 'MultiPoint':
                #find the closest point to the center line
                min_distance = None
                min_point = None

                # Iterate over each line segment in the MultiLineString
                for point in intersection.geoms:
                    # Calculate the distance from the point to the line segment
                    distance = calculate_distance_of_polyline(LineString([point, center_point]))
                    
                    # Check if this is the maximum distance so far
                    if min_distance is None or distance < min_distance:
                        min_distance = distance
                        min_point = point
                
                lineA = LineString([min_point, center_point])
                A_lines.append(lineA)
                one_side_perpendicular_lines_lengths.append(calculate_distance_of_polyline(lineA))
            
            
            intersection = polygon.exterior.intersection(lineB)
            if intersection.geom_type == 'Point':
                lineB = LineString([center_point, intersection])
                B_lines.append(lineB)
                second_side_perpendicular_lines_lengths.append(calculate_distance_of_polyline(lineB))
            
            elif intersection.geom_type == 'MultiPoint':
                #find the closest point to the center line
                min_distance = None
                min_point = None

                # Iterate over each line segment in the MultiLineString
                for point in intersection.geoms:
                    # Calculate the distance from the point to the line segment
                    distance = calculate_distance_of_polyline(LineString([point, center_point]))
                    
                    # Check if this is the maximum distance so far
                    if min_distance is None or distance < min_distance:
                        min_distance = distance
                        min_point = point
                
                lineB = LineString([min_point, center_point])
                B_lines.append(lineB)
                second_side_perpendicular_lines_lengths.append(calculate_distance_of_polyline(lineB))
    
    #gpd.GeoDataFrame(geometry= list_of_perpendicular_line).to_file('data.shp')
    avg_distance_A_side = sum(one_side_perpendicular_lines_lengths) / len(one_side_perpendicular_lines_lengths)
    avg_distance_B_side = sum(second_side_perpendicular_lines_lengths) / len(second_side_perpendicular_lines_lengths)

    #print (avg_distance_A_side, avg_distance_B_side)
    return avg_distance_A_side, avg_distance_B_side



def road_function(src: rasterio, image: np.ndarray, road_geom_coords: list, road_id ,max_dist: float = 50, points_distance: float = 1.5, prependicular_length: float = 2) -> np.array:
    """
    Detects and extracts a road mask from an aerial image based on road geometry coordinates.

    Parameters:
        src (rasterio): Rasterio dataset object representing the source image.
        image (numpy.ndarray): Aerial image represented as a NumPy array.
        road_geom_coords (list): List of (longitude, latitude) coordinates representing the road geometry.
        max_dist (float, optional): Maximum distance threshold for material estimation. Defaults to 50.
        points_distance (float, optional): Distance between intermediate points. Defaults to 1.5.
        prependicular_length (float, optional): Length of perpendicular lines. Defaults to 2.

    Returns:
        numpy.ndarray: Binary mask representing the detected road.
    """
    try:
        bounds_shapefile = gpd.read_file(r'results\\boundries.shp')
        bound = bounds_shapefile[bounds_shapefile['id'] == road_id].iloc[0]['geometry']
        bound = (bound, 1)

        polygon_mask = features.geometry_mask([bound], out_shape=(src.height, src.width), transform=src.transform, invert=True).astype('uint8')

        return polygon_mask, bound[0]

    except:
        print (road_id)
        print ('boundries no found. working on the entire road')

    # Initialize an empty array for the final road mask
    combined_total = np.zeros(image.shape[:2], dtype= 'uint8')
    
    fixed_road_geom_coords = road_geom_coords
    initial_coord = fixed_road_geom_coords[0]
    
    # Iterate over coordinates in the road geometry
    for coord in fixed_road_geom_coords[1:len(fixed_road_geom_coords)]:
        initial , final  = initial_coord , coord
        
        # Generate intermediate points between two consecutive coordinates
        intern_poly = [Point(initial)] + create_intermidiate_points_between_true_points(Point(initial), Point(final), points_distance) + [Point(final)]
        
        list_of_perpendicular_line = []
        # Generate perpendicular lines to the intermediate polyline
        vec = calculate_line_vector_direction(LineString(intern_poly))
        for i in range(len(intern_poly)):
            perpendicular_line = create_perpendicular_line(LineString(intern_poly), intern_poly[i], vec)
            mod_perpendicular_line = modify_polyline(perpendicular_line, prependicular_length)
            list_of_perpendicular_line.append(mod_perpendicular_line)

        initial_coord = final

        # Extract pixel values intersecting with the perpendicular lines
        pixel_array = get_pixels_intersects_with_polylines(src ,image ,list_of_perpendicular_line)

        # Create a buffer polygon around the intermediate polyline
       
        bound = [(LineString(intern_poly).buffer(BAD_GUESS_POLYGON_BUFFER), 1)]
        
        polygon_mask = features.geometry_mask(bound, out_shape=(src.height, src.width), transform=src.transform, invert=True).astype('uint8')

        # Initialize an empty array for the combined mask
        combined = np.zeros(image.shape[:2]).astype('uint8')

        # Update the combined mask based on material estimation and distance criteria
        x , y = np.where(polygon_mask > 0)
        for i in range(x.shape[0]):
            column = x[i]
            row = y[i]
            dist_rgb = np.where(estimate_material(image[column, row] , pixel_array) < max_dist, 1 , 0)
            if 1 in dist_rgb:
                combined[column, row] = 1
        
        # Update the total combined mask
        combined_total = np.where(combined != 0, 255, combined_total)
        
        #print (initial, final)

    #kernel = np.ones(KERNEL_SIZE ,np.uint8)
    result = combined_total

    try:
        # Apply morphological operations to refine the mask
        opening = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel)
        dilation = cv2.dilate(opening, kernel,iterations = 1)
        
        # Find the biggest connected component in the refined mask
        mask = find_biggest_component(image, dilation)
        
        # Create a mask of a straight road from the road mask and perpendicular lines
        avg_distance_A_side, avg_distance_B_side = create_mask_of_straigh_road_from_road_mask(src, mask, list_of_perpendicular_line)

        # Handle cases where average distances exceed certain thresholds
        if avg_distance_A_side > AVRAGE_DISTANCE_MAX:
            avg_distance_A_side = AVRAGE_DISTANCE_MAX_DISTANCE_SUBSTITUTE
        if avg_distance_B_side > AVRAGE_DISTANCE_MAX:
            avg_distance_B_side = AVRAGE_DISTANCE_MAX_DISTANCE_SUBSTITUTE
        
        # Generate center lines based on the calculated average distances
        lineA = calculate_to_lines_to_center_line(fixed_road_geom_coords, avg_distance_A_side)
        lineB = calculate_to_lines_to_center_line(fixed_road_geom_coords, avg_distance_B_side, a_side= False)

        # Create a road polygon from the center lines
        road_polygon = Polygon(list(lineA.coords) + list(lineB.coords)[::-1])

        # Check if the polygon is closed and valid
        if road_polygon.is_empty or not road_polygon.is_valid or not road_polygon.exterior.is_closed:
            # If not closed, close the polygon
            exterior_coords = list(road_polygon.exterior.coords)
            if exterior_coords[0] != exterior_coords[-1]:
                exterior_coords.append(exterior_coords[0])
                road_polygon = Polygon(exterior_coords)
        
        # Create a mask from the road polygon
        shapes = [(road_polygon, 1)]
        polyline_mask = features.geometry_mask(shapes, out_shape=(src.height, src.width), transform=src.transform, invert=True).astype('uint8')
    

        return polyline_mask, road_polygon
    
    except:
        # Handle the exception by creating center lines and road polygon
        lineA = calculate_to_lines_to_center_line(fixed_road_geom_coords, AVRAGE_DISTANCE_FAILIURE)
        lineB = calculate_to_lines_to_center_line(fixed_road_geom_coords, AVRAGE_DISTANCE_FAILIURE, a_side= False)
        
        road_polygon = Polygon(list(lineA.coords) + list(lineB.coords)[::-1])
        # Check if the polygon is closed and valid
        if road_polygon.is_empty or not road_polygon.is_valid or not road_polygon.exterior.is_closed:
            # If not closed, close the polygon
            exterior_coords = list(road_polygon.exterior.coords)
            if exterior_coords[0] != exterior_coords[-1]:
                exterior_coords.append(exterior_coords[0])
                road_polygon = Polygon(exterior_coords)

        shapes = [(road_polygon, 1)]
        polyline_mask = features.geometry_mask(shapes, out_shape=(src.height, src.width), transform=src.transform, invert=True).astype('uint8')
        
        return polyline_mask, road_polygon


def equalize_with_blend(original_image, equalized_image, blend_factor):
    # Blend original and equalized images using the specified blend factor
    blended_image = cv2.addWeighted(original_image, 1 - blend_factor, equalized_image, blend_factor, 0)
    return blended_image

def create_outline(tile):
    """
    Creates an outlined and processed version of an input image.

    Parameters:
    tile (numpy.ndarray): Input RGB image with shape (3, height, width).

    Returns:
    numpy.ndarray: Processed outlined image with shape (height, width) and grayscale values.
    """
    # Step 1: Reshape the input tile to match the desired channel order
    reshaped_array = np.transpose(tile, (1, 2, 0))

    # Step 2: Convert the reshaped image to grayscale (Layer1)
    layer1 = cv2.cvtColor(reshaped_array, cv2.COLOR_BGR2GRAY)

    # Step 3: Calculate the negative version of Layer1 (inverse gray image)
    inverse_gray_image = np.uint8(255) - layer1

    # Step 4: Translate (shift) the inverse gray image
    translated = cv2.warpAffine(inverse_gray_image, np.float32([[1, 0, 2], [0, 1, -2]]), (reshaped_array.shape[1], reshaped_array.shape[0]))

    # Step 5: Blend Layer1 and the translated image
    blended_image = cv2.addWeighted(layer1, 0.5, translated, 0.5, 0)

    # Step 6: Apply histogram equalization to enhance contrast
    equalized_image = cv2.equalizeHist(blended_image)

    #reducing the equalizeHist function
    # Set the blend factor (0 to 1, 0 means no equalization effect, 1 means full equalization)

    #Blend the blended_image and equalized_blended_image with the specified blend factor
    blended_result = equalize_with_blend(blended_image, equalized_image, 0.5)

    # Step 7: Apply Gaussian blur for smoothing
    gauss_filter_blur_image = cv2.GaussianBlur(blended_result, (0, 0), 3)

    # The final processed image is the Gaussian-blurred outline
    outline = gauss_filter_blur_image

    # Return the processed outline
    return outline

def create_blur(tile: np.array, blur_level: int):
    # Step 7: Apply Gaussian blur for smoothing
    gauss_filter_blur_image = cv2.GaussianBlur(tile, (0, 0), blur_level)
    # Return the processed
    return gauss_filter_blur_image


def worker_function(data):
    """
    Process a section of a road shapefile and save the results as a pickle file.

    Parameters:
        section (int): Starting index of the section to process.
        image_path (str): Path to the image file.
        process_batch_size (int): Batch size for processing road geometries.
        pixel_distance_thresh (float): Maximum distance threshold for material estimation.
        spaces_between_perpendicular (float): Distance between intermediate points for perpendicular lines.
        perpendicular_length (float): Length of perpendicular lines.

    Returns:
        None
    """
    list_geometries = []
    section, image_path = data
    image = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)
    roads_shapefile = gpd.read_file('results/roads.shp')
    total_combined = np.zeros((image.shape[:2])).astype('uint8')
    #print('Total combined:',total_combined.shape)
    src = rasterio.open(image_path)
    
    end_point = section + PROCESS_BATCH_SIZE
    if section + PROCESS_BATCH_SIZE > len(roads_shapefile['geometry']):
        end_point = len(roads_shapefile['geometry'])
    

    data = [[row[-1], row[1]] for row in roads_shapefile.iloc[section: end_point].itertuples()]

    for i in tqdm(range(len(data))):
        geom, road_id = data[i]
        road_mask , road_polygon = road_function(
            src,
            image, 
            make_list_from_polyline(geom),
            road_id,
            PIXEL_DISTANCE_TRESH, 
            SPACES_BETWEEN_PREPENDICULAR, 
            PREPENDICULAR_LENGTH)
        
        #print ('Total Road:' ,road_mask.shape)
        list_geometries.append(road_polygon)
        total_combined = np.where(road_mask != 0, road_mask, total_combined)
    
    table = {'polyline': [str(make_list_from_polyline(polyline)) for polyline in roads_shapefile.geometry[section: end_point]]}
    road_associated_polygon = gpd.GeoDataFrame(geometry= list_geometries, data= table)



    
    import pickle
    os.makedirs('results/', exist_ok= True)
    with open(f'results/image_{section}_{end_point}.pkl', 'wb') as f:
        pickle.dump((road_associated_polygon, total_combined), f)
    
    return list_geometries

def break_to_linestrings(road_polyline, distance):
    all_polylines = []
    #road to straight polylines
    polylines = [LineString([road_polyline.coords[i - 1], road_polyline.coords[i]]) for i in range(1, len(road_polyline.coords))]
    for polyline in polylines:
        coords = [Point(coord) for coord in polyline.coords]
        
        polyline_with_distance_adjesments = LineString([coords[0]] + create_intermidiate_points_between_true_points(coords[0], coords[-1], distance) + [coords[-1]])
        
        polylines_with_distance_adjesments = [LineString([polyline_with_distance_adjesments.coords[i - 1], polyline_with_distance_adjesments.coords[i]]) 
                                                          for i in range(1, len(polyline_with_distance_adjesments.coords))]
        
        all_polylines += polylines_with_distance_adjesments

    return all_polylines

def modify_sumo(sumo_shapefile, road_distance):
    road_ids = []
    road_types = []
    tos = []
    from_s = []
    lanes_nums = []
    road_widths = []
    sidewalks = []
    sidewalk_ws = []
    geometries = []

    for road in sumo_shapefile.itertuples():
        _, road_id, road_type, to, from_, lanes_num, road_width, sidewalk, sidewalk_w, geometry = list(road) 
        linestrings = break_to_linestrings(geometry, road_distance)
        for idx , linestring in enumerate(linestrings):
            new_road_id = road_id + f'#{idx}'
            new_geometry = linestring

            road_ids.append(new_road_id)
            road_types.append(road_type)
            tos.append(to)
            from_s.append(from_)
            lanes_nums.append(lanes_num)
            road_widths.append(road_width)
            sidewalks.append(sidewalk)
            sidewalk_ws.append(sidewalk_w)
            geometries.append(new_geometry)

    table = {
                'id': road_ids,
                'type': road_types,
                'to': tos,
                'from': from_s,
                'lane_num': lanes_nums,
                'road_w': road_widths,
                'sidewalk': sidewalks,
                'sidewalk_w': sidewalk_ws
            }

    return gpd.GeoDataFrame(geometry= geometries, data = table)


if __name__ == '__main__':
    multiprocessing.freeze_support()

    parser = argparse.ArgumentParser(description='Detect roads in TIFF image')
    parser.add_argument('file_path', metavar='path', type=str, help="Path to the file you want to process")
    parser.add_argument('shapefile_path', metavar='shp_path', type=str, help="Path of shapefile with the roads only in the image")

    args = parser.parse_args()
    image_path = args.file_path
    shapefile_path = args.shapefile_path

    
    folder_path = "results"
    if os.path.exists(folder_path):
        shutil.rmtree(folder_path)
        print(f"The folder at {folder_path} and its contents have been successfully deleted.")
    else:
        print(f"The folder at {folder_path} does not exist.")

    num_cores = multiprocessing.cpu_count()

    print (shapefile_path)
    raw_roads_shapefile = gpd.read_file(shapefile_path)
    
    with rasterio.open(image_path) as src:
        profile = src.profile  # Get the metadata (e.g., spatial resolution, CRS)
        image_transform = src.transform
        crs = src.crs

        
    # Read the shapefile and reproject to match the CRS of the GeoTIFF
    shapefile = raw_roads_shapefile.to_crs(profile["crs"])  # Reproject the shapefile

    # Create a bounding box from the GeoTIFF extent
    geotiff_bounds = box(*src.bounds)

    # Filter out polygons that do not intersect with the GeoTIFF
    cleaned_shapefile = raw_roads_shapefile[raw_roads_shapefile.geometry.intersects(geotiff_bounds)]

    """
    #creating even length polylines for improving proccesing time
    if not(cleaned_shapefile.empty):
        list_even_length_linestrings = []
        for raw_geom in cleaned_shapefile['geometry']:
            list_coords_geom = []
            for i in range(1, len(raw_geom.coords)):
                list_coords_geom = list_coords_geom + [Point(raw_geom.coords[i-1])] + create_intermidiate_points_between_true_points(Point(raw_geom.coords[i-1]), Point(raw_geom.coords[i]), ROAD_SORT_SIZE)

            
            list_coords_geom = list_coords_geom + [Point(raw_geom.coords[-1])]
                

            for i in range(1, len(list_coords_geom)):
                print ([list_coords_geom[i-1], list_coords_geom[i]])
                list_even_length_linestrings.append(LineString([list_coords_geom[i-1], list_coords_geom[i]]))

        roads_shapefile = gpd.GeoDataFrame(geometry= list_even_length_linestrings)
        os.makedirs('results',exist_ok= True)
        
    """
    cleaned_shapefile = modify_sumo(cleaned_shapefile, ROAD_SORT_SIZE)
    os.makedirs('results',exist_ok= True)
    cleaned_shapefile.to_file('results/roads.shp')

    buldings_support.get_roads_boundries(config['BUILDINGS_VECTORS'], cleaned_shapefile, image_path).to_file('results/boundries.shp')



    if not(cleaned_shapefile.empty):
        print (os.path.basename(image_path))
        image = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)

        num_processes = num_cores
        index_list = [[i, image_path] for i in range(0, len(cleaned_shapefile['geometry']), PROCESS_BATCH_SIZE)]

        with multiprocessing.Pool(processes=num_processes) as pool:
            # Use the map function to apply the simple_function to each input value
            results = pool.map(worker_function, index_list)
            
        list_geoms = []
        for list_polygons in results:
            for geom in list_polygons:
                list_geoms.append(geom)
        
        
        grand_geodata_frame = gpd.GeoDataFrame()
        total = np.zeros(image.shape[:2]).astype('uint8')
        for path in glob.glob('results/*.pkl'):
            with open(path, 'rb') as f:
                data_frame ,array = pickle.load(f)
                grand_geodata_frame = pd.concat([grand_geodata_frame, data_frame])
            total = np.where(array != 0, array, total)
            os.remove(path)

    
        
        total = np.dstack((total, total, total))
        kernel = np.ones((5,5))

        dilation = cv2.dilate(total, kernel ,iterations = 3)
        erosion = cv2.erode(dilation ,kernel,iterations = 3)

        total = erosion[:,:,0]
        
        output_folder = os.path.join(OUTPUT_FOLDER, os.path.basename(image_path))

        os.makedirs(output_folder, exist_ok= True)
        
        gpd.GeoDataFrame(geometry= list_geoms).to_file(os.path.join(output_folder, 'road_width_polygons.shp'))

        copy(image_path, os.path.join(output_folder, os.path.basename(image_path)))
        """
            channel_values = np.where(total != 0, 255, 0)
            tifffile.imwrite(os.path.join(output_folder, 'mask.tif'), np.dstack((channel_values, channel_values, channel_values)))
            copy_geodata_to(image_path, os.path.join(output_folder, 'mask.tif'))

            outline = create_outline(np.transpose(image, (2, 0, 1)))
            tifffile.imwrite(os.path.join(output_folder, 'outline.tif'), channel_values)
            copy_geodata_to(image_path, os.path.join(output_folder, 'outline.tif'))
        """
        channel_values = np.where(total != 0, 255, 0)
        outline = create_outline(np.transpose(image, (2, 0, 1)))

        cv2.imwrite(f'{os.path.join(output_folder, "mask.png")}', channel_values)
        cv2.imwrite(f'{os.path.join(output_folder, "outline.png")}', np.dstack((outline, outline, outline)))

        tifffile.imwrite(os.path.join(output_folder, f'drawn_{os.path.basename(image_path)}'), draw_clean_roads_on_image(image, total))
        copy_geodata_to(image_path, os.path.join(output_folder, f'drawn_{os.path.basename(image_path)}'))

        gpd.GeoDataFrame(geometry= cleaned_shapefile.geometry).to_file(f'{output_folder}/roads_in_image.shp')

        grand_geodata_frame.to_file(f'{output_folder}/roads_in_image_with_association_to_road.shp')








    