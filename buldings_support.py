import geopandas as gpd
import os
from shapely.geometry import Polygon, box, Point,LineString
import numpy as np
import rasterio
from rasterio import features
from shapely.geometry import shape

from glob import glob
from matplotlib import pyplot as plt

from geopy.distance import geodesic
import math
from shapely.ops import unary_union

import json



def create_image_overlaping_polygon(image_path):
    # Open the image with rasterio
    with rasterio.open(image_path) as src:
        # Get the bounding box coordinates
        bounds = src.bounds

        return box(*bounds)

def get_vector(polyline):
    coords = [Point(coord) for coord in polyline.coords]
    
    vec = np.array([coords[-1].x - coords[0].x, 
                    coords[-1].y - coords[0].y])
    
    return Point(vec)

def get_vector_angle(vec):
    # Calculate the angle (m_angle_with_x_axis) of the vector with respect to the x-axis
    if (vec.x != 0):
        m_angle_with_x_axis = math.degrees(math.atan(vec.y / vec.x))
    else:
        m_angle_with_x_axis = 90

    return (m_angle_with_x_axis)


def get_set_of_points(angle, base_point, width_meters):
    point = Point(base_point)

    pointA = geodesic(meters= width_meters / 2).destination((point.y, point.x), -angle)
    pointA = Point([pointA.longitude, pointA.latitude])

    pointB = geodesic(meters= width_meters / 2).destination((point.y, point.x), -angle + 180)
    pointB = Point([pointB.longitude, pointB.latitude]) 

    return (pointA, pointB)


def create_polygon_from_polyline(road_polyline, width_meters):
    a_side = []
    b_side = []
    #road to straight polylines
    polylines = [LineString([road_polyline.coords[i - 1], road_polyline.coords[i]]) for i in range(1, len(road_polyline.coords))]
    
    for polyline in polylines:
        vector = get_vector(polyline)
        angle = get_vector_angle(vector)

        pointA, pointB = get_set_of_points(angle, polyline.coords[0], width_meters)

        a_side.append(pointA)
        b_side.append(pointB)

    
    pointA, pointB = get_set_of_points(angle, polylines[-1].coords[-1], width_meters)
    
    a_side.append(pointA)
    b_side.append(pointB)

    return Polygon(a_side + b_side[::-1])



    
def get_roads_boundries(buildings_shapefile_path, roads_from_sumo_shapefile, image_path):
    #building vectors
    
    buildings_shapefile = gpd.read_file(buildings_shapefile_path)
    roads_from_sumo_shapefile = roads_from_sumo_shapefile

    #images                            
    SHAPES = list(buildings_shapefile['geometry'])
    
    #read image data using rasterio
    src = rasterio.open(image_path)

    # Create a masked array to obtain the pixel values that overlap with the polyline
    polyline_mask = features.geometry_mask(SHAPES, out_shape=(src.height, src.width), transform=src.transform, invert= True).astype('uint8') * 255

    # Find shapes in the mask using rasterio.features.shapes
    #shapes_ = features.shapes(polyline_mask, mask= polyline_mask > 0,transform= src.transform)

    # Convert all shapes to Shapely Polygons
    #polygons = [shape(s) for s, _ in shapes_]

    #polygons_list.append(polygons)

    #Lets Try to decide what roads' widths will be determened by the buildings surrounding it, or will be gussed.
    image_bounds = create_image_overlaping_polygon(image_path)

    #get the roads that are within the image
    image_roads = roads_from_sumo_shapefile[roads_from_sumo_shapefile.geometry.intersects(image_bounds)]

    # Modify the geometry of intersecting polylines to be within the image bounds
    #for index, row in image_roads.iterrows():
    #    intersection = row['geometry'].intersection(image_bounds)
    #    image_roads.at[index, 'geometry'] = intersection

    road_ids = []
    road_polygons = []
    for road in image_roads.itertuples():
        road_polyline = road[-1]
        road_id = road[1]

        road_polygon = create_polygon_from_polyline(road_polyline, 15)

        road_mask = features.geometry_mask([(road_polygon, 1)], out_shape=(src.height, src.width), transform=src.transform, invert= True).astype('uint8') * 255

        col, row = np.where((road_mask == 255) & (polyline_mask == 255))
        
        if len(col) != 0:    
            road_mask[(col, row)] = 0

            # Find shapes in the mask using rasterio.features.shapes
            shapes_ = features.shapes(road_mask, mask= road_mask == 255, transform= src.transform)

            # Convert all shapes to Shapely Polygons
            polygons = [shape(s) for s, _ in shapes_]

            polygon = unary_union(polygons)

            if polygons != []:
                
                if polygon.geom_type == 'Polygon':
                    road_polygons.append(polygon)
                    road_ids.append(road_id)
                
                elif (polygon.geom_type == 'MultiPolygon'):

                    largest_polygon = None
                    largest_area = 0.0
                    # Iterate through each polygon in the MultiPolygon
                    for poly in polygon.geoms:
                        # Check the area of the current polygon
                        area = poly.area

                        # Update the largest polygon and area if the current polygon has a larger area
                        if area > largest_area:
                            largest_polygon = poly
                            largest_area = area
                    
                    road_polygons.append(largest_polygon)   
                    road_ids.append(road_id)

    table = {'id': road_ids}
    
    return gpd.GeoDataFrame(geometry = road_polygons, data = table)
