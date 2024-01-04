import geopandas as gpd
import cv2
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image

from blend_modes import darken_only
from blend_modes import lighten_only
from blend_modes import normal


def create_grany_road(true_image, mask_roads):
     # Find the coordinates of non-zero values in the binary mask
    non_zero_coords = np.where(mask_roads > 0)

    # Extract pixel values and corresponding indices from the original image
    masked_pixels = true_image[non_zero_coords]

    # Shuffle the masked pixels and get the shuffled indices
    shuffled_indices = np.random.permutation(len(masked_pixels))

    # Create a final image with zeros
    final_image = np.zeros_like(true_image)

    # Assign shuffled pixel values to corresponding positions in the final image
    final_image[non_zero_coords] = masked_pixels[shuffled_indices]

    return final_image


def combine_to_road(cancel_phazes, mask_roads, true_image, graint_road, brightness_mask):
    base_image = np.where(np.dstack((mask_roads, mask_roads, mask_roads)) > 0, np.dstack((cancel_phazes, cancel_phazes, cancel_phazes)), true_image)
    
    brightness_mask_dark = np.array(Image.fromarray(brightness_mask).convert('RGBA'), dtype= float)
    brightness_mask_light = np.array(Image.fromarray(brightness_mask).convert('RGBA'), dtype= float)
    base_image_RGBA = np.array(Image.fromarray(base_image).convert('RGBA'), dtype= float)
    graint_road_RGBA = np.array(Image.fromarray(graint_road).convert('RGBA'), dtype= float)

    base_image_before_pixelated_road_2 = lighten_only(base_image_RGBA, brightness_mask_dark, 0.50)
    base_image_before_pixelated_road_1 = darken_only(base_image_RGBA, brightness_mask_light, 0.25)

    final = cv2.addWeighted(base_image_before_pixelated_road_1, 0.5, base_image_before_pixelated_road_2, 0.5, 0)

    final_mask = normal(final, graint_road_RGBA, 0.15).astype('uint8')
    final_mask = np.array(Image.fromarray(final_mask).convert('RGB'), dtype= np.uint8)

    final_image = np.where(np.dstack((mask_roads, mask_roads, mask_roads)) > 0, final_mask, true_image)


    return final_image

def cancel_phazes(true_image, mask_roads):    
    # Step 2: Convert the reshaped image to grayscale (Layer1)
    layer1 = cv2.cvtColor(true_image, cv2.COLOR_RGB2GRAY)

    # Step 3: Calculate the negative version of Layer1 (inverse gray image)
    inverse_gray_image = np.uint8(255) - layer1

    # Step 5: Blend Layer1 and the gray_image 
    blended_image = cv2.addWeighted(layer1, 0.5, inverse_gray_image, 0.5, 0)

    return np.where(mask_roads > 0, blended_image, 0)
def create_brightness_mask(true_image, mask_roads):
    # Convert the original image to HSV format
    blurred_img = cv2.GaussianBlur(true_image, (0,0), 25)
    original_hsv = cv2.cvtColor(blurred_img, cv2.COLOR_RGB2HSV)

    # Get the indices of non-zero values in the binary mask
    non_zero_coords = np.where(mask_roads > 0)

    # Extract pixel values from the HSV image at the non-zero coordinates
    masked_pixels_hsv = original_hsv[non_zero_coords]

    # Calculate brightness for each pixel (Value channel in HSV)
    brightness_values = masked_pixels_hsv[:, 2]

    # Normalize brightness values between 0 and 1
    normalized_brightness = (brightness_values - np.min(brightness_values)) / (np.max(brightness_values) - np.min(brightness_values))

    # Create a brightness mask with the shape of the original image
    brightness_mask = np.zeros(true_image.shape[:2])

    brightness_mask[non_zero_coords] = np.clip(normalized_brightness, 0.25, 0.75)

    return np.uint8(brightness_mask * 255)


def create_image(true_image, mask_roads):
    #apply 'edge-blur' to roads mask to make it look as one road
    mask_roads = cv2.GaussianBlur(mask_roads, (21, 21), 0)
    mask_roads = np.where(mask_roads > 0, 255, 0)

    brightness_mask = create_brightness_mask(true_image, mask_roads)
    canceled_phazes = cancel_phazes(true_image, mask_roads)
    grany_road = create_grany_road(true_image, mask_roads)
    image = combine_to_road(canceled_phazes, mask_roads, true_image, grany_road, brightness_mask)
   
    blurred_img = cv2.GaussianBlur(image, (9, 9), 0)
    mask = np.zeros(image.shape, np.uint8)

    gray = mask_roads.astype('uint8')
    thresh = gray
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(mask, contours, -1, (255,255,255),5)
    output = np.where(mask==np.array([255, 255, 255]), blurred_img, image)
   
    return output