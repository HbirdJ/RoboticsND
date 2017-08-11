import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160), rgb_max=(256,256,256)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) & (img[:,:,0] <= rgb_max[0]) \
                & (img[:,:,1] > rgb_thresh[1]) & (img[:,:,1] <= rgb_max[1]) \
                & (img[:,:,2] > rgb_thresh[2]) & (img[:,:,2] <= rgb_max[2]) 
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = xpix*np.cos(yaw_rad) - ypix*np.sin(yaw_rad)
    ypix_rotated = xpix*np.sin(yaw_rad) + ypix*np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot/scale + xpos)
    ypix_translated = (ypix_rot/scale + ypos)
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    
    img = Rover.img
    rock_rgb = (0, 50, 0)
    rock_rgb_max = (255, 255, 35)
    dst_size = 5
    bottom_offset = 6
    
    # 1) Define source and destination points for perspective transform
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    
    # 2) Apply perspective transform
    transformed = perspect_transform(img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    rock_rgb = (0, 50, 0)
    rock_rgb_max = (255, 255, 35)
    
    rock = color_thresh(transformed, rock_rgb, rock_rgb_max)
    ground = color_thresh(transformed)
    #borders = color_thresh(transformed,(-1,-1,-1))-color_thresh(transformed)-rock
    borders = color_thresh(transformed, (0,0,0))-rock-ground
    warped = np.dstack((borders*255, rock*255, ground*255)).astype(np.float)
    # transformed = perspect_transform(warped, source, destination)
    
    
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    #Rover.vision_image[:,:,0] = borders
    #Rover.vision_image[:,:,1] = rock
    #Rover.vision_image[:,:,2] = ground
    
    # Rover.vision_image = warped
    


    # 5) Convert map image pixel values to rover-centric coords
    #Ignore edges of image due to loss of fidelity
    boxx_min = 100
    boxy_min = 80
    boxx_max = 160
    boxy_max = 280
    

    xpix_obs, ypix_obs = rover_coords(borders[boxx_min:boxx_max, boxy_min:boxy_max])
    xpix_rock, ypix_rock = rover_coords(rock)
    xpix_ground, ypix_ground = rover_coords(ground[boxx_min:boxx_max, boxy_min:boxy_max])
    
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = 200
    scale = 10
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
   
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obs, ypix_obs, xpos, ypos, yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(xpix_rock, ypix_rock, xpos, ypos, yaw, world_size, scale)
    navigable_x_world, navigable_y_world = pix_to_world(xpix_ground, ypix_ground, xpos, ypos, yaw, world_size, scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
        
    #Judge accuracy of perception based on pitch and roll
    pitch = Rover.pitch * np.pi/180
    roll = Rover.roll * np.pi/180
    inaccuracy = (1 - np.cos(pitch))*1000+(1 - np.cos(roll))*1000
    print(inaccuracy)
    
#     map_building = np.zeros_like(Rover.worldmap)
    
#     map_building[obstacle_y_world, obstacle_x_world, 0] += 255
#     map_building[rock_y_world, rock_x_world, 1] += 255
#     map_building[navigable_y_world, navigable_x_world, 2] += 255
    
    if inaccuracy < .2:
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += (25)
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 2] -= (25)
        Rover.worldmap[rock_y_world, rock_x_world, 1] += (1)
        Rover.worldmap[navigable_y_world, navigable_x_world,2] += (50)
        Rover.worldmap[navigable_y_world, navigable_x_world,0] -= (25)
    
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_ground, ypix_ground)
    
    nav_average = np.mean(Rover.nav_angles)*180/np.pi+35
    nav_dist = np.mean(Rover.nav_dists)
    
    cv2.putText(warped,"Goal Dir: "+ "%.2f" % nav_average, (0, 20), 
                  cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 1)
    cv2.putText(warped,"Avail Ground: "+ "%.2f" % nav_dist, (0, 40), 
                  cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 1)
    cv2.putText(warped,"Mode: "+ Rover.mode, (0, 60), 
                  cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 1)
    
    Rover.vision_image = warped
    

    
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
       
    
 
    
    
    return Rover