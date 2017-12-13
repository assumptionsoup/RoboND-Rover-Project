import numpy as np
import cv2
import math
from scipy.ndimage.morphology import binary_erosion, binary_closing

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, lower_thresh=(160, 160, 160), upper_thresh=(256, 256, 256)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (upper_thresh[0] > img[:,:,0]) & (img[:,:,0] > lower_thresh[0]) \
                & (upper_thresh[1] > img[:,:,1]) & (img[:,:,1] > lower_thresh[1]) \
                & (upper_thresh[2] > img[:,:,2]) & (img[:,:,2] > lower_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

def rover_to_persp(binary_img, x_pixel, y_pixel):
    xpos = (-x_pixel + binary_img.shape[0]).astype(np.int)
    ypos = (-y_pixel + binary_img.shape[1] / 2).astype(np.int)
    mask = np.zeros([binary_img.shape[0], binary_img.shape[1]], dtype=bool)
    mask[xpos, ypos] = True
    # print(binary_img.shape)
    # print(len(xpos), binary_img.nonzero()[0][0], binary_img.nonzero()[1][0])
    return mask

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
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


class RoverImageData(object):
    def __init__(self, rover, image_mask=None):
        self.image = rover.img
        self.image_mask = image_mask or np.zeros_like(self.image[:,:,0])
        self.rover = rover

    def set_image_mask(self, mask):
        self.image_mask = mask
        return self

    def apply_threshold(self, *args, **kwargs):
        self.image_mask = color_thresh(self.image, *args, **kwargs)
        return self

    def build(self, cull_mask=None):
        # This isn't a great builder pattern implementation
        # I'm just hacking things as I go...

        self.perspective_culled = None
        self.rover_coords_culled = None
        self.world_coords_culled = None
        self.build_perspective()
        self.build_world_coords()
        if cull_mask is not None:
            self.build_culled(cull_mask)
        self.build_polar_coords()
        return self

    def build_perspective(self):
        dst_size = self.rover.grid_scale / 2.0
        bottom_offset = 0
        self.perspective = perspect_transform(
            self.image_mask,
            np.float32([[14, 140], [301, 140], [200, 96], [118, 96]]),
            np.float32([[self.image.shape[1] / 2 - dst_size, self.image.shape[0] - bottom_offset],
                        [self.image.shape[1] / 2 + dst_size, self.image.shape[0] - bottom_offset],
                        [self.image.shape[1] / 2 + dst_size, self.image.shape[0] - 2 * dst_size - bottom_offset],
                        [self.image.shape[1] / 2 - dst_size, self.image.shape[0] - 2 * dst_size - bottom_offset],
                        ])
        )
        return self

    def build_world_coords(self):
        self.rover_coords = rover_coords(self.perspective)
        self.world_coords = pix_to_world(
            *self.rover_coords, self.rover.pos[0], self.rover.pos[1],
            self.rover.yaw, self.rover.worldmap.shape[0], self.rover.grid_scale)
        return self

    def build_culled(self, cull_mask):
        self.perspective_culled = np.copy(self.perspective)
        self.perspective_culled[cull_mask] = 0
        self.rover_coords_culled = rover_coords(self.perspective_culled)
        self.world_coords_culled = pix_to_world(
            *self.rover_coords_culled, self.rover.pos[0], self.rover.pos[1],
            self.rover.yaw, self.rover.worldmap.shape[0], self.rover.grid_scale)
        return self

    def build_polar_coords(self):
        if self.rover_coords_culled is not None:
            self.dists, self.angles = to_polar_coords(*self.rover_coords_culled)
        else:
            self.dists, self.angles = to_polar_coords(*self.rover_coords)

    @classmethod
    def get_dist_mask(self, image, dist=7.5, grid_scale=10):
        # Find distances that are too far away
        max_dist = dist * grid_scale
        all_rover_coords = rover_coords(np.ones([image.shape[0], image.shape[1]]))
        rover_dist_ind = (np.sqrt(all_rover_coords[0]**2 + all_rover_coords[1]**2) > max_dist).nonzero()

        # Get image coords back from rover coords
        dist_mask = rover_to_persp(np.ones([image.shape[0], image.shape[1]]),
                                            all_rover_coords[0][rover_dist_ind[0]],
                                            all_rover_coords[1][rover_dist_ind[0]])
        return dist_mask


def first_nonzero(arr, axis, invalid_val=-1):
    mask = arr != 0
    return np.where(mask.any(axis=axis), mask.argmax(axis=axis), invalid_val)

def last_nonzero(arr, axis, invalid_val=-1):
    mask = arr[::-1,:] != 0
    return np.where(mask.any(axis=axis), mask.argmax(axis=axis), invalid_val)



# Apply the above functions in succession and update the Rover state accordingly
def perception_step(rover):
    image = rover.img

    # Find distances that are too far away
    dist_mask = RoverImageData.get_dist_mask(image)
    ##################################
    #   Find all rover image masks   #
    rock_data = RoverImageData(rover) \
        .apply_threshold((100, 100, -1), (220, 190, 80)) \
        .build(dist_mask)
    wall_data = RoverImageData(rover) \
        .apply_threshold((-1, -1, -1), (120, 120, 90)) \
        .build(dist_mask)
    forward_obstacle_data = RoverImageData(rover) \
        .apply_threshold((-1, -1, -1), (120, 120, 90)) \
        .build(dist_mask)

    obstacle_persp = forward_obstacle_data.perspective
    # Cull everything but the center view.
    wall_center = int(obstacle_persp.shape[1]/2)
    obstacle_persp[:,:wall_center-25] = 0
    obstacle_persp[:,wall_center+25:] = 0
    obstacle_persp[rock_data.perspective] = 0  # Don't avoid rock samples
    # Rebuild.
    forward_obstacle_data \
        .build_world_coords() \
        .build_culled(dist_mask) \
        .build_polar_coords()

    sky_data = RoverImageData(rover).apply_threshold((90, 90, 90))

    first_wall_data = first_nonzero(wall_data.image_mask, 0, wall_data.image.shape[0] - 1)
    below_skyline = first_wall_data <= np.arange(wall_data.image.shape[0])[:,None]
    sky_data.image_mask[below_skyline] = 0
    sky_data.build(dist_mask)

    terrain_data = RoverImageData(rover)

    last_wall_zeros = last_nonzero(wall_data.image_mask, 0, wall_data.image.shape[0] - 1)
    # This was stuff that would use the wall / sky data to improve
    # terrain_data - but it didn't help the fidelity score.
    # below_wall = wall_data.image.shape[0] - 1 - last_wall_zeros < np.arange(wall_data.image.shape[0])[:,None]
    # terrain_data.image_mask[below_wall] = 1
    terrain_data.apply_threshold((120, 150, 130))
    # non_terrain = np.ones_like(terrain_data.image_mask)
    # non_terrain[below_skyline] = 0
    # non_terrain[:,:] += wall_data.image_mask
    # non_terrain[:,:] += binary_closing(sky_data.image_mask, iterations=3)

    # terrain_data.apply_threshold((100, 100, 90))
    # terrain_data.image_mask[:,:] *= np.logical_not(non_terrain)
    terrain_data.build(dist_mask)


    ###########################
    #     Build debug image   #
    rover.vision_image[:,:,:] = 0
    def addRGB(bool_map, rgb):
        # I'm sure there's a quick numpy way to do this that I haven't
        # learned yet...
        for x in range(3):
            rover.vision_image[:,:,x] += bool_map * rgb[x]


    # The terrain is white / gray
    addRGB(terrain_data.perspective, (50, 50, 50))
    addRGB(terrain_data.perspective_culled, (100, 100, 100))

    # Walls are brown
    addRGB(wall_data.perspective, (81, 50, 30))
    addRGB(wall_data.perspective_culled, (30, 0, 0))

    # Rocks are bright yellow
    addRGB(rock_data.perspective, (140, 133, 70))
    addRGB(rock_data.perspective_culled, (115, 107, 45))

    # Blue skies
    addRGB(sky_data.perspective, (20, 20, 50))
    addRGB(sky_data.perspective_culled, (20, 20, 110))

    ## straight from camera debug
    # # The terrain is white / gray
    # addRGB(terrain_data.image_mask, (150, 150, 150))

    # # Walls are brown
    # addRGB(wall_data.image_mask, (91, 50, 30))

    # # Rocks are bright yellow
    # addRGB(rock_data.image_mask, (255, 240, 115))

    # # Blue skies
    # addRGB(sky_data.image_mask, (40, 40, 160))


    ###########################
    #    Save data to rover.  #

    rover.nav_dist = terrain_data.dists
    rover.nav_angles = terrain_data.angles

    rover.terrain_data = terrain_data
    rover.obstacle_data = wall_data
    rover.forward_obstacle_data = forward_obstacle_data

    def addToMap(image_data, channel, value):
        x, y = image_data.world_coords_culled
        rover.worldmap[y, x, channel] = np.minimum(rover.worldmap[y, x, channel] + value, 255)

    # world_dists = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    if (math.degrees(1.0 - math.cos(rover.pitch * np.pi / 180)) < 0.01 and
        math.degrees(1.0 - math.cos(rover.roll * np.pi / 180)) < 0.01):

        addToMap(wall_data, 0, 1)
        addToMap(rock_data, 1, 1)
        addToMap(terrain_data, 2, 1)


    if len(rock_data.world_coords[0]) > 5:
        rover.rocks_dist, rover.rocks_angles = to_polar_coords(*rock_data.rover_coords)
    else:
        rover.rocks_dist, rover.rocks_angles = None, None

    return rover
