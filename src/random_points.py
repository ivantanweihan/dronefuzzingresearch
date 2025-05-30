import random
from shapely.geometry import Polygon, Point
import helpers as helpers

# randomizer function using numpy
def polygon_random_points (lst, num_points):
    poly = Polygon(lst)

    min_x, min_y, max_x, max_y = poly.bounds
    
    points = []
    while len(points) < num_points:
        random_point = Point([random.uniform(min_x, max_x), random.uniform(min_y, max_y)])
        if (random_point.within(poly)):
            points.append(random_point)
            # helpers.debug_log("Generated X : " + str(random_point.x))
            # helpers.debug_log("Generated Y : " + str(random_point.y))
    return points