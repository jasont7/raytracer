import sys
import numpy as np


def save_ppm(image, file_path):
    height, width, _ = image.shape
    max_color = 255

    with open(file_path, 'w') as f:
        f.write("P3\n{} {}\n{}\n".format(width, height, max_color)) # header

        # pixel data
        for r in range(height):
            for c in range(width):
                color = image[r, c]
                f.write("{} {} {} ".format(color[0], color[1], color[2]))
            f.write('\n')


def normalize(v):
    # convert v to unit length vector
    return v / np.linalg.norm(v)


def reflect(v, axis):
    # reflect vector v around axis
    return v - 2 * np.dot(v, axis) * axis


def distance(p1, p2):
    # distance between two points
    return np.linalg.norm(p1 - p2)


class Light:
    def __init__(self, position, color):
        self.position = position
        self.color = color


class Sphere:
    def __init__(self, center, scale, color, ka, kd, ks, kr, shininess):
        # position
        self.center = center
        self.scale = scale
        self.transformation_matrix = np.diag(scale)
        self.inverse_transformation_matrix = np.diag(1 / np.array(scale))

        # illumination
        self.color = color
        self.ka = ka
        self.kd = kd
        self.ks = ks
        self.kr = kr
        self.shininess = shininess

    def intersect(self, origin, direction):
        # transform the ray
        transformed_origin = np.dot(self.inverse_transformation_matrix, origin - self.center)
        transformed_direction = normalize(np.dot(self.inverse_transformation_matrix, direction))

        # intersection with unit sphere
        a = np.dot(transformed_direction, transformed_direction)
        b = 2 * np.dot(transformed_origin, transformed_direction)
        c = np.dot(transformed_origin, transformed_origin) - 1
        delta = b**2 - 4*a*c

        if delta < 0:
            return None, False  # no intersection

        sqrt_delta = np.sqrt(delta)
        t1 = (-b - sqrt_delta) / (2 * a)
        t2 = (-b + sqrt_delta) / (2 * a)

        split = False
        if t1 > 2.24 and t2 > 2.24: # both intersections are in front of the ray origin
            t = min(t1, t2)
        elif t1 > 2.24 or t2 > 2.24: # one of t1 or t2 is in front of the ray origin
            t = max(t1, t2)
            split = True
        else:
            return None, False  # intersection is behind the ray origin

        # transform intersection point back to original space
        intersection_point = transformed_origin + t * transformed_direction
        intersection_point = np.dot(self.transformation_matrix, intersection_point) + self.center

        return intersection_point, split


class Scene:
    def __init__(self, camera, screen, width, height, bg_color, ambient, spheres, lights, out):
        self.camera = camera  # camera position
        self.width, self.height = width, height  # image resolution
        self.image = np.zeros((self.height, self.width, 3))
        self.screen = screen # [left, top, right, bottom]

        self.bg_color = bg_color
        self.ambient = ambient

        self.spheres = spheres
        self.lights = lights

        self.out = out

    def nearest_intersection(self, origin, direction):
        D = [] # distances from origin to intersection points
        for sphere in self.spheres:
            intersection_point, is_split = sphere.intersect(origin, direction)
            if intersection_point is not None:
                D.append(distance(intersection_point, origin))
            else:
                D.append(np.inf)

        min_dist = min(D)
        nearest_obj = self.spheres[D.index(min_dist)] if min_dist != np.inf else None
        return nearest_obj, min_dist, is_split
    
    def compute_transformed_normal(self, sphere, intersection_point):
        intersection_translated = intersection_point - sphere.center
        unscaled_normal = normalize(np.dot(sphere.inverse_transformation_matrix, intersection_translated))
        transformed_normal = normalize(np.dot(sphere.inverse_transformation_matrix.T, unscaled_normal))
        return transformed_normal

    def light_inside_sphere(self, light_position, sphere):
        # check if light is inside the sphere
        light_position_transformed = np.dot(sphere.inverse_transformation_matrix, light_position - sphere.center)
        return np.linalg.norm(light_position_transformed) < 1

    def compute_color(self, origin, direction):
        color = np.zeros((3))
        reflection = 1
        any_intersection = False

        for _ in range(3): # 3 reflection bounces
            nearest_obj, nearest_obj_dist, is_split = self.nearest_intersection(origin, direction)
            if not nearest_obj:
                break
            any_intersection = True

            intersection = origin + nearest_obj_dist * direction # IP between ray and nearest sphere

            # compute transformed normal at intersection point
            intersection_normal = self.compute_transformed_normal(nearest_obj, intersection)
            if is_split:
                intersection_normal = -intersection_normal

            # shift the intersection point a little bit towards the camera to avoid self-intersection
            intersection = intersection + 0.000001 * intersection_normal

            illumination = np.zeros((3)) # R,G,B

            # ambient component
            illumination += nearest_obj.ka * self.ambient * nearest_obj.color

            for light in self.lights:
                if is_split and not self.light_inside_sphere(light.position, nearest_obj):
                    continue

                # light_ray is the vector from the intersection point to the light
                light_ray = normalize(light.position - intersection)
                light_ray_dist = np.linalg.norm(light.position - intersection)

                # check if there is an object in the light ray path (blocking the light)
                _, blocking_obj_dist, _ = self.nearest_intersection(intersection, light_ray)
                if blocking_obj_dist < light_ray_dist - 0.000001:
                    # blocking object is closer to the light than the intersection point (in light ray path)
                    continue

                # diffuse component
                N_dot_L = max(np.dot(intersection_normal, light_ray), 0)
                illumination += nearest_obj.kd * light.color * N_dot_L * nearest_obj.color

                # specular component
                camera_ray = normalize(self.camera - intersection)
                R = 2 * np.dot(intersection_normal, light_ray) * intersection_normal - light_ray
                R = normalize(R)
                R_dot_V = max(np.dot(R, camera_ray), 0)
                n = nearest_obj.shininess
                illumination += nearest_obj.ks * light.color * R_dot_V**n

            # reflection component
            color += reflection * illumination
            reflection *= nearest_obj.kr
            direction = reflect(direction, intersection_normal)
            origin = intersection
        
        if not any_intersection:
            return self.bg_color

        color = np.clip(color, 0, 1) # bound color between 0 and 1
        return color

    def render(self):
        H = (self.screen[1] - self.screen[3]) / 2
        W = (self.screen[2] - self.screen[0]) / 2

        for r in range(self.height):  # nRows = self.height
            for c in range(self.width):  # nCols = self.width
                # convert pixel coordinates to screen space
                u_c = -W + W * (2*c / self.width)
                v_r = H - H * (2*r / self.height)

                P = np.array([u_c, v_r, 0])
                direction = normalize(P - self.camera)

                color = self.compute_color(self.camera, direction)
                self.image[r, c] = color
        
        self.image = (self.image * 255).astype(np.uint8)
        save_ppm(self.image, self.out)


def main():
    variables = { # default values
        'camera': np.array([0, 0, 1]),
        'screen': [-1, 1, 1, -1],
        'width': 100,
        'height': 100,
        'bg_color': np.zeros((3)),
        'ambient': np.zeros((3)),
        'spheres': [],
        'lights': [],
        'out': "",
    }

    # read input file
    file_path = sys.argv[1]
    with open(file_path, 'r') as file:
        for line in file:
            words = line.split()
            if not words: continue # skip empty lines

            key = words[0]
            if key == "NEAR":
                variables['camera'][2] = float(words[1])
            elif key == "LEFT":
                variables['screen'][0] = float(words[1])
            elif key == "TOP":
                variables['screen'][1] = float(words[1])
            elif key == "RIGHT":
                variables['screen'][2] = float(words[1])
            elif key == "BOTTOM":
                variables['screen'][3] = float(words[1])
            elif key == "RES":
                variables['width'], variables['height'] = int(words[1]), int(words[2])
            elif key == "AMBIENT":
                variables['ambient'] = np.array([float(words[1]), float(words[2]), float(words[3])])
            elif key == "BACK":
                variables['bg_color'] = np.array([float(words[1]), float(words[2]), float(words[3])])
            elif key == "OUTPUT":
                variables['out'] = words[1]
            elif key == "SPHERE":
                center = np.array([float(words[2]), float(words[3]), float(words[4]) + variables['camera'][2]])
                scale = np.array([float(words[5]), float(words[6]), float(words[7])])
                color = np.array([float(words[8]), float(words[9]), float(words[10])])
                ka = float(words[11])
                kd = float(words[12])
                ks = float(words[13])
                kr = float(words[14])
                shininess = int(words[15])
                variables['spheres'].append(Sphere(center, scale, color, ka, kd, ks, kr, shininess))
            elif key == "LIGHT":
                position = np.array([float(words[2]), float(words[3]), float(words[4]) + variables['camera'][2]])
                color = np.array([float(words[5]), float(words[6]), float(words[7])])
                variables['lights'].append(Light(position, color))

    scene = Scene(**variables)
    scene.render()


if __name__ == '__main__':
    main()
