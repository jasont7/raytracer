# Ray Tracer

This is a simple ray tracer implemented in Python. It renders a 3D scene with spheres and lights, and saves the output as a PPM image file.

## How to Run

1. Download the RayTracer.py file.
2. Run the following command: `> python RayTracer.py input_file.txt`

Replace `input_file.txt` with the path to your input file.

## Input File Format

The ray tracer reads the scene configuration from an input file. The input file should follow the following format:

```
NEAR <distance>
LEFT <left>
TOP <top>
RIGHT <right>
BOTTOM <bottom>
RES <width> <height>
AMBIENT <r> <g> <b>
BACK <r> <g> <b>
OUTPUT <output_file.ppm>
SPHERE <name> <center_x> <center_y> <center_z> <scale_x> <scale_y> <scale_z> <color_r> <color_g> <color_b> <ka> <kd> <ks> <kr> <shininess>
LIGHT <name> <position_x> <position_y> <position_z> <color_r> <color_g> <color_b>
```

- `NEAR`: Sets the distance of the camera from the scene.
- `LEFT`, `TOP`, `RIGHT`, `BOTTOM`: Define the screen boundaries.
- `RES`: Sets the width and height of the output image.
- `AMBIENT`: Sets the ambient light color.
- `BACK`: Sets the background color.
- `OUTPUT`: Specifies the output file name.
- `SPHERE`: Defines a sphere in the scene. Parameters include the center position, scale, color, and material properties (ambient, diffuse, specular, reflection, shininess).
- `LIGHT`: Defines a light source in the scene. Parameters include the position and color.

Note: The input file can contain multiple `SPHERE` and `LIGHT` lines to define multiple spheres and lights in the scene.

## Example Input File

```
NEAR 1
LEFT -1
RIGHT 1
BOTTOM -1
TOP 1
RES 600 600
SPHERE s1 0 0 -10 2 4 2 0.5 0 0 1 1 0.9 0 50
SPHERE s2 4 4 -10 1 2 1 0 0.5 0 1 1 0.9 0 50
SPHERE s3 -4 2 -10 1 2 1 0 0 0.5 1 1 0.9 0 50
LIGHT l1 0 0 0 0.9 0.9 0.9
LIGHT l2 10 10 -10 0.9 0.9 0
LIGHT l3 -10 5 -5 0 0 0.9
BACK 1 1 1
AMBIENT 0.2 0.2 0.2
OUTPUT output.ppm
```

This input file sets up a scene with three spheres, a camera positioned at (0, 0, 1), and three light sources. The output image will have a resolution of 600x600 pixels and will be saved as `output.ppm`.

## Output

The ray tracer saves the rendered image as a PPM file specified in the input file's `OUTPUT` line.

The output from the example input file above looks like this:
![output.ppm](https://github.com/jasont7/raytracer/assets/26695415/0a10170e-ac51-4d97-803b-00aa0ff1ef44)
