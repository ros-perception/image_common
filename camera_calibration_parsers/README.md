## Overview

`camera_calibration_parsers` contains C++ functions for reading and
writing camera parameters. These are mainly used internally by camera
drivers and camera calibration tools, but the formats are
human-readable.

For usage, see the
[code API](http://www.ros.org/doc/api/camera_calibration_parsers/html/).

## Command-line tools

### convert

This utility converts between calibration file formats.

#### Usage



```
$ rosrun  camera_calibration_parsers convert  in-file out-file
```



#### Example

##### yml to ini



```
$ rosrun  camera_calibration_parsers convert  cal.yml cal.ini
```



##### ost.txt to yml

*(**ost.txt** file is in the calibrationdata.tar.gz file saved during
the calibraton process)*



```
$ mv ost.txt ost.ini
$ rosrun  camera_calibration_parsers convert  ost.ini cal.yml
```



## File formats

### Videre INI

This is the INI format used by Videre cameras to store calibration
parameters internally. We are using it as our preferred format for
storing calibration on camera flash memory (not only Videre cameras).

Example:



```
# Prosilica camera intrinsics

[image]

width
2448

height
2050

[prosilica]

camera matrix
4827.93789 0.00000 1223.50000
0.00000 4835.62362 1024.50000
0.00000 0.00000 1.00000

distortion
-0.41527 0.31874 -0.00197 0.00071 0.00000

rectification
1.00000 0.00000 0.00000
0.00000 1.00000 0.00000
0.00000 0.00000 1.00000

projection
4827.93789 0.00000 1223.50000 0.00000
0.00000 4835.62362 1024.50000 0.00000
0.00000 0.00000 1.00000 0.00000
```



### YAML

The YAML output is based on that used by the OpenCV calibration program.

Example:



```
image_width: 2448
image_height: 2050
camera_name: prosilica
camera_matrix:
  rows: 3
  cols: 3
  data: [4827.94, 0, 1223.5, 0, 4835.62, 1024.5, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.41527, 0.31874, -0.00197, 0.00071, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [4827.94, 0, 1223.5, 0, 0, 4835.62, 1024.5, 0, 0, 0, 1, 0]
```
