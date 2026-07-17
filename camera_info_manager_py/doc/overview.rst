Overview
========

``camera_info_manager_py`` provides a Python interface for camera calibration
data in ROS 2, mirroring the behaviour of the C++ ``camera_info_manager``
package.  It is intended for camera drivers written in Python that need to:

- Load ``sensor_msgs/msg/CameraInfo`` from a calibration file on disk.
- Save new calibration data received via the ``set_camera_info`` service.
- Serve the current calibration to the rest of the system.

Module Structure
----------------

.. code-block:: none

   camera_info_manager/
   ├── camera_info_manager.py       ← CameraInfoManager + utility functions
   └── zoom_camera_info_manager.py  ← ZoomCameraInfoManager and subclasses

``camera_info_manager.py``
~~~~~~~~~~~~~~~~~~~~~~~~~~

The main module defines:

- :class:`~camera_info_manager.CameraInfoManager` — the primary class for
  non-zoom cameras.
- :class:`~camera_info_manager.CameraInfoError` — base exception.
- :class:`~camera_info_manager.CameraInfoMissingError` — raised when
  ``getCameraInfo()`` / ``isCalibrated()`` is called before
  ``loadCameraInfo()``.
- Utility functions: :func:`~camera_info_manager.genCameraName`,
  :func:`~camera_info_manager.loadCalibrationFile`,
  :func:`~camera_info_manager.saveCalibration`,
  :func:`~camera_info_manager.resolveURL`, :func:`~camera_info_manager.parseURL`.

``zoom_camera_info_manager.py``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Provides zoom-aware subclasses:

- :class:`~camera_info_manager.zoom_camera_info_manager.ZoomCameraInfoManager`
  — abstract base for zoom cameras.
- :class:`~camera_info_manager.zoom_camera_info_manager.ApproximateZoomCameraInfoManager`
  — computes K matrix from field-of-view bounds.
- :class:`~camera_info_manager.zoom_camera_info_manager.InterpolatingZoomCameraInfoManager`
  — interpolates between calibrations taken at multiple zoom levels.

Calibration URL
---------------

The location for loading and saving calibration files is given as a URL.
Supported schemes:

``file://``
    Absolute filesystem path.

    .. code-block:: none

       file:///home/user/calibrations/camera.yaml

``package://``
    Path relative to the share directory of a ROS 2 package.

    .. code-block:: none

       package://my_driver/config/camera.yaml

URL substitution variables (resolved in one pass):

- ``${NAME}`` — the current camera name.
- ``${ROS_HOME}`` — ``$ROS_HOME`` env var, or ``~/.ros`` if unset.

Default URL when none is provided:

.. code-block:: none

   file://${ROS_HOME}/camera_info/${NAME}.yaml

Explicit Loading
----------------

Unlike the C++ ``camera_info_manager``, this Python implementation performs
no lazy loading.  You **must** call :py:meth:`~camera_info_manager.CameraInfoManager.loadCameraInfo`
before calling :py:meth:`~camera_info_manager.CameraInfoManager.getCameraInfo`
or :py:meth:`~camera_info_manager.CameraInfoManager.isCalibrated`.
Calling either without a prior ``loadCameraInfo()`` raises
:class:`~camera_info_manager.CameraInfoMissingError`.

ROS 2 Service
-------------

The constructor automatically advertises ``set_camera_info`` relative to the
node's namespace (or under an optional custom namespace prefix).  Calibration
tools can write new calibration data to the running driver via this service;
the manager saves it to the configured URL and updates the in-memory
``CameraInfo``.

Differences from C++ camera_info_manager
-----------------------------------------

- **No lazy loading**: ``loadCameraInfo()`` must be called explicitly.
- **No lifecycle support** in the base class constructor (pass node interfaces
  manually if needed).
- **Zoom camera classes** exist only in Python (no C++ equivalent yet).
