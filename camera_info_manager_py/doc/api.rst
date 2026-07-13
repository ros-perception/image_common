API Reference
=============

Core classes and utilities (module :doc:`/camera_info_manager.camera_info_manager`):

- :class:`camera_info_manager.camera_info_manager.CameraInfoManager` —
  primary class for loading, saving and serving calibration data.
- :class:`camera_info_manager.camera_info_manager.CameraInfoError` —
  base exception raised on calibration errors.
- :class:`camera_info_manager.camera_info_manager.CameraInfoMissingError` —
  raised when :py:meth:`~camera_info_manager.camera_info_manager.CameraInfoManager.getCameraInfo`
  is called before
  :py:meth:`~camera_info_manager.camera_info_manager.CameraInfoManager.loadCameraInfo`.
- :func:`camera_info_manager.camera_info_manager.genCameraName`
- :func:`camera_info_manager.camera_info_manager.loadCalibrationFile`
- :func:`camera_info_manager.camera_info_manager.saveCalibration`
- :func:`camera_info_manager.camera_info_manager.resolveURL`
- :func:`camera_info_manager.camera_info_manager.parseURL`

Zoom camera classes (module :doc:`/camera_info_manager.zoom_camera_info_manager`):

- :class:`camera_info_manager.zoom_camera_info_manager.ZoomCameraInfoManager` —
  abstract base for zoom-aware cameras.
- :class:`camera_info_manager.zoom_camera_info_manager.ApproximateZoomCameraInfoManager` —
  computes K matrix from field-of-view bounds.
- :class:`camera_info_manager.zoom_camera_info_manager.InterpolatingZoomCameraInfoManager` —
  interpolates between calibrations at multiple zoom levels.
