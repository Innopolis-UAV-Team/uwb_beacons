DEFAULT_PARAMS = {
    'port': '/dev/ttyUSB1',
    'baud': 460800,
    'frame_id': 'map',
    'calibration': 'linear',
    'calib_params': [1.0, 0.0],
    'z_sign': 1,
    'timer_frequency': 100.0,
    'min_range': 0.0,
    'max_range': 100.0,
    'field_of_view': 0.1,
    'radiation_type': 1,
    'timeout': 1.0, # Serial timeout in seconds
    'publication_frequency': 10.0,  # Hz
    # GPS coordinate for zero position in landing station coordinates system
    'longitude': 0.0,
    'latitude': 0.0,
    'altitude': 0.0,

    "anchor.1": [0.0, 0.0, 0.0],
    "anchor.2": [3.0, 0.0, 0.0],
    "anchor.3": [0.0, 3.0, 0.0],
    "anchor.4": [3.0, 3.0, 0.0],
    "anchor.5": [0.0, 0.0, 0.0],
    "anchor.6": [0.0, 0.0, 0.0],
    "anchor.7": [0.0, 0.0, 0.0],
    "anchor.8": [0.0, 0.0, 0.0],
    "anchor.9": [0.0, 0.0, 0.0],
    "anchor.10": [0.0, 0.0, 0.0],
    "anchor.11": [0.0, 0.0, 0.0],
    "anchors_names": ["anchor.1", "anchor.2", "anchor.3", "anchor.4", "anchor.5", "anchor.6", "anchor.7", "anchor.8", "anchor.9", "anchor.10"]
}
