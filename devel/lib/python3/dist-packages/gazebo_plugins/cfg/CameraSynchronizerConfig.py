## *********************************************************
##
## File autogenerated for the gazebo_plugins package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [{'name': 'projector_rate', 'type': 'double', 'default': 60.0, 'level': 31, 'description': 'Projector pulse frequency in Hz.', 'min': 40.0, 'max': 120.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'projector_pulse_length', 'type': 'double', 'default': 0.002, 'level': 31, 'description': 'Length of the projector pulses in s. At high currents the hardware may limit the pulse length.', 'min': 0.001, 'max': 0.002, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'projector_pulse_shift', 'type': 'double', 'default': 0.0, 'level': 31, 'description': 'How far off-center the intermediate projector pulses are. Zero is on-center, one is touching the following pulse.', 'min': 0.0, 'max': 1.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'projector_mode', 'type': 'int', 'default': 2, 'level': 31, 'description': 'Indicates whether the projector should be off, on when in use or on all the time.', 'min': 1, 'max': 3, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'ProjectorOff', 'type': 'int', 'value': 1, 'srcline': 56, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The projector is always off.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'ProjectorAuto', 'type': 'int', 'value': 2, 'srcline': 57, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The projector is on if one of the cameras is using it.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'ProjectorOn', 'type': 'int', 'value': 3, 'srcline': 58, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The projector is always on.', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'The projectors operating mode.'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'prosilica_projector_inhibit', 'type': 'bool', 'default': False, 'level': 16, 'description': 'Indicates if the projector should turn off when the prosilica camera is exposing.', 'min': False, 'max': True, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'stereo_rate', 'type': 'double', 'default': 30.0, 'level': 3, 'description': 'Indicates the frame rate for both stereo cameras in Hz. (Gets rounded to suitable divisors of projector_rate.)', 'min': 1.0, 'max': 60.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'wide_stereo_trig_mode', 'type': 'int', 'default': 4, 'level': 3, 'description': 'Indicates the triggering mode of the wide stereo camera.', 'min': 2, 'max': 4, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'IgnoreProjector', 'type': 'int', 'value': 2, 'srcline': 62, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The cameras frequency can be set independently of the projector frequency. There is no deterministic phase relation between projector firing and camera triggering.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'WithProjector', 'type': 'int', 'value': 3, 'srcline': 63, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera always exposes while the projector is on.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'WithoutProjector', 'type': 'int', 'value': 4, 'srcline': 64, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera always exposes while the projector is off.', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'The triggering mode for the wide camera.'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'narrow_stereo_trig_mode', 'type': 'int', 'default': 4, 'level': 3, 'description': 'Indicates the triggering mode of the narrow stereo camera.', 'min': 2, 'max': 5, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'IgnoreProjector', 'type': 'int', 'value': 2, 'srcline': 62, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The cameras frequency can be set independently of the projector frequency. There is no deterministic phase relation between projector firing and camera triggering.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'WithProjector', 'type': 'int', 'value': 3, 'srcline': 63, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera always exposes while the projector is on.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'WithoutProjector', 'type': 'int', 'value': 4, 'srcline': 64, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera always exposes while the projector is off.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'AlternateProjector', 'type': 'int', 'value': 5, 'srcline': 65, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera alternates between frames with and without the projector.', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'The triggering mode for the narrow camera.'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'forearm_r_rate', 'type': 'double', 'default': 30.0, 'level': 4, 'description': 'Indicates the frame rate for the right forearm camera in Hz. (Gets rounded to suitable divisors of projector_rate.)', 'min': 1.0, 'max': 60.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'forearm_r_trig_mode', 'type': 'int', 'default': 1, 'level': 4, 'description': 'Indicates the triggering mode of the right forearm camera.', 'min': 1, 'max': 4, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'InternalTrigger', 'type': 'int', 'value': 1, 'srcline': 61, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera does not use the trigger input.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'IgnoreProjector', 'type': 'int', 'value': 2, 'srcline': 62, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The cameras frequency can be set independently of the projector frequency. There is no deterministic phase relation between projector firing and camera triggering.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'WithProjector', 'type': 'int', 'value': 3, 'srcline': 63, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera always exposes while the projector is on.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'WithoutProjector', 'type': 'int', 'value': 4, 'srcline': 64, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera always exposes while the projector is off.', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'The triggering mode for a forearm camera.'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'forearm_l_rate', 'type': 'double', 'default': 30.0, 'level': 8, 'description': 'Indicates the frame rate for the left forearm camera in Hz. (Gets rounded to suitable divisors of projector_rate.)', 'min': 1.0, 'max': 60.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'forearm_l_trig_mode', 'type': 'int', 'default': 1, 'level': 8, 'description': 'Indicates the triggering mode of the left forearm camera.', 'min': 1, 'max': 4, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'InternalTrigger', 'type': 'int', 'value': 1, 'srcline': 61, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera does not use the trigger input.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'IgnoreProjector', 'type': 'int', 'value': 2, 'srcline': 62, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The cameras frequency can be set independently of the projector frequency. There is no deterministic phase relation between projector firing and camera triggering.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'WithProjector', 'type': 'int', 'value': 3, 'srcline': 63, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera always exposes while the projector is on.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'WithoutProjector', 'type': 'int', 'value': 4, 'srcline': 64, 'srcfile': '/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_plugins/cfg/CameraSynchronizer.cfg', 'description': 'The camera always exposes while the projector is off.', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'The triggering mode for a forearm camera.'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'projector_tweak', 'type': 'double', 'default': 0.0, 'level': 31, 'description': 'Adds a time shift in seconds to the projector timing. Useful for debugging but not in normal use.', 'min': -0.1, 'max': 0.1, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'camera_reset', 'type': 'bool', 'default': False, 'level': 31, 'description': 'Does a hard reset of all the cameras using a long pulse on the trigger line. This parameter resets itself to false after 3 to 4 seconds.', 'min': False, 'max': True, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}], 'groups': [], 'srcline': 246, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

CameraSynchronizer_ProjectorOff = 1
CameraSynchronizer_ProjectorAuto = 2
CameraSynchronizer_ProjectorOn = 3
CameraSynchronizer_InternalTrigger = 1
CameraSynchronizer_IgnoreProjector = 2
CameraSynchronizer_WithProjector = 3
CameraSynchronizer_WithoutProjector = 4
CameraSynchronizer_AlternateProjector = 5
