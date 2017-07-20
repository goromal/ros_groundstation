
NED_with_GPS_defaults = \
    {
        'name': 'Principle Subscibers and Publishers',
        'description': 'All ROS information is exchanged with the plane in NED format, and the plane\n' +
        'may provide an initial GPS position for accurate latlong rendering.',
        'pubsubs':
        {
            'State Subscriber': (True, '/state'),
            'GPS init Subscriber': (False, '/gps_init'),
            'Path Subscriber': (True, '/current_path'),
            'Waypoint Subscriber': (True, '/waypoint_path'),
            'Waypoint Publisher': (False, '/waypoint/pub')
        }
    }

misc_defaults = \
    {
        'RC Raw Subscriber': (True, '/rc_raw'),
        'GPS Data Subscriber': (True, '/gps/data'),
        'Obstacle Subscriber': (False, '/obstacles')
    }
