#!/usr/bin/env python
"""
GPS quality dict
@author: Michael Hutchinson (mhutchinson@sagarobotics.com)
"""

from sensor_msgs.msg import NavSatFix, NavSatStatus

gps_qualities = {
    # Unknown
    -1: [
        NavSatStatus.STATUS_NO_FIX,
        NavSatFix.COVARIANCE_TYPE_UNKNOWN
    ],
    # Invalid
    0: [
        NavSatStatus.STATUS_NO_FIX,
        NavSatFix.COVARIANCE_TYPE_UNKNOWN
    ],
    # SPS
    1: [
        NavSatStatus.STATUS_FIX,
        NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    ],
    # DGPS
    2: [
        NavSatStatus.STATUS_SBAS_FIX,
        NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    ],
    # PPS
    3: [
        NavSatStatus.STATUS_SBAS_FIX,
        NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    ],
    # RTK Fix
    4: [
        NavSatStatus.STATUS_GBAS_FIX,
        NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    ],
    # RTK Float
    5: [
        NavSatStatus.STATUS_GBAS_FIX,
        NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    ],
    # Estimated (dead reckoning) mode
    6: [
        NavSatStatus.STATUS_NO_FIX,
        NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    ],
    # Manual input mode
    7: [
        NavSatStatus.STATUS_SBAS_FIX,
        NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    ]
}