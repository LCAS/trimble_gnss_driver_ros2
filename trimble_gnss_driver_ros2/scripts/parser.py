#!/usr/bin/env python
"""
GSOF parser
@author: Michael Hutchinson (mhutchinson@sagarobotics.com)
"""

parse_maps = {
    1: [
        ('GPS_TIME',
         'GPS_WEEK',
         'SVN_NUM',
         'FLAG_1',
         'FLAG_2',
         'INIT_NUM'),
        ('>LH4B')
       ],
    2: [
        ('LATITUDE',
         'LONGITUDE',
         'HEIGHT_WGS84'),
        ('>3d'),
       ],
    3: [
        ('X_POS',
         'Y_POS',
         'Z_POS'),
        ('>3d')
       ],
    4: [
        ('LOCAL_DATUM_ID',
         'LOCAL_DATUM_LAT',
         'LOCAL_DATUM_LON',
         'LOCAL_DATUM_HEIGHT',
         'OPRT'),
        ('>8s3dB')
       ],
    5: [
        ('LOCAL_DATUM_ID',
         'LOCAL_ZONE_ID',
         'LOCAL_ZONE_NORTH',
         'LOCAL_ZONE_EAST',
         'LOCAL_DATUM_HEIGHT'),
        ('>2s3d')
       ],
    6: [
        ('DELTA_X',
         'DELTA_Y',
         'DELTA_Z'),
        ('>3d')
       ],
    7: [
        ('DELTA_EAST',
         'DELTA_NORTH',
         'DELTA_UP'),
        ('>3d')
       ],
    8: [
        ('VEL_FLAG',
         'VELOCITY',
         'HEADING',
         'VERT_VELOCITY'),
        ('>B3f')
       ],
    9: [
        ('PDOP',
         'HDOP',
         'VDOP',
         'TDOP'),
        ('>4f')
       ],
    10:[
        ('CLOCK_FLAG',
         'CLOCK_OFFSET',
         'FREQ_OFFSET'),
        ('>B2d')
       ],
    11:[
        ('POSITION_RMS_VCV',
         'VCV_XX',
         'VCV_XY',
         'VCV_XZ',
         'VCV_YY',
         'VCV_YZ',
         'VCV_ZZ',
         'UNIT_VAR_VCV',
         'NUM_EPOCHS_VCV'),
        ('>8fh')
       ],
    12:[
        ('POSITION_RMS_SIG',
         'SIG_EAST',
         'SIG_NORT',
         'COVAR_EN',
         'SIG_UP',
         'SEMI_MAJOR',
         'SEMI_MINOR',
         'ORIENTATION',
         'UNIT_VAR_SIG',
         'NUM_EPOCHS_SIG'),
        ('>9fh')
       ],
    15:[
        ('SERIAL_NUM'),
        ('>l')
       ],
    16:[
        ('GPS_MS_OF_WEEK',
         'CT_GPS_WEEK',
         'UTC_OFFSET',
         'CT_FLAGS'),
        ('>l2hB')
       ],
    26:[
        ('UTC_MS_OF_WEEK',
         'UTC_GPS_WEEK',
         'UTC_SVS_NUM',
         'UTC_FLAG_1',
         'UTC_FLAG_2',
         'UTC_INIT_NUM'),
        ('>lh4B')
       ],
    27:[
        ('GPS_TIME',
        'FLAGS',
        'SVN_NUM',
        'CALC_MODE',
        'RESERVED',
        'PITCH',
        'YAW',
        'ROLL',
        'MASTER_SLAVE_RANGE',
        'PDOP',
        'PITCH_VAR',
        'YAW_VAR',
        'ROLL_VAR',
        'PITCH_YAW_COVAR',
        'PITCH_ROLL_COVAR',
        'YAW_ROLL_COVAR',
        'MASTER_SLAVE_RANGE_VAR'),
        ('>L4B4dH7f')
       ],
    # 34 needs to be dealt with carefully/differently as it depends on the number of
    # satellites in view.
    # 34:[
    #     ('PRN',
    #      'SV_SYSTEM',
    #      'SV_FLAG1',
    #      'SV_FLAG2',
    #      'ELEVATION',
    #      'AZIMUTH',
    #      'SNR_L1',
    #      'SNR_L2',
    #      'SNR_L5'),
    #     ('>5Bh3B')
    #    ],
    35:[
        ('FLAGS',
         'BASE_NAME',
         'BASE_ID_1',
         'BASE_ID_2',
         'BASE_LATITUDE',
         'BASE_LONGITUDE',
         'BASE_HEIGHT'),
        ('>B8s2B3d')
       ],
    37:[
        ('BATT_CAPACITY',
         'REMAINING_MEM'),
        ('>hd')
       ],
    38:[
        ('OTHER',
         'OTHER',
         'OTHER',
         'OTHER',
         'OTHER',
         'OTHER',
         'OTHER',
         'OTHER',
         'OTHER',
         'OTHER',
         'OTHER',
         'OTHER',
         'POSITION_TYPE'),
        ('>L2Bf3BHBLBfB')
       ],
    41:[
        ('GPS_TIME_REF',
         'GPS_WEEK_REF',
         'LAT_REF',
         'LONG_REF',
         'HEIGHT_REF',
         'QI'),
        ('>LH3dB')
       ],
    49:[
        ('GPS_WEEK',
         'GPS_TIME',
         'IMU_ALIGNMENT_STATUS',
         'GPS_QUALITY',
         'FUSED_LATITUDE',
         'FUSED_LONGITUDE',
         'FUSED_ALTITUDE',
         'FUSED_VEL_NORTH',
         'FUSED_VEL_EAST',
         'FUSED_VEL_DOWN',
         'FUSED_TOTAL_SPEED',
         'FUSED_ROLL',
         'FUSED_PITCH',
         'FUSED_YAW',
         'FUSED_TRACK_ANGLE',
         'FUSED_VEL_ROLL',
         'FUSED_VEL_PITCH',
         'FUSED_VEL_YAW',
         'FUSED_ACC_X',
         'FUSED_ACC_Y',
         'FUSED_ACC_Z'),
        ('>HL2B3d4f4d6f')
       ],
    50:[
        ('GPS_WEEK',
         'GPS_TIME',
         'IMU_ALIGNMENT_STATUS',
         'GPS_QUALITY',
         'FUSED_RMS_LATITUDE',
         'FUSED_RMS_LONGITUDE',
         'FUSED_RMS_ALTITUDE',
         'FUSED_RMS_VEL_NORTH',
         'FUSED_RMS_VEL_EAST',
         'FUSED_RMS_VEL_DOWN',
         'FUSED_RMS_ROLL',
         'FUSED_RMS_PITCH',
         'FUSED_RMS_YAW'),
        ('>HL2B9f')
       ]
    }
