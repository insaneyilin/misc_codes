#!/usr/bin/env python3

"""
Print today's time information
"""

import time
import datetime

# change your birthday here
MY_BIRTHDAY = datetime.datetime(2000, 1, 1, 00, 00, 00)
DT_NOW = datetime.datetime.now()

print(DT_NOW)
print('UNIX timestamp: {}'.format(DT_NOW.timestamp()))
print('Week: {} / 52'.format(time.strftime('%W')))
print('You have lived {} days'.format((DT_NOW - MY_BIRTHDAY).days))
