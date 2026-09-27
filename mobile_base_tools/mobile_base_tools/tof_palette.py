#!/usr/bin/env python3

# Copyright 2026 Safwan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""One colour language for every ToF visual, whichever sensor drew it.

Colour carries meaning, never sensor identity: all eight sensors look the
same, so a colour change on screen always means the scene changed. Raw data is
a neutral slate, the floor a quiet green, and only classified obstacles use the
warning scale - raw rays hit the floor at close range all the time, so warning
colours on them would be permanently red and mean nothing.

The RViz config mirrors these values; keep ``mobile_base_sim.rviz`` in step.
"""

from std_msgs.msg import ColorRGBA


def rgba(hex_colour, alpha):
    """Build a ColorRGBA from '#rrggbb' and an alpha."""
    value = hex_colour.lstrip('#')
    red, green, blue = (int(value[i:i + 2], 16) / 255.0 for i in (0, 2, 4))
    return ColorRGBA(r=red, g=green, b=blue, a=alpha)


SLATE = '#C8D2DC'      # raw returns, ray hits  (RViz: 200; 210; 220)
SLATE_DIM = '#5A6470'  # rays that saw nothing
FLOOR = '#30A46C'      # confirmed floor         (RViz: 48; 164; 108)
CLEAR = '#52A9FF'      # obstacle, outside the caution band
CAUTION = '#F5A524'    # obstacle within CAUTION_RANGE
CRITICAL = '#E5484D'   # obstacle within CRITICAL_RANGE
TEXT = '#F0F4F8'
ALERT = '#FF7A1A'     # a ray, and its hit, on a confirmed obstacle
OFF = '#8A5A5E'       # a sensor switched off
# The LiDAR scan is drawn violet (RViz: 170; 140; 255), outside this scale.

# Measured from the body origin to the nearest face of the shape. The body
# reaches 0.13 m, so the critical band leaves about 10 cm of clearance.
CRITICAL_RANGE = 0.25
CAUTION_RANGE = 0.50

# Rays are context, not content: just enough to show the fan, never enough to
# hide what is behind it. Only a ray on an obstacle is allowed to stand out.
RAY_HIT = rgba(SLATE, 0.11)
RAY_MISS = rgba(SLATE_DIM, 0.055)
RAY_ALERT = rgba(ALERT, 0.38)
HIT_ALERT = rgba(ALERT, 0.95)
SENSOR_OFF = rgba(OFF, 0.7)
FLOOR_FILL = rgba(FLOOR, 0.18)
LABEL = rgba(TEXT, 0.95)


def warning_hex(distance):
    """Warning colour for an obstacle ``distance`` metres from the body."""
    if distance < CRITICAL_RANGE:
        return CRITICAL
    if distance < CAUTION_RANGE:
        return CAUTION
    return CLEAR


def warning_colour(distance, alpha):
    """Warning colour as a ColorRGBA."""
    return rgba(warning_hex(distance), alpha)
