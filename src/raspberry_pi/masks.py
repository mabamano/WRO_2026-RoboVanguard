"""
ROBOVANGUARD - WRO Future Engineers 2026
CIELAB Color Space Threshold Ranges
(Non-overlapping thresholds with Blue & Orange exclusion for Black Walls)
"""

# LAB Color Bounds [Low L, Low A, Low B], [High L, High A, High B]

# Black walls & borders:
# L <= 65 (dark), A in [100, 155], B in [115, 145] (neutral center to exclude Blue where B < 110)
rBlack = [[0, 100, 115], [65, 155, 145]]

# Orange turn indicator lines (WIDENED RANGE for 100% reliable detection under all lighting):
# L in [15, 255], A in [135, 220], B in [135, 255]
rOrange = [[15, 135, 135], [255, 220, 255]]

# Blue turn indicator lines:
rBlue = [[20, 110, 0], [255, 170, 110]]

# Red signal pillars:
rRed = [[0, 153, 140], [131, 198, 171]]

# Green signal pillars:
rGreen = [[0, 45, 0], [255, 117, 153]]

# Magenta parking lot area:
rMagenta = [[0, 171, 106], [255, 195, 135]]
lotType = "light"
