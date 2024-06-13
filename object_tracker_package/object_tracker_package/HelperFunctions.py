# Helper Functions
def pixel_to_cm(pixel, marker_size):
    return pixel * (5.2 / marker_size)  # Marker eig. 5,2

#def cm_to_pixel(cm, marker_size):
    return cm / (6.4 * marker_size)
