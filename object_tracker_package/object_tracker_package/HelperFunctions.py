# Helper Functions
def pixel_to_cm(pixel, marker_size):
    return (pixel / 40)  # Marker eig. 2.01, 5.2 / marker_size

#def cm_to_pixel(cm, marker_size):
    return cm / (6.4 * marker_size)
