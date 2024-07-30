# speed_faktor entspricht anzahl pixel auf dem Frame pro cm in echt
speed_faktor = 37.5

def pixel_to_cm(pixel, marker_size):
    
    return (pixel / speed_faktor) 
#def cm_to_pixel(cm, marker_size):
    return cm / speed_faktor
