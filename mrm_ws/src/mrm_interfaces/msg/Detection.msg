# Kameradan algılanan tek bir renkli nesne
string color                         # "red" | "green" | "blue"
geometry_msgs/Point position_map     # map çerçevesinde 3B konum
geometry_msgs/Point position_base    # base_footprint çerçevesinde 3B konum (görüntü anındaki robot pozuna göre)
geometry_msgs/Point position_odom    # odom çerçevesinde 3B konum (kısa vadede kaymayan: hedef takibi için)
float32 pixel_area                   # Blob alanı (piksel)
float32 depth                        # Kameraya uzaklık (m)
