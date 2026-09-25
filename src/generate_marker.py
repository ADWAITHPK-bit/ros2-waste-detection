import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
img = cv2.aruco.generateImageMarker(
    aruco_dict,
    0,
    200
)

# Add a white quiet-zone border - ArUco detection requires this margin
border_size = 40
img_with_border = cv2.copyMakeBorder(
    img, border_size, border_size, border_size, border_size,
    cv2.BORDER_CONSTANT, value=255
)

cv2.imwrite("marker_0.png", img_with_border)
print(f"Saved marker_0.png with {border_size}px white border, final size: {img_with_border.shape}")
