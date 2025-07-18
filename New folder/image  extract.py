import cv2

# Load image in grayscale
img = cv2.imread('initial_pos_2.png', cv2.IMREAD_GRAYSCALE)

# Create ORB detector
orb = cv2.ORB_create(nfeatures=1000)

# Detect keypoints and descriptors
keypoints, descriptors = orb.detectAndCompute(img, None)

# Draw keypoints on image
output_img = cv2.drawKeypoints(img, keypoints, None, color=(0,255,0), flags=0)

# Display result
cv2.imshow('ORB Keypoints', output_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
