import cv2
import numpy as np

# Define HSV ranges for red, green, and blue
color_ranges = {
    "Red1": (np.array([0, 120, 70], np.uint8), np.array([10, 255, 255], np.uint8)),
    "Red2": (np.array([170, 120, 70], np.uint8), np.array([180, 255, 255], np.uint8)),
    "Green": (np.array([25, 52, 72], np.uint8), np.array([102, 255, 255], np.uint8)),
    "Blue": (np.array([94, 80, 2], np.uint8), np.array([120, 255, 255], np.uint8))
}

# Function to detect colors and contours
def detect_colors(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), "uint8")

    for color, (lower, upper) in color_ranges.items():
        # Create mask for the color
        if "Red" in color:
            mask = cv2.inRange(hsv_image, color_ranges["Red1"][0], color_ranges["Red1"][1]) + \
                cv2.inRange(hsv_image, color_ranges["Red2"][0], color_ranges["Red2"][1])
        else:
            mask = cv2.inRange(hsv_image, lower, upper)

        # mask = cv2.inRange(hsv_image, lower, upper)
        mask = cv2.dilate(mask, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 300:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(image, f"{color} Color", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return image

# List of images
path = "/home/kirilio3/Desktop/412/ros_3/images/" # Replace with your own image path
image_paths = [f"{path}blue.png", f"{path}green.png", f"{path}red.png"]

# Process each image
for image_path in image_paths:
    image = cv2.imread(image_path)
    
    if image is None:
        print(f"Error: Could not read {image_path}")
        continue

    processed_image = detect_colors(image)

    # Show the processed image
    cv2.imshow(f"Processed - {image_path}", processed_image)
    cv2.waitKey(0)

cv2.destroyAllWindows()


