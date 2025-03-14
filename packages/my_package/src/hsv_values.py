import cv2
import numpy as np

# Function to calculate the lower and upper HSV values for the selected ROI
def get_hsv_range(image, roi):
    """
    image: The input image in BGR color space
    roi: The selected region of interest (ROI) in the format (x, y, w, h)
    """
    x, y, w, h = roi  # Extract the ROI coordinates (x, y, width, height)

    # Crop the image to focus on the region of interest (ROI)
    roi_image = image[y:y+h, x:x+w]

    # Convert the image from BGR to HSV
    hsv_image = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)

    # Calculate the lower and upper HSV values (based on the ROI)
    lower_hue = np.min(hsv_image[:, :, 0])
    upper_hue = np.max(hsv_image[:, :, 0])
    lower_saturation = np.min(hsv_image[:, :, 1])
    upper_saturation = np.max(hsv_image[:, :, 1])
    lower_value = np.min(hsv_image[:, :, 2])
    upper_value = np.max(hsv_image[:, :, 2])

    # Return the lower and upper HSV values
    lower_hsv = np.array([lower_hue, lower_saturation, lower_value])
    upper_hsv = np.array([upper_hue, upper_saturation, upper_value])

    return lower_hsv, upper_hsv

def process_image(image_path):
    """
    This function processes a single image by allowing the user to select an ROI
    and then calculates the lower and upper HSV values for that ROI.
    """
    # Read the image
    image = cv2.imread(image_path)

    # Show the image and allow the user to select the ROI manually
    roi = cv2.selectROI("Select ROI", image, fromCenter=False, showCrosshair=True)
    cv2.destroyWindow("Select ROI")  # Close the ROI selection window

    # Get the lower and upper HSV values for the selected ROI
    lower_hsv, upper_hsv = get_hsv_range(image, roi)

    # Display the results
    print(f"HSV Range for image {image_path}:")
    print("Lower HSV:", lower_hsv)
    print("Upper HSV:", upper_hsv)

    # Optionally, show the selected ROI and its HSV representation
    x, y, w, h = roi
    roi_image = image[y:y+h, x:x+w]
    hsv_roi_image = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
    
    # Display the original ROI and HSV image
    cv2.imshow(f"Original ROI - {image_path}", roi_image)
    cv2.imshow(f"HSV ROI - {image_path}", hsv_roi_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    path = '/home/kirilio3/Desktop/412/ros_3/images/'   # Replace with your own image path
    # List of images to process
    image_paths = [f"{path}blue.png", f"{path}green.png", f"{path}red.png"]  

    # Loop through each image and process it
    for image_path in image_paths:
        process_image(image_path)
