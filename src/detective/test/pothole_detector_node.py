import numpy as np
import cv2
import matplotlib.pyplot as plt

def detect_potholes(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise and improve edge detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Use Canny edge detection to find edges in the image
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    potholes = []

    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the approximated contour has an elliptical shape
        if len(approx) >= 5:  # Minimum points required to fit an ellipse
            ellipse = cv2.fitEllipse(approx)
            potholes.append(ellipse)

            # Draw the detected pothole on the image
            cv2.ellipse(image, ellipse, (0, 255, 0), 2)

    return potholes

if __name__ == "__main__":
    image= cv2.imread('pothole_image_2.png')
    potholes = detect_potholes(image)

    # Display the original image with detected potholes
    cv2.imshow("Detected Potholes", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
