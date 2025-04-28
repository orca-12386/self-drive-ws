import cv2
import numpy as np

def nothing(x):
    pass

def main():
    # Read the image
    image = cv2.imread('saved_image.png')
    if image is None:
        print("Error: Image not found!")
        return

    # Create a window
    cv2.namedWindow('HSV Thresholding')

    # Create trackbars for lower HSV
    cv2.createTrackbar('Low H', 'HSV Thresholding', 20, 179, nothing)
    cv2.createTrackbar('Low S', 'HSV Thresholding', 100, 255, nothing)
    cv2.createTrackbar('Low V', 'HSV Thresholding', 100, 255, nothing)

    # Create trackbars for upper HSV
    cv2.createTrackbar('High H', 'HSV Thresholding', 30, 179, nothing)
    cv2.createTrackbar('High S', 'HSV Thresholding', 255, 255, nothing)
    cv2.createTrackbar('High V', 'HSV Thresholding', 255, 255, nothing)

    while True:
        # Get current positions of all trackbars
        low_h = cv2.getTrackbarPos('Low H', 'HSV Thresholding')
        low_s = cv2.getTrackbarPos('Low S', 'HSV Thresholding')
        low_v = cv2.getTrackbarPos('Low V', 'HSV Thresholding')

        high_h = cv2.getTrackbarPos('High H', 'HSV Thresholding')
        high_s = cv2.getTrackbarPos('High S', 'HSV Thresholding')
        high_v = cv2.getTrackbarPos('High V', 'HSV Thresholding')

        # Convert the image to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only desired colors
        lower_yellow = np.array([low_h, low_s, low_v])
        upper_yellow = np.array([high_h, high_s, high_v])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        result = cv2.bitwise_and(image, image, mask=mask)

        # Show the original and result
        cv2.imshow('Original Image', image)
        cv2.imshow('Mask', mask)
        cv2.imshow('Detected Color', result)

        # Break the loop when user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
