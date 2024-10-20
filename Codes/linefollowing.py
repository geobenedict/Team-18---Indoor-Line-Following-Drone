import cv2
import numpy as np
import requests

# Set up the ESP32-CAM stream URL
url = 'http://192.168.4.1/capture'
cap = cv2.VideoCapture(url)

hsvVals = [0, 0, 150, 179, 33, 255]

sensors = 3
threshold = 0.2
width, height = 480, 360
sensitivity = 3  # if number is high, less sensitive
weights = [-25, -15, 0, 15, 25]
fSpeed = 15
curve = 0


# Thresholding function to detect the line
def thresholding(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([hsvVals[0], hsvVals[1], hsvVals[2]])
    upper = np.array([hsvVals[3], hsvVals[4], hsvVals[5]])
    mask = cv2.inRange(hsv, lower, upper)
    return mask


# Detect contours and check if a T-junction is present
def getContours(imgThres, img):
    cx = 0
    isTjunction = False
    contours, hierarchy = cv2.findContours(imgThres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        biggest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(biggest)
        cx = x + w // 2
        cy = y + h // 2

        # Detect T-junction (if the width is significantly greater than height)
        if w / h > 2:  # T-junction condition: width greater than height
            isTjunction = True

        cv2.drawContours(img, biggest, -1, (255, 0, 255), 7)
        cv2.circle(img, (cx, cy), 10, (0, 255, 0), cv2.FILLED)

    return cx, isTjunction


# Simulate sensor outputs from the image
def getSensorOutput(imgThres, sensors):
    imgs = np.hsplit(imgThres, sensors)
    totalPixels = (img.shape[1] // sensors) * img.shape[0]
    senOut = []
    for x, im in enumerate(imgs):
        pixelCount = cv2.countNonZero(im)
        if pixelCount > threshold * totalPixels:
            senOut.append(1)
        else:
            senOut.append(0)
    return senOut


# Send movement commands to the ESP32-CAM
def sendCommands(senOut, cx, isTjunction):
    global curve
    lr = (cx - width // 2) // sensitivity
    lr = int(np.clip(lr, -10, 10))
    if 2 > lr > -2: lr = 0
    print(f'Translation Command (Left/Right): {lr}')

    # Curve adjustment based on sensor output
    if senOut == [1, 0, 0]:
        curve = weights[0]  # Strong left turn
    elif senOut == [1, 1, 0]:
        curve = weights[1]  # Moderate left turn
    elif senOut == [0, 1, 0]:
        curve = weights[2]  # Straight forward
    elif senOut == [0, 1, 1]:
        curve = weights[3]  # Moderate right turn
    elif senOut == [0, 0, 1]:
        curve = weights[4]  # Strong right turn
    else:
        curve = weights[2]  # Default to straight for ambiguous cases

    # Check if T-junction is detected
    if isTjunction:
        print("T-junction detected. Stopping the drone.")
        fSpeed = 0  # Stop the drone
    else:
        fSpeed = 15  # Continue forward movement

    # Print the curve and stop/fSpeed status for debugging
    print(f'Rotation Command (Curve): {curve}')
    print(f'Forward Speed: {fSpeed}')

    # Send the 'curve', 'lr', and 'fSpeed' values to ESP32
    data = {'curve': curve, 'lr': lr, 'fSpeed': fSpeed}
    url1 = 'http://192.168.4.1/commands'  # Update to correct URL for sending commands
    response = requests.post(url1, json=data)

    # Check response status
    if response.status_code == 200:
        print("Command sent successfully")
    else:
        print("Failed to send command, Status Code:", response.status_code)


# Main loop to capture video, process the image, and send commands
while True:
    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    img = cv2.imdecode(img_arr, -1)
    img = cv2.resize(img, (width, height))
    imgThres = thresholding(img)

    cx, isTjunction = getContours(imgThres, img)  # Detect contours and check for T-junction
    senOut = getSensorOutput(imgThres, sensors)
    sendCommands(senOut, cx, isTjunction)  # Stop if T-junction is detected

    # Display the processed images for debugging
    cv2.imshow("Output", img)
    cv2.imshow("Path", imgThres)
    cv2.waitKey(1)
    if cv2.waitKey(1) and 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
