import airsim
import cv2
import numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()



while True:
    # We use simGetImage (singular) to target the camera by index
    # Camera 0 is the default front-facing camera
    # ImageType 7 is Infrared
    response_raw = client.simGetImage("0", airsim.ImageType.Infrared, vehicle_name="PX4")
    
    if not response_raw:
        continue

    # Convert the raw string data into a numpy array
    img1d = np.frombuffer(response_raw, dtype=np.uint8)
    
    # Check if the buffer is empty
    if img1d.size == 0:
        continue

    # Decode the image buffer
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_UNCHANGED)

    gray_img = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    
    # 2. Apply a Thermal Palette (COLORMAP_JET or COLORMAP_INFERNO)
    thermal_img = cv2.applyColorMap(gray_img, cv2.COLORMAP_JET)

    # 3. Display the "Human Readable" thermal feed
    cv2.imshow("Thermal Visualization", thermal_img)

    if img_rgb is not None:
        cv2.imshow("Direct Thermal Feed", img_rgb)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()