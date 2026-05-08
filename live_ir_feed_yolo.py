import airsim
import cv2
import numpy as np
from ultralytics import YOLO
import math
import time
import csv

LOG_PATH = "detection_logs/detection_log_" + time.strftime("%Y%m%d-%H%M%S") + ".csv"
with open(LOG_PATH, "a", newline="") as f:
    csv.writer(f).writerow(["timestamp","class","conf","track_id","dog_n","dog_e","drone_alt"])

last_log_time = 0
LOG_COOLDOWN = 8.0  # seconds


model = YOLO("best.pt")

client = airsim.MultirotorClient()
client.confirmConnection()

cv2.namedWindow("YOLO on Thermal Feed", cv2.WINDOW_NORMAL)
cv2.resizeWindow("YOLO on Thermal Feed", 1280, 720)

# ground-projection constants
ALTITUDE = 20
FOV_H = math.radians(90)
IMG_W = 1280
IMG_H = 720
meters_per_pixel = (2 * ALTITUDE * math.tan(FOV_H / 2)) / IMG_W

def make_thermal_realistic(thermal_bgr):
    # add gaussian noise (simulates sensor noise)
    noise = np.random.normal(0, 8, thermal_bgr.shape).astype(np.int16)
    noisy = np.clip(thermal_bgr.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    # slight blur (simulates thermal camera optics + low resolution feel)
    blurred = cv2.GaussianBlur(noisy, (5, 5), 1.5)
    return blurred

def thermal_with_white_hot(gray, white_threshold=200):
    thermal = cv2.applyColorMap(gray, cv2.COLORMAP_JET) # apply JET
    hot_mask = gray > white_threshold # bool array locating greyscale pixels above threshold
    thermal[hot_mask] = [255, 255, 255] # set them to white in the thermal image
    return thermal

def thermal_with_red_gradient(gray, white_threshold=200, halo_strength=80):
    # blur for edge softness
    gray_blurred = cv2.GaussianBlur(gray, (5, 5), 1.0) # 5x5 kernel

    # identify the hot region
    hot_mask = gray_blurred > white_threshold
    hot_mask_u8 = hot_mask.astype(np.uint8) * 255 # convert bool to uint8 for distance transform

    # blur the hot mask to create a glow outside the dog
    halo = cv2.GaussianBlur(hot_mask_u8, (31, 31), 8) # big kernel for a halo around the dog
    halo = (halo * (halo_strength / 255.0)).astype(np.uint8) # scale down from the white hot
    enhanced_gray = np.maximum(gray_blurred, halo) 

    # JET on the new gray
    thermal = cv2.applyColorMap(enhanced_gray, cv2.COLORMAP_JET)

    # white at center, fading to red at edges
    if hot_mask.any():
        dist = cv2.distanceTransform(hot_mask_u8, cv2.DIST_L2, 5) # distance from hot region, in pixels
        if dist.max() > 0:
            depth = dist / dist.max()                 # 0 at edge, 1 at center
            depth_3 = np.stack([depth] * 3, axis=-1)
            red = np.array([0, 0, 255], dtype=np.float32)
            white = np.array([255, 255, 255], dtype=np.float32)
            blended = red * (1 - depth_3) + white * depth_3
            thermal[hot_mask] = blended[hot_mask].astype(np.uint8)

    return thermal

def get_drone_ned(client, vehicle_name="PX4"):
    raw = client.client.call('simGetVehiclePose', vehicle_name)
    pos = raw['position'] if isinstance(raw, dict) else raw[0]
    if isinstance(pos, list):
        return pos[0], pos[1], pos[2]
    return pos['x_val'], pos['y_val'], pos['z_val']

while True:
    response_raw = client.simGetImage("front_center", airsim.ImageType.Infrared, vehicle_name="PX4")

    if not response_raw:
        continue

    img1d = np.frombuffer(response_raw, dtype=np.uint8)
    if img1d.size == 0:
        continue

    img_bgr = cv2.imdecode(img1d, cv2.IMREAD_UNCHANGED)
    if img_bgr is None:
        continue

    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    # thermal = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    thermal = thermal_with_red_gradient(gray) # take raw grey output and run through function to make it look realistic

    # make realistic thermal feed
    thermal = make_thermal_realistic(thermal) # noise and blur

    # YOLO inference + draw boxes
    results = model.track(thermal, verbose=False, conf=0.30, imgsz=736, persist=True, tracker="byte_track_custom.yaml", classes = [0])
    annotated = results[0].plot(labels = False)

    if results[0].boxes:
        try:
            x, y, z = get_drone_ned(client)
        except Exception as e:
            print(f"pose fetch failed: {e}")
            x, y, z = None, None, None

        for box in results[0].boxes:
            if box.id is None:
                continue

            track_id = int(box.id[0])
            conf = float(box.conf[0])
            if conf < 0.8:
                continue
            if time.time() - last_log_time < LOG_COOLDOWN:
                continue
            last_log_time = time.time()

            x1b, y1b, x2b, y2b = box.xyxy[0].cpu().numpy()
            cx = (x1b + x2b) / 2
            cy = (y1b + y2b) / 2
            offset_n = (IMG_H/2 - cy) * meters_per_pixel  # top of image = north
            offset_e = (cx - IMG_W/2)  * meters_per_pixel  # right of image = east

            dog_n = x + offset_n if x is not None else None
            dog_e = y + offset_e if y is not None else None
            drone_alt = -z if z is not None else None  # NED z is negative-up


            cls = int(box.cls[0])
            name = model.names[cls]
            with open(LOG_PATH, "a", newline="") as f:
                csv.writer(f).writerow([
                    time.time(), name, f"{conf:.3f}", track_id,
                    f"{dog_n:.2f}" if dog_n is not None else "",
                    f"{dog_e:.2f}" if dog_e is not None else "",
                    f"{drone_alt:.2f}" if drone_alt is not None else "",
                ])

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            # class + confidence
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = f"{model.names[cls]} {conf:.2f}"
            cv2.putText(annotated, label, (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # size estimate
            w_px = x2 - x1
            h_px = y2 - y1
            est_length_m = max(w_px, h_px) * meters_per_pixel
            cv2.putText(annotated, f"Size: {est_length_m:.2f} m",
                        (int(x1), int(y2) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


    # print(f"image dims: {img_bgr.shape}")

    cv2.imshow("YOLO on Thermal Feed", annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()