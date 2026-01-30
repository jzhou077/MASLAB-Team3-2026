import cv2
import utils.constants as constants
import camera_utils
import numpy as np

class Camera:

    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, constants.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, constants.FRAME_HEIGHT)


    def get_frame(self, writeToFile=False, filename="frame.jpg"):
        """
        Gets a frame from the camera.

        writeToFile: writes the frame to a jpg file if true
        filename: file that the frame is written to 
        """
        ret, frame = self.cap.read()
        
        if not ret:
            raise RuntimeError("Cannot grab frame")
        
        if writeToFile:
            cv2.imwrite(f"debug_imgs/{filename}", frame)

        return frame
    
    def show_stream(self):
        """
        Shows live video stream
        """
        print("Press q to quit")

        while True:
            frame = self.get_frame()

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) == ord('q'):
                break
        
    def detect_cans(self, frame, writeToFile=False, filename="cans.jpg"):
        """
        Finds cans using color detection.

        Returns:
            detected_cans: list of dicts, each dict has:
                - 'color': str
                - 'orient': str
                - 'bottom_coords': (x, y)
                - 'bbox': (x, y, w, h)
                - 'angle': float
                - 'stats': (red_frac, green_frac, yellow_frac, ratio, theta)
                - 'contour': numpy.ndarray
        """

        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 200)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0.0)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, constants.WHITE_BAL_TEMP)

        # frame = self.get_frame()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsv, constants.LOWER_RED1, constants.UPPER_RED1) | cv2.inRange(hsv, constants.LOWER_RED2, constants.UPPER_RED2)
        green_mask = cv2.inRange(hsv, constants.LOWER_GREEN, constants.UPPER_GREEN)
        yellow_mask = cv2.inRange(hsv, constants.LOWER_YELLOW, constants.UPPER_YELLOW)
        black_mask = cv2.inRange(hsv, constants.LOWER_BLACK, constants.UPPER_BLACK)

        # Clean masks
        kernel = camera_utils.make_kernel(5)
        red_mask = camera_utils.clean_mask(red_mask, kernel)
        green_mask = camera_utils.clean_mask(green_mask, kernel)
        yellow_mask = camera_utils.clean_mask(yellow_mask, kernel)

        # Masks excluding the ones that are surrounded by black
        kernel = np.ones((5, 5), np.uint8)
        black_mask_dilated = cv2.dilate(black_mask, kernel, iterations=2) # increase iterations to expand black's influence
        red_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(black_mask_dilated))
        green_mask = cv2.bitwise_and(green_mask, cv2.bitwise_not(black_mask_dilated))
        yellow_mask = cv2.bitwise_and(yellow_mask, cv2.bitwise_not(black_mask_dilated))

        # Combined mask
        union_mask = cv2.bitwise_or(red_mask, cv2.bitwise_or(green_mask, yellow_mask))
        
        contours, _ = cv2.findContours(union_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Cans: Found {len(contours)} contours")

        detected_cans = list()

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            
            # Gets rid of potentially thin lines
            if w < 20 or h < 20:
                continue
                
            area = cv2.contourArea(cnt)
            if area < 200 or area > 250000:
                continue

            
            # looking at black ratio inside and outside the contour -------
            pad = 10
            x0 = max(0, x - pad)
            y0 = max(0, y - pad)
            x1 = min(frame.shape[1], x + w + pad)
            y1 = min(frame.shape[0], y + h + pad)

            # mask for the contour itself
            contour_mask = np.zeros_like(black_mask)
            cv2.drawContours(contour_mask, [cnt], -1, 255, -1)

            # black inside contour
            black_inside = cv2.bitwise_and(black_mask, contour_mask)
            black_ratio_inside = cv2.countNonZero(black_inside) / max(cv2.countNonZero(contour_mask), 1)
            # print(f"Black ratio inside contour: {black_ratio_inside:.2f}")

            # black outside contour
            roi_black_outside = black_mask[y0:y1, x0:x1].copy()
            roi_contour_mask = contour_mask[y0:y1, x0:x1]
            roi_black_outside = cv2.bitwise_and(roi_black_outside, cv2.bitwise_not(roi_contour_mask))
            black_ratio_outside = cv2.countNonZero(roi_black_outside) / roi_black_outside.size
            # print(f"Black ratio outside contour: {black_ratio_outside:.2f}")

            if black_ratio_inside > constants.BLACK_RATIO_REJECT or black_ratio_outside > constants.BLACK_RATIO_REJECT:
                continue

            # Determining can color
            red_frac, green_frac, yellow_frac = camera_utils.contour_color_fractions(cnt, red_mask, green_mask, yellow_mask)
            color_label = "UNKNOWN"
            color_label = camera_utils.decide_color(red_frac, green_frac, yellow_frac)

            # Determining can orientation
            rect = cv2.minAreaRect(cnt)
            orient_label, ratio, theta = camera_utils.orientation_from_min_area_rect(rect)

            bottom_coords = ((x+(x+w))//2, y + h)

            detected_cans.append({
                'color': color_label,
                'orient': orient_label,
                'bottom_coords': bottom_coords,
                'bbox': (x, y, w, h),
                'angle': theta,
                'stats': (red_frac, green_frac, yellow_frac, ratio, theta),
                'contour': cnt
            })

            if writeToFile:
                cv2.drawContours(frame, [cnt], -1, (0, 0, 0), 2)  # contour in blue
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 2)
                bx, by = bottom_coords
                cv2.circle(frame, (bx, by), 5, (0, 0, 0), -1)
                cv2.putText(frame, f"Can: {color_label}", (bx+5, by+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                cv2.putText(frame, f"{orient_label}", (bx+5, by+35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                cv2.putText(frame, f"R:{red_frac:.2f} G:{green_frac:.2f} Y:{yellow_frac:.2f}", 
                            (bx+5, by+50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        
        if writeToFile:
            cv2.imwrite(f"debug_imgs/{filename}", frame)

        return detected_cans

    def detect_goals(self, frame, writeToFile=False, filename="goals.jpg"):
        """
        Finds scoring zones using color detection.

        Returns:
            goals: a list of dicts with the following structure
                - 'color': str
                - 'center_uv' (center of goal): str
                - 'bbox': (x, y, w, h)
                - 'area': int
        """

        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -7)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0.0)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3000)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        
        # Black mask
        color_range = np.max(frame, axis=2) - np.min(frame, axis=2)
        range_thres = np.where(color_range < 20, 255, 0).astype(np.uint8)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_black = cv2.inRange(hsv, constants.LOWER_BLACK, constants.UPPER_BLACK)
        black_mask = cv2.bitwise_and(hsv_black, range_thres)
        black_mask = cv2.dilate(black_mask, kernel, iterations=4)
        black_mask = cv2.erode(black_mask, kernel, iterations=1)

        # Combining colored masks with black masks
        masks = {
            "Red": cv2.inRange(hsv, constants.LOWER_RED1, constants.UPPER_RED1) | cv2.inRange(hsv, constants.LOWER_RED2, constants.UPPER_RED2),
            "Green": cv2.inRange(hsv, constants.LOWER_GREEN, constants.UPPER_GREEN),
            "Yellow": cv2.inRange(hsv, constants.LOWER_YELLOW, constants.UPPER_YELLOW)
        }

        for color, mask in masks.items():
            masks[color] = cv2.dilate(masks[color], kernel, iterations=2)
            masks[color] = cv2.erode(masks[color], kernel, iterations=1)

            masks[color] = cv2.bitwise_and(black_mask, masks[color])

        # Getting goals
        goals = []

        for color, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                print("Area of zone detected:", area, " Color:", color)

                if color == "Yellow":
                    if area < 200:
                        continue
                else:
                    if area < 3000:
                        continue
                        
                x, y, w, h = cv2.boundingRect(cnt)

                # ignore 5% top-of-frame detections
                if y < constants.FRAME_HEIGHT * 0.05:
                    continue
                
                # Finding centroid of contour
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                goals.append({
                    "color": color,
                    "center_uv": (cx, cy),
                    "bbox": (x, y, w, h),
                    "area": area
                })

                if writeToFile:
                    cv2.drawContours(frame, [cnt], -1, (0,0,0), 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 2)
                    cv2.putText(frame, f"Zone: {color}", ((x+(x+w))//2+5, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        if writeToFile:
            cv2.imwrite(f"debug_imgs/{filename}", frame)

        return goals