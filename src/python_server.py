from flask import Flask, request, jsonify
import cv2
import numpy as np
import time
import os
from datetime import datetime

app = Flask(__name__)

# Debug settings
DEBUG_MODE = True
DEBUG_IMAGE_SAVE_PATH = "./debug_images"
os.makedirs(DEBUG_IMAGE_SAVE_PATH, exist_ok=True)  # Create folder if doesn't exist

@app.route('/detect', methods=['POST'])
def detect():
    try:
        # Check if request contains image data
        if not request.data:
            return jsonify({"error": "No image data received"}), 400

        # Decode image
        img_array = np.frombuffer(request.data, np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        
        if img is None:
            return jsonify({"error": "Failed to decode image"}), 400

        # Save original debug image
        if DEBUG_MODE:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            original_path = os.path.join(DEBUG_IMAGE_SAVE_PATH, f"original_{timestamp}.jpg")
            cv2.imwrite(original_path, img)
            print(f"Saved original image to: {original_path}")

        # Enhanced car detection
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours and draw on debug image
        min_contour_area = 500
        car_count = 0
        debug_img = img.copy()
        
        for cnt in contours:
            if cv2.contourArea(cnt) > min_contour_area:
                car_count += 1
                # Draw bounding boxes on debug image
                x,y,w,h = cv2.boundingRect(cnt)
                cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0,255,0), 2)

        # Save processed debug image
        if DEBUG_MODE:
            processed_path = os.path.join(DEBUG_IMAGE_SAVE_PATH, f"processed_{timestamp}.jpg")
            cv2.imwrite(processed_path, debug_img)
            print(f"Saved processed image to: {processed_path}")

        # Response
        response = {
            "car_count": car_count,
            "timestamp": time.time(),
            "debug_images": {
                "original": f"original_{timestamp}.jpg",
                "processed": f"processed_{timestamp}.jpg"
            } if DEBUG_MODE else None
        }

        return jsonify(response)

    except Exception as e:
        print(f"Error processing image: {str(e)}")
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=4999, debug=True)