from flask import Flask, request
from ultralytics import YOLO
import os
from datetime import datetime

# --- CONFIGURATION ---
UPLOAD_FOLDER = 'theft_evidence'
os.makedirs(UPLOAD_FOLDER, exist_ok=True) # <--- THIS PREVENTS THE CRASH

app = Flask(__name__)

# Load the Brain (YOLOv8 Nano Model - Fast & Accurate)
print("🧠 LOADING AI MODEL... (This might take a minute)")
model = YOLO("yolov8n.pt") 
print("✅ AI READY.")

@app.route('/upload', methods=['POST'])
def upload_file():
    if 'imageFile' not in request.files:
        return "No imageFile provided", 400

    file = request.files['imageFile']
    
    if file.filename == '':
        return "No selected file", 400

    if file:
        # 1. Generate a Time-Stamped Filename
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = f"{timestamp}.jpg"
        filepath = os.path.join(UPLOAD_FOLDER, filename)
        
        # 2. Save the Photo
        file.save(filepath)
        
        # 3. AI Analysis (The Logic)
        # We tell YOLO to look at the image and identify objects
        results = model(filepath, verbose=False) 
        
        person_detected = False
        
        # Loop through everything the AI saw
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0]) # Get the object ID
                
                # Class 0 is ALWAYS "Person" in YOLO
                if class_id == 0:
                    person_detected = True
                    break # Stop looking if we found a thief
        
        # 4. The Verdict
        if person_detected:
            print(f"🚨 THEFT CAUGHT! Saved: {filename}")
            # Here you could add code to email the owner or sound a louder alarm
        else:
            print(f"✅ False Alarm (No Human Detected). Saved: {filename}")

        return "Image Received", 200

if __name__ == '__main__':
    # 0.0.0.0 is CRITICAL. It lets the ESP32 talk to your laptop.
    app.run(host='0.0.0.0', port=5000, debug=False)
