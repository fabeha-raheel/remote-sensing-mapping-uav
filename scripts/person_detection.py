import cv2
import numpy as np
import tensorflow as tf

# Load the pre-trained MobileNet SSD model
model = tf.saved_model.load('ssd_mobilenet_v2_coco')

# Function to perform object detection using MobileNet SSD
def detect_objects(frame):
    # Resize frame to 300x300 (required input size for MobileNet SSD)
    input_frame = cv2.resize(frame, (300, 300))

    # Preprocess frame: normalize pixel values and convert to batch
    input_tensor = np.expand_dims(input_frame, axis=0)
    input_tensor = tf.convert_to_tensor(input_tensor, dtype=tf.float32)

    # Perform inference to detect objects
    detections = model(input_tensor)

    # Parse the detection results
    for i in range(int(detections['num_detections'][0])):
        class_id = int(detections['detection_classes'][0][i])
        score = float(detections['detection_scores'][0][i])
        bbox = detections['detection_boxes'][0][i]

        # Class ID for person in COCO dataset is 1
        if class_id == 1 and score > 0.5:
            h, w, _ = frame.shape
            ymin, xmin, ymax, xmax = bbox.numpy()
            xmin = int(xmin * w)
            xmax = int(xmax * w)
            ymin = int(ymin * h)
            ymax = int(ymax * h)

            # Draw rectangle around the detected person
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)

    return frame

# Function to process a video file
def process_video(video_path, output_path):
    # Open the video file
    cap = cv2.VideoCapture(video_path)

    # Check if the video opened successfully
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    # Get the frame width, height, and frames per second (fps)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    # Define the codec and create VideoWriter object
    # Adjust parameters as needed (e.g., filename, codec, frame size)
    codec = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' codec for MP4 format
    out = cv2.VideoWriter(output_path, codec, fps, (frame_width, frame_height))

    # Process each frame in the video
    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        # Detect objects in the frame
        frame_with_detections = detect_objects(frame)

        # Write the frame with detections to the output video
        out.write(frame_with_detections)

        # Display the resulting frame
        cv2.imshow('Object Detection', frame_with_detections)

        # Press 'q' on keyboard to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the VideoCapture and VideoWriter objects
    cap.release()
    out.release()

    # Close all OpenCV windows
    cv2.destroyAllWindows()

# Specify paths to input video and output video
input_video_path = '/home/ugv/rtab_ws/src/remote-sensing-mapping-uav/output_video.avi'
output_video_path = '/home/ugv/rtab_ws/src/remote-sensing-mapping-uav/output_video.mp4'

# Process the input video and save the output with detections
process_video(input_video_path, output_video_path)
