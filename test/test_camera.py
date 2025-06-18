import cv2
import numpy as np

# cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture('tcpclientsrc host=192.168.1.245 port=7001 ! decodebin ! queue ! videoconvert ! videoscale ! video/x-raw,width=1920,height=1080 ! appsink drop=1', cv2.CAP_GSTREAMER)

pipeline = 'gst-launch-1.0 -v tcpclientsrc host=192.168.1.245 port=7001 ! decodebin ! queue ! videoconvert ! videoscale ! video/x-raw,width=1280,height=720 ! autovideosink'
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print('Error: Unable to open pipeline')
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('Camera Feed', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# import cv2

# def list_available_cameras():
#     num_cameras = 10  # Maximum number of cameras to check
#     available_cameras = []
    
#     for i in range(num_cameras):
#         cap = cv2.VideoCapture(i)
#         if not cap.isOpened():
#             break
#         available_cameras.append(i)
#         cap.release()
    
#     return available_cameras

# if __name__ == "__main__":
#     cameras = list_available_cameras()
#     print("Available cameras:", cameras)

# import cv2

# cap = cv2.VideoCapture('autovideosrc ! videoconvert ! appsink')

# while True:
#     try:
#         ret, frame = cap.read()
#         cv2.imshow("Frame", frame)
#         if cv2.waitKey(1) == ord('q'):
#             break
#     except Exception as e:
#         print(e)
#         break

# cap.release()
# cv2.destroyAllWindows()

cmake -D CMAKE_BUILD_TYPE=Release \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_GSTREAMER=ON \
-D WITH_FFMPEG=ON \
-D WITH_LIBV4L=ON \
-D BUILD_opencv_python3=ON \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
-D PYTHON3_EXECUTABLE=$(which python3) \
-D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
-D PYTHON3_NUMPY_INCLUDE_DIRS=$(python3 -c "import numpy; print(numpy.get_include())") \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D INSTALL_C_EXAMPLES=OFF \
-D OPENCV_ENABLE_NONFREE=ON \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
-D BUILD_EXAMPLES=OFF \
-D BUILD_TESTS=OFF \
-D BUILD_PERF_TESTS=OFF \
..
