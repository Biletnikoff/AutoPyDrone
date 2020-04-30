
import cv2
from hud import get_hud

# Windows dependencies
# - Python 2.7.6: http://www.python.org/download/
# - OpenCV: http://opencv.org/
# - Numpy -- get numpy from here because the official builds don't support x64:
#   http://www.lfd.uci.edu/~gohlke/pythonlibs/#numpy

# Mac Dependencies
# - brew install python
# - pip install numpy
# - brew tap homebrew/science
# - brew install opencv

frame_idx = 0
action_str = "centered"
dimensions = (960, 720)

cWidth = int((dimensions[0] / 2) + 140)
cHeight = int((dimensions[1] / 2) - 20)

cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

while rval:
    cv2.imshow("preview", frame)
    rval, frame = vc.read()
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break
    else:
        frame_idx += 1
        get_hud(frame, idx=frame_idx, action=action_str)
        cv2.circle(frame, (cWidth, cHeight), 10, (0, 0, 255), 2)

vc.release()
cv2.destroyWindow("preview")