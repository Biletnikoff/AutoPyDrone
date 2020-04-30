
import cv2.cv2 as cv2
from hud import get_hud
import numpy as np
import time
import datetime
import constants

def run(drone, args, lerp, ddir, face_cascade):

    if not drone.tello.connect():
        print("Tello not connected")
        return

    if not drone.tello.set_speed(drone.speed):
        print("Not set speed to lowest possible")
        return

    # In case streaming is on. This happens when we quit this program without the escape key.
    if not drone.tello.streamoff():
        print("Could not stop video stream")
        return

    if not drone.tello.streamon():
        print("Could not start video stream")
        return

    frame_read = drone.tello.get_frame_read()
    drone.tello.get_battery()

    imgCount = 0
    scan = 0
    frame_idx = 0
    OVERRIDE = False
    should_stop = False
    override_speed = args.override_speed
    target_distance = args.distance
    action_str = 'Searching For Target'

    safety_zone_x = args.saftey_x
    safety_zone_y = args.saftey_y

    if args.debug:
        print("DEBUG MODE ENABLED!")

    while not should_stop:
        drone.update()

        if frame_read.stopped:
            frame_read.stop()
            break

        current_time = str(datetime.datetime.now()).replace(':', '-').replace('.', '_')

        frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)
        drone_frame = frame_read.frame

        vid = drone.tello.get_video_capture()

        if args.save_session:
            cv2.imwrite("{}/tellocap{}.jpg".format(ddir, imgCount), drone_frame)

        frame = np.rot90(frame)
        imgCount += 1

        time.sleep(1 / constants.FPS)

        # Listen for key presses
        keyboard = cv2.waitKey(20)
        if keyboard == ord('t'):
            if not args.debug:
                print("Lifting Off")
                drone.tello.takeoff()
                drone.tello.get_battery()
            drone.send_rc_control = True

        if keyboard == ord('l'):
            if not args.debug:
                print("Landing")
                drone.tello.land()
            drone.send_rc_control = False

        if keyboard == 8:
            if not OVERRIDE:
                OVERRIDE = True
                print("OVERRIDE ENABLED")
            else:
                OVERRIDE = False
                print("OVERRIDE DISABLED")

        if keyboard == 27:
            should_stop = True
            break
        gray = cv2.cvtColor(drone_frame, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.05, minNeighbors=3) # Detects face returns an array

        # scaleFactor – Parameter specifying how much the image size is reduced at each image scale.
        # 1.05 is a good possible value for this, which means you use a small step for resizing, i.e. reduce size by 5%, you increase the chance of a matching size with the model for detection is found.
        # This also means that the algorithm works slower since it is more thorough. You may increase it to as much as 1.4 for faster detection, with the risk of missing some faces altogether.
        #
        # minNeighbors – Parameter specifying how many neighbors each candidate rectangle should have to retain it.
        #
        # This parameter will affect the quality of the detected faces. Higher value results in less detections but with higher quality. 3~6 is a good value for it.
        #
        # minSize – Minimum possible object size. Objects smaller than that are ignored.
        #
        # This parameter determine how small size you want to detect. You decide it! Usually, [30, 30] is a good start for face detection.
        #
        # maxSize – Maximum possible object size. Objects bigger than this are ignored.

        target_face_size = constants.OPENCV_FACE_SIZES[target_distance]

        # These are our center drone_window_dimensions
        noFaces = len(faces) == 0
        bounding_box_size = 0
        drone_window_center_width = int((constants.DRONE_OBERSERVATION_WINDOW_DIMENSIONS[0] / 2) - 20)
        drone_window_center_height = int((constants.DRONE_OBERSERVATION_WINDOW_DIMENSIONS[1] / 2) - 20)
        drone_window_center_x = drone_window_center_width
        drone_window_center_y = drone_window_center_height

        if drone.send_rc_control and not OVERRIDE:
            frame_idx += 1

            for (x, y, w, h) in faces:


                roi_gray = gray[y:y + h, x:x + w]
                roi_color = drone_frame[y:y + h, x:x + w]
                action_str = "TARGET FOUND"

                face_box_col = (255, 0, 0)
                face_box_stroke = 2

                bounding_box_x = x + w
                bounding_box_y = y + h
                bounding_box_size = w * 2

                target_x = int((bounding_box_x + x) / 2)
                target_y = int((bounding_box_y + y) / 2) + constants.UDOFFSET

                true_center_vector = np.array((drone_window_center_width, drone_window_center_height, target_face_size))
                true_target_vector = np.array((target_x, target_y, bounding_box_size))
                distance_vector = true_center_vector - true_target_vector

                dist_error = target_face_size - w
                dist_control = drone.dist_pid.control(dist_error)

                if not args.debug:
                    offset_x = target_x - drone_window_center_x
                    h_control = drone.h_pid.control(offset_x)
                    drone.yaw_velocity = h_control
                    scan = h_control

                    offset_y = target_y - drone_window_center_y
                    v_control = drone.v_pid.control(-offset_y)
                    drone.up_down_velocity = v_control

                    drone.for_back_velocity = dist_control
                    print('-----dist_control', dist_control)
                    print('-----dist_error', dist_error)
                    print("offset=(%d,%d), cur_size=%d, size_error=%d, h_control=%f" %
                          (offset_x, offset_y, w, dist_error, h_control))

                cv2.rectangle(drone_frame, (x, y), (bounding_box_x, bounding_box_y), face_box_col, face_box_stroke)
                cv2.circle(drone_frame, (target_x, target_y), 10, (0, 255, 0), 2)

                # Draw the safety zone
                # cv2.rectangle(drone_frame, (target_x - safety_zone_x, target_y - safety_zone_y),
                #               (target_x + safety_zone_x, target_y + safety_zone_y), (0, 255, 0), face_box_stroke)

                cv2.putText(drone_frame, str(distance_vector), (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            if noFaces:
                print(bounding_box_size, target_distance)
                drone.h_pid.reset()
                drone.v_pid.reset()
                drone.dist_pid.reset()
                drone.yaw_velocity = scan
                drone.up_down_velocity = 0
                drone.for_back_velocity = 0
                action_str = "No Target"
                print("NO TARGET")

        # Draw the center of screen circle, this is what the drone tries to match with the target coords
        cv2.circle(drone_frame, (drone_window_center_width, drone_window_center_height), 10, (0, 0, 255), 2)
        get_hud(drone_frame, idx=frame_idx, action=action_str)
        dCol = lerp(np.array((0, 0, 255)), np.array((255, 255, 255)), target_distance + 1 / 7)

        if OVERRIDE:
            text = "User Control: {}".format(override_speed)
            dCol = (255, 255, 255)
        else:
            text = "AI Control: {}".format(str(target_distance))

        cv2.putText(drone_frame, text, (31, 665), cv2.FONT_HERSHEY_SIMPLEX, 1, dCol, 2)
        cv2.imshow(f'Drone Tracking...', drone_frame)

    drone.tello.get_battery()
    cv2.destroyAllWindows()
    drone.tello.end()
