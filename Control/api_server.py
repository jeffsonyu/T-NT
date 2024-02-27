import os
import cv2
import sys
import threading
import uvicorn
import time
import base64
import rospy
from geometry_msgs.msg import PoseStamped

from fastapi import FastAPI
from fastapi.responses import JSONResponse, RedirectResponse
from fastapi.encoders import jsonable_encoder

controller = FastAPI()
cam = None
cam_gel = None
last_marker_pos = [[0.5, 0, 0.3], [0, 0, 0, 1]]

def make_response(status_code, **kwargs):
    data = {'code': status_code, 'time+stamp': time.time()}
    data.update(**kwargs)
    json_compatible_data = jsonable_encoder(data)
    resp = JSONResponse(content=json_compatible_data, status_code=status_code)
    return resp


@controller.get("/")
def root():
    return RedirectResponse(url='/docs')

@controller.get('/v1/gel')
def get_gel():
    global cam
    rval, frame = cam.read()
    if rval:
        frame_compressed = base64.b64encode(cv2.imencode('.jpg', frame)[1].tobytes())
    else:
        frame_compressed = ''
    return make_response(status_code=200, ret=rval, blob=frame_compressed)

@controller.get('/v1/line_gel')
def get_line_gel():
    global cam_gel
    rval, frame = cam_gel.read()
    if rval:
        frame_compressed = base64.b64encode(cv2.imencode('.jpg', frame)[1].tobytes())
    else:
        frame_compressed = ''
    return make_response(status_code=200, ret=rval, blob=frame_compressed)


@controller.get('/v1/marker')
def get_marker():
    global last_marker_pos

    try:
        system_info = rospy.wait_for_message("/aruco_single/pose", PoseStamped, timeout=0.2)
        marker_pos = [system_info.pose.position.x, 
                        system_info.pose.position.y, 
                        system_info.pose.position.z]
        
        marker_quat = [system_info.pose.orientation.x, 
                        system_info.pose.orientation.y, 
                        system_info.pose.orientation.z, 
                        system_info.pose.orientation.w]

        last_marker_pos = [marker_pos, marker_quat]
    except:
        pass
    
    return make_response(status_code=200, blob=last_marker_pos)


def entrypoint(argv):
    global cam
    global cam_gel

    rospy.init_node('aruco', anonymous=True)

    cam_id = int(argv[0])
    cam = cv2.VideoCapture(cam_id)
    print("Kinova cam opened:", cam.isOpened())
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

    cam_gel_id = int(argv[1])
    cam_gel = cv2.VideoCapture(cam_gel_id)
    print("Gel cam opened:", cam_gel.isOpened())
    cam_gel.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cam_gel.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)


    try:
        thread = threading.Thread(target=uvicorn.run, kwargs={'app': controller, 'port': 8080 ,'host': '0.0.0.0'})
        thread.start()
        while True:
            time.sleep(86400)
        
    except KeyboardInterrupt:
        print(f"got KeyboardInterrupt")
        os._exit(1)


if __name__ == '__main__':
    import sys
    entrypoint(sys.argv[1:])