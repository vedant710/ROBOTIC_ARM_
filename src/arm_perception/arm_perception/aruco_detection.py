import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import  Node
from rclpy.qos  import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg  import CameraInfo,  Image
from tf2_ros import Buffer, TransformListener, TransformException



def make_T(R, t):
    T  = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def quat_to_mat(q):               # function for quaternions to matrix
    x, y, z, w = q
    n = x*x + y*y + z*z + w*w
    s = 2.0 / n                   # dividig quaternions that arent unit length
    return np.array(
        [
        [1 - s*(y*y + z*z), s*(x*y - z*w),     s*(x*z + y*w)],
        [s*(x*y + z*w),     1 - s*(x*x + z*z), s*(y*z - x*w)],
        [s*(x*z - y*w),     s*(y*z + x*w),     1 - s*(x*x + y*y)]
        ]
    )          
 
def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)

class  ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__("detect_aruco")
        


#settings params
        self.declare_parameter("image_topic", "/overhead_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/overhead_camera/camera_info")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_length", 0.057)
        self.declare_parameter("supported_ids", [0, 1, 2])
        self.declare_parameter("max_reproj_px", 3.0)
            
        gp = lambda name: self.get_parameter(name).value
        self.target_frame = gp("target_frame")
        self.marker_length = gp("marker_length")
        self.supported_ids = list(gp ("supported_ids"))
        self.max_reproj =gp("max_reproj_px")
        


#aruco detector
        self.dictionary = cv.aruco.getPredefinedDictionary(getattr(cv.aruco, gp ("dictionary")))
        self.new_api =  hasattr(cv.aruco, "ArucoDetector")
        if self.new_api:
            self.params = cv.aruco.DetectorParameters()
            self.params.minMarkerDistanceRate = 0.01
            self.detector = cv.aruco.ArucoDetector(self.dictionary, self.params)
        else:
            self.params = cv.aruco.DetectorParameters_create()
            self.params.minMarkerDistanceRate = 0.01

# camera numbers to be filled after
        self.K = None
        self.D = None

# TF2 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  

        self.bridge = CvBridge()

#marker corners in its own frame
        h = self.marker_length / 2.0
        self.obj_pts  = np.array([
                        [-h,  h, 0],
                        [ h,  h, 0],
                        [ h, -h, 0],
                        [-h, -h, 0]], dtype=np.float32)


#inputs and outputs
        self.create_subscription(CameraInfo, gp("camera_info_topic"), self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, gp("image_topic"), self.on_image, qos_profile_sensor_data)  
        self.pub_pose = self.create_publisher(PoseStamped, "/perception/marker_pose", 10)
        self.pub_debug = self.create_publisher(Image, "/perception/debug_image", 10)
        
        self.get_logger().info("detect_aruco ready")



    def on_info(self, msg):
        self.K = np.array (msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
        self.get_logger().info(f"K =\n{self.K}", once=True)
    
    def on_image(self, msg):
        if self.K  is None:
            self.get_logger().warn("no camera info yet", throttle_duration_sec= 2.0)
            return

        self.get_logger().info(f"frame {msg.width}x{msg.height} {msg.encoding}",
                               throttle_duration_sec=2.0)

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        


        if self.new_api:
            corners, ids,  _= self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)

        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)

        debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        debug_msg.header = msg.header
        self.pub_debug.publish(debug_msg)

        if ids is None:
            return

        self.get_logger().info(f"saw IDs: {ids.flatten().tolist()}", throttle_duration_sec=1.0)

        for c, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            if marker_id not in self.supported_ids:
                continue

            img_pts = c.reshape(4, 2).astype(np.float32)
            ok,  rvec, tvec = cv.solvePnP(self.obj_pts, img_pts, self.K, self.D, 
                                          flags=cv.SOLVEPNP_IPPE_SQUARE)

            if not ok:
                continue

            self.get_logger().info(f"ID {marker_id}: t = {tvec.flatten().round(3)} m")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()