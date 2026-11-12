# usage: bag2asl.py <bag_dir> <out_dir>  -> out/cam0/data/<ns>.png, out/cam0/data.csv, out/imu0/data.csv (header stamps)
import sys, os, cv2, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu
bag, out = sys.argv[1], sys.argv[2]
os.makedirs(f"{out}/cam0/data", exist_ok=True); os.makedirs(f"{out}/imu0", exist_ok=True)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
r.set_filter(rosbag2_py.StorageFilter(topics=["/cam0/image_raw", "/imu0"]))
fc = open(f"{out}/cam0/data.csv", "w"); fi = open(f"{out}/imu0/data.csv", "w")
fc.write("#timestamp [ns],filename\n"); fi.write("#timestamp [ns],wx,wy,wz,ax,ay,az\n")
nc = ni = 0
while r.has_next():
    topic, data, _ = r.read_next()
    if topic == "/imu0":
        m = deserialize_message(data, Imu); ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        w, a = m.angular_velocity, m.linear_acceleration
        fi.write(f"{ns},{w.x!r},{w.y!r},{w.z!r},{a.x!r},{a.y!r},{a.z!r}\n"); ni += 1
    else:
        m = deserialize_message(data, Image); ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        assert m.encoding == "mono8", m.encoding
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]
        cv2.imwrite(f"{out}/cam0/data/{ns}.png", img); fc.write(f"{ns},{ns}.png\n"); nc += 1
print(out, "images", nc, "imu", ni)
