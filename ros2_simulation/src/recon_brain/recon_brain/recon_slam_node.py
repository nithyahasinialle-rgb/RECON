#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from recon_msgs.msg import CSIData
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import math
from scipy.signal import find_peaks

CALIB_SAMPLES=100; YAW_DEADBAND=0.05; CSI_BG_ALPHA=0.95
CSI_PEAK_THRESH=0.35; CSI_PEAK_DIST=8
CSI_MIN_DIST_M=0.5; CSI_MAX_DIST_M=12.0
CSI_FAN_HALF_ANGLE=0.4; CSI_FAN_RAYS=18
CSI_DIST_NOISE=0.15; CSI_ANG_NOISE=0.05
HUMAN_BUFFER_LEN=40; HUMAN_ENERGY_THRESH=80.0

class ReconSlamNode(Node):
    def __init__(self):
        super().__init__("recon_slam_node")
        self.imu_sub=self.create_subscription(Imu,"/imu/data_raw",self.imu_callback,10)
        self.csi_sub=self.create_subscription(CSIData,"/wifi/csi",self.csi_callback,10)
        self.pose_pub=self.create_publisher(PoseStamped,"/robot_pose",10)
        self.pc_pub=self.create_publisher(PointCloud2,"/map_points",10)
        self.marker_pub=self.create_publisher(Marker,"/human_detection",10)
        self.yaw=0.0; self.last_imu_time=self.get_clock().now()
        self._calib_buf=[]; self._gyro_bias=0.0; self._calibrated=False
        self._window=np.hanning(64); self._bg_csi=np.zeros(64,dtype=complex)
        self._valid_idx=np.concatenate([np.arange(1,27),np.arange(38,64)])
        self._vital_buf=[]

    def imu_callback(self,msg):
        now=self.get_clock().now()
        dt=(now-self.last_imu_time).nanoseconds*1e-9
        self.last_imu_time=now
        if not(0.0<dt<0.5): return
        gz=float(msg.angular_velocity.z)
        if not self._calibrated:
            self._calib_buf.append(gz)
            if len(self._calib_buf)>=CALIB_SAMPLES:
                self._gyro_bias=float(np.mean(self._calib_buf)); self._calibrated=True
            self._publish_pose(now); return
        gz_c=gz-self._gyro_bias
        if abs(gz_c)>YAW_DEADBAND: self.yaw+=gz_c*dt
        self._publish_pose(now)

    def _publish_pose(self,stamp):
        pose=PoseStamped(); pose.header.stamp=stamp.to_msg()
        pose.header.frame_id="odom"
        pose.pose.position.x=0.0; pose.pose.position.y=0.0; pose.pose.position.z=0.0
        pose.pose.orientation.z=math.sin(self.yaw*0.5)
        pose.pose.orientation.w=math.cos(self.yaw*0.5)
        self.pose_pub.publish(pose)

    def csi_callback(self,msg):
        try:
            vals=np.asarray(msg.raw_csi,dtype=np.float64)
            if vals.size<128: return
            csi=vals[1::2][:64]+1j*vals[0::2][:64]
            csi[0]=0.0+0j; csi[27:38]=0.0+0j
            try:
                phase=np.angle(csi); unwrapped=np.unwrap(phase[self._valid_idx])
                coeffs=np.polyfit(self._valid_idx.astype(float),unwrapped,1)
                clean_ph=phase-np.polyval(coeffs,np.arange(64,dtype=float))
            except Exception: clean_ph=np.angle(csi)
            clean_csi=np.abs(csi)*self._window*np.exp(1j*clean_ph)
            self._bg_csi=CSI_BG_ALPHA*self._bg_csi+(1.0-CSI_BG_ALPHA)*clean_csi
            dynamic_csi=clean_csi-self._bg_csi
            padded=np.pad(np.fft.fftshift(dynamic_csi),(96,96),mode="constant")
            cir=np.abs(np.fft.ifft(np.fft.ifftshift(padded)))
            cir_max=np.max(cir)
            if cir_max<1e-9: return
            peaks,_=find_peaks(cir,height=cir_max*CSI_PEAK_THRESH,distance=CSI_PEAK_DIST)
            points=[]
            for peak_idx in peaks:
                distance_m=peak_idx*(3e8/(2.0*20e6*256))
                if not(CSI_MIN_DIST_M<distance_m<CSI_MAX_DIST_M): continue
                for angle in np.linspace(self.yaw-CSI_FAN_HALF_ANGLE,self.yaw+CSI_FAN_HALF_ANGLE,CSI_FAN_RAYS):
                    r=distance_m+np.random.normal(0.0,CSI_DIST_NOISE)
                    th=angle+np.random.normal(0.0,CSI_ANG_NOISE)
                    px=r*math.cos(th); py=r*math.sin(th)
                    for pz in [0.0,0.4,0.8,1.2,1.6,2.0]:
                        points.append([px,py,pz])
            if points:
                hdr=Header(); hdr.stamp=self.get_clock().now().to_msg(); hdr.frame_id="odom"
                self.pc_pub.publish(point_cloud2.create_cloud_xyz32(hdr,points))
            self._vital_buf.append(float(np.abs(csi[26])))
            if len(self._vital_buf)>HUMAN_BUFFER_LEN: self._vital_buf.pop(0)
            if len(self._vital_buf)==HUMAN_BUFFER_LEN: self._publish_human_marker(float(np.var(self._vital_buf)))
        except Exception: pass

    def _publish_human_marker(self,energy):
        m=Marker(); m.header.frame_id="odom"; m.type=Marker.SPHERE; m.action=Marker.ADD
        m.pose.position.z=2.8; m.pose.orientation.w=1.0
        m.scale.x=m.scale.y=m.scale.z=1.2; m.color.a=0.9
        if energy>HUMAN_ENERGY_THRESH:
            m.color.r=1.0; m.color.g=0.0; m.color.b=0.0
        else:
            m.color.r=0.0; m.color.g=0.9; m.color.b=1.0
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args); node=ReconSlamNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()
