import rclpy, math, time
from rclpy.node import Node
from nav_msgs.msg import Odometry

class S(Node):
    def __init__(self):
        super().__init__('samp')
        self.create_subscription(Odometry,'/state_estimation',self.cb,50)
        self.data=[]; self.t0=None; self.last=-9
    def cb(self,m):
        t=self.get_clock().now().nanoseconds/1e9
        if self.t0 is None: self.t0=t
        rt=t-self.t0
        p=m.pose.pose.position; q=m.pose.pose.orientation
        yaw=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.data.append((rt,p.x,p.y,p.z,yaw))
        if rt-self.last>=2.0:
            self.last=rt
            print(f"t={rt:5.1f}s  x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f}  yaw={yaw:+6.1f}",flush=True)

rclpy.init(); n=S(); end=time.time()+100
while time.time()<end and rclpy.ok(): rclpy.spin_once(n,timeout_sec=0.2)
d=n.data
if d:
    xs=[r[1] for r in d]; ys=[r[2] for r in d]; zs=[r[3] for r in d]; yw=[r[4] for r in d]
    s=d[0]; e=d[-1]
    print("=== SUMMARY ===",flush=True)
    print(f"samples={len(d)} dur={e[0]:.1f}s",flush=True)
    print(f"start x={s[1]:+.3f} y={s[2]:+.3f} z={s[3]:+.3f} yaw={s[4]:+.1f}",flush=True)
    print(f"end   x={e[1]:+.3f} y={e[2]:+.3f} z={e[3]:+.3f} yaw={e[4]:+.1f}",flush=True)
    print(f"range x[{min(xs):+.2f},{max(xs):+.2f}] y[{min(ys):+.2f},{max(ys):+.2f}] z[{min(zs):+.2f},{max(zs):+.2f}] yaw[{min(yw):+.0f},{max(yw):+.0f}]",flush=True)
    rete=math.sqrt((e[1]-s[1])**2+(e[2]-s[2])**2+(e[3]-s[3])**2)
    print(f"RETURN-TO-START pos err={rete:.3f} m, yaw err={e[4]-s[4]:+.1f} deg",flush=True)
n.destroy_node(); rclpy.shutdown()
