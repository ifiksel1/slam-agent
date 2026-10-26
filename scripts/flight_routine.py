#!/usr/bin/env python3
"""Automated QuadRover flight test routine (Guided flight from the companion).

The pilot arms in MANUAL. This script then switches to GUIDED, takes off, and flies the parts
switched on in the config, in a fixed order:

  takeoff -> pitch moves -> roll moves -> yaw turns -> climb -> diagonals -> orbits
          -> attitude doublets -> land (or hold for the pilot)

It never arms. Any mode other than GUIDED, or a disarm, ends the run at once (the pilot takes over
with ch7 mid = QRLoiter). SIGINT = hold: hover where it is and end. SIGUSR1 = land now.

Every move streams position + velocity targets along a straight line at the routine speed and then
holds until settled, so the speed is the routine's, not Q_WP_SPD. Doublets are attitude targets
with thrust 0.5, which QuadRover's Guided reads as "hold height" (Q_GUID_OPTIONS 0).

Output: one JSON object per line on stdout (events), for revolute-gui's Testing page.

Two backends behind one interface:
  mavros  - on the robot, inside the SLAM container (ROS1, /mavros/...)
  mavlink - pymavlink straight to the FC or SITL (used by the SITL autotest FlightRoutineScript)

usage:
  flight_routine.py --dry-run [--config JSON]
  flight_routine.py --backend mavros --config JSON          (robot; revolute-gui pipes it in)
  flight_routine.py --backend mavlink --url tcp:127.0.0.1:5763 --config JSON   (SITL)
"""
import argparse
import json
import math
import signal
import sys
import time

DEFAULTS = {
    'height_m': 1.5,          # takeoff height above the liftoff point
    'distance_m': 1.0,        # pitch / roll moves and diagonals
    'speed_ms': 0.5,          # every move and the orbits
    'climb_m': 0.5,           # up and down from the takeoff height
    'yaw_deg': 90.0,          # yaw turns each way
    'yaw_rate_degs': 30.0,
    'orbit_radius_m': 1.5,
    'orbit_turns': 1,
    'orbit_dir': 'both',      # cw, ccw, both
    'doublet_deg': 5.0,       # roll and pitch doublets
    'doublet_yaw_deg': 15.0,  # yaw doublet (an attitude step, not a rate)
    'doublet_s': 1.0,         # each half of a doublet
    'max_height_m': 2.0,      # refuse plans above this (the room's ceiling minus a margin)
    'min_height_m': 0.8,      # refuse plans below this (climb's down step)
    'battery_min_v': 22.0,    # refuse to start below this (0 = no check)
    'land': True,             # land at the end; False = hold for the pilot
    'slam_check': True,       # refuse to start without fresh SLAM odometry (off only for SITL tests)
    'parts': {
        'pitch': True, 'roll': True, 'yaw': True, 'climb': True,
        'diagonals': True, 'orbits': True, 'doublets': False,
    },
}
PART_ORDER = ('pitch', 'roll', 'yaw', 'climb', 'diagonals', 'orbits', 'doublets')
GUIDED = 15
POS_TOL_M = 0.2               # a move is reached within this
SETTLE_S = 1.0                # and stays there this long
YAW_TOL_DEG = 5.0
STREAM_DT = 0.05              # 20 Hz targets
ATT_DT = 0.02                 # 50 Hz attitude targets


def emit(ev, **kw):
    kw['ev'] = ev
    kw['t'] = round(time.time(), 2)
    print(json.dumps(kw), flush=True)


def merge_config(cfg):
    out = json.loads(json.dumps(DEFAULTS))
    for k, v in (cfg or {}).items():
        if k == 'parts' and isinstance(v, dict):
            out['parts'].update({p: bool(on) for p, on in v.items() if p in out['parts']})
        elif k in out:
            out[k] = v
    for k in ('height_m', 'distance_m', 'speed_ms', 'climb_m', 'yaw_deg', 'yaw_rate_degs',
              'orbit_radius_m', 'doublet_deg', 'doublet_yaw_deg', 'doublet_s', 'max_height_m',
              'min_height_m', 'battery_min_v'):
        out[k] = float(out[k])
    out['orbit_turns'] = int(out['orbit_turns'])
    out['land'] = bool(out['land'])
    out['slam_check'] = bool(out['slam_check'])
    return out


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------------------------
# The plan: steps in body/world terms, before the start pose is known
# ---------------------------------------------------------------------------------------------

def build_plan(cfg):
    """(steps, envelope, problems). Steps are dicts; positions are relative to the hover centre:
    'fwd'/'right' (body, from the heading at takeoff) or 'north'/'east' (world), 'up' from the
    takeoff height."""
    d, c, h, r = cfg['distance_m'], cfg['climb_m'], cfg['height_m'], cfg['orbit_radius_m']
    p = cfg['parts']
    steps = [{'part': 'takeoff', 'name': 'take off to %.1f m' % h, 'kind': 'takeoff'}]

    def move(part, name, **rel):
        steps.append(dict({'part': part, 'name': name, 'kind': 'move'}, **rel))

    if p['pitch']:
        move('pitch', 'forward %.1f m' % d, fwd=d)
        move('pitch', 'back to centre')
        move('pitch', 'back %.1f m' % d, fwd=-d)
        move('pitch', 'back to centre')
    if p['roll']:
        move('roll', 'right %.1f m' % d, right=d)
        move('roll', 'back to centre')
        move('roll', 'left %.1f m' % d, right=-d)
        move('roll', 'back to centre')
    if p['yaw']:
        y = cfg['yaw_deg']
        for name, rel in (('yaw +%.0f deg' % y, y), ('yaw back', 0.0), ('yaw -%.0f deg' % y, -y),
                          ('yaw back', 0.0)):
            steps.append({'part': 'yaw', 'name': name, 'kind': 'yaw', 'yaw_deg': rel})
    if p['climb']:
        move('climb', 'up %.1f m' % c, up=c)
        move('climb', 'back to %.1f m' % h)
        move('climb', 'down %.1f m' % c, up=-c)
        move('climb', 'back to %.1f m' % h)
    if p['diagonals']:
        k = d / math.sqrt(2)
        for name, n, e in (('north-east', k, k), ('north-west', k, -k), ('south-east', -k, k),
                           ('south-west', -k, -k)):
            move('diagonals', '%s %.1f m' % (name, d), north=n, east=e)
            move('diagonals', 'back to centre')
    if p['orbits']:
        dirs = {'cw': ('cw',), 'ccw': ('ccw',), 'both': ('cw', 'ccw')}.get(cfg['orbit_dir'], ('cw', 'ccw'))
        move('orbits', 'out to the orbit edge (%.1f m ahead)' % r, fwd=r)
        steps.append({'part': 'orbits', 'name': 'face the centre', 'kind': 'yaw', 'yaw_deg': 180.0,
                      'hold_fwd': r})
        for dr in dirs:
            steps.append({'part': 'orbits', 'name': '%d turn%s %s, radius %.1f m'
                          % (cfg['orbit_turns'], '' if cfg['orbit_turns'] == 1 else 's', dr.upper(), r),
                          'kind': 'orbit', 'dir': dr})
        steps.append({'part': 'orbits', 'name': 'face forward', 'kind': 'yaw', 'yaw_deg': 0.0,
                      'hold_fwd': r})
        move('orbits', 'back to centre')
    if p['doublets']:
        for axis, a in (('roll', cfg['doublet_deg']), ('pitch', cfg['doublet_deg']),
                        ('yaw', cfg['doublet_yaw_deg'])):
            steps.append({'part': 'doublets', 'name': '%s doublet +-%.0f deg' % (axis, a),
                          'kind': 'doublet', 'axis': axis, 'deg': a})
    if cfg['land']:
        steps.append({'part': 'land', 'name': 'land', 'kind': 'land'})
    else:
        steps.append({'part': 'hold', 'name': 'hold for the pilot', 'kind': 'hold'})

    radius = 0.0
    for s in steps:
        radius = max(radius, abs(s.get('fwd', 0.0)), abs(s.get('right', 0.0)),
                     math.hypot(s.get('north', 0.0), s.get('east', 0.0)))
    top = h + (c if p['climb'] else 0.0)
    bottom = h - (c if p['climb'] else 0.0)
    env = {'max_height_m': round(top, 2), 'min_height_m': round(bottom, 2),
           'radius_m': round(radius, 2), 'clear_square_m': round(2 * radius + 1.0, 1)}
    problems = []
    if top > cfg['max_height_m'] + 1e-6:
        problems.append('plan reaches %.2f m, above the max height %.2f m' % (top, cfg['max_height_m']))
    if bottom < cfg['min_height_m'] - 1e-6:
        problems.append('plan goes down to %.2f m, below the min height %.2f m' % (bottom, cfg['min_height_m']))
    if cfg['speed_ms'] <= 0 or cfg['speed_ms'] > 2.0:
        problems.append('speed %.2f m/s outside 0-2 m/s' % cfg['speed_ms'])
    if cfg['orbit_radius_m'] < 0.5 and p['orbits']:
        problems.append('orbit radius under 0.5 m')
    if cfg['doublet_deg'] > 15 or cfg['doublet_yaw_deg'] > 45:
        problems.append('doublet angle too large (roll/pitch <= 15 deg, yaw <= 45 deg)')
    return steps, env, problems


# ---------------------------------------------------------------------------------------------
# Backends. Positions are ENU metres (east, north, up), yaw is ENU radians (from east, CCW),
# roll right and pitch nose-up positive.
# ---------------------------------------------------------------------------------------------

class MavrosBackend:
    """MAVROS inside the SLAM container."""

    def __init__(self):
        import rospy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import AttitudeTarget, PositionTarget, State, StatusText
        from mavros_msgs.srv import CommandLong
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import BatteryState
        self.rospy, self.PositionTarget, self.AttitudeTarget = rospy, PositionTarget, AttitudeTarget
        self.S = {'state': None, 'pose': None, 'volt': None, 'odom_t': None}
        self.texts = []
        rospy.init_node('flight_routine', anonymous=True, disable_signals=True)
        rospy.Subscriber('/mavros/state', State, lambda m: self.S.__setitem__('state', m))
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, lambda m: self.S.__setitem__('pose', m))
        rospy.Subscriber('/mavros/battery', BatteryState, lambda m: self.S.__setitem__('volt', m.voltage))
        rospy.Subscriber('/mavros/statustext/recv', StatusText, lambda m: self.texts.append(m.text))
        rospy.Subscriber('/Odometry', Odometry, lambda m: self.S.__setitem__('odom_t', time.time()))
        self.sp_pos = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=5)
        self.sp_att = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=5)
        rospy.wait_for_service('/mavros/cmd/command', timeout=10)
        self.cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

    def now(self):
        return time.time()

    def wait(self, dt):
        time.sleep(dt)

    def ready(self):
        return self.S['state'] is not None and self.S['pose'] is not None

    def armed_mode(self):
        st = self.S['state']
        return bool(st.armed), ('GUIDED' if st.mode == 'GUIDED' else st.mode)

    def pose(self):
        p = self.S['pose'].pose
        q = p.orientation
        roll, pitch, yaw = quat_to_euler(q.w, q.x, q.y, q.z)     # ENU/FLU
        return p.position.x, p.position.y, p.position.z, yaw, roll, -pitch

    def voltage(self):
        return self.S['volt']

    def slam_ok(self):
        t = self.S['odom_t']
        return t is not None and time.time() - t < 0.5

    def command(self, cmd, p1=0.0, p2=0.0, p7=0.0):
        res = self.cmd(broadcast=False, command=cmd, confirmation=0, param1=p1, param2=p2, param3=0,
                       param4=0, param5=0, param6=0, param7=p7)
        return bool(res.success)

    def pos_target(self, e, n, u, ve, vn, vu, yaw):
        m = self.PositionTarget()
        m.header.stamp = self.rospy.Time.now()
        m.coordinate_frame = self.PositionTarget.FRAME_LOCAL_NED      # values in ENU; MAVROS converts
        m.type_mask = 64 | 128 | 256 | 2048                            # ignore accel and yaw rate
        m.position.x, m.position.y, m.position.z = e, n, u
        m.velocity.x, m.velocity.y, m.velocity.z = ve, vn, vu
        m.yaw = yaw
        self.sp_pos.publish(m)

    def att_target(self, roll, pitch, yaw):
        m = self.AttitudeTarget()
        m.header.stamp = self.rospy.Time.now()
        m.type_mask = 1 | 2 | 4                                         # attitude only, no body rates
        w, x, y, z = euler_to_quat(roll, -pitch, yaw)                   # ENU/FLU: pitch nose-up is negative
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = w, x, y, z
        m.thrust = 0.5                                                  # climb rate 0: hold height
        self.sp_att.publish(m)

    def new_texts(self):
        out, self.texts = self.texts, []
        return out


class MavlinkBackend:
    """pymavlink to the FC or SITL; its own sysid (never 255, the GCS failsafe's)."""

    def __init__(self, url):
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.m = mavutil.mavlink_connection(url, source_system=245, source_component=191,
                                            robust_parsing=True)
        self.st = {'hb': None, 'lp': None, 'att': None, 'boot_ms': None, 'volt': None}
        self.texts = []
        self.acks = {}
        t0 = time.time()
        while self.st['hb'] is None and time.time() - t0 < 30:
            self.pump(1.0)
        if self.st['hb'] is None:
            raise SystemExit('no heartbeat from the autopilot')
        self.m.target_system, self.m.target_component = 1, 1
        for msg_id in (32, 30, 1):                  # LOCAL_POSITION_NED, ATTITUDE, SYS_STATUS at 20 Hz
            self.m.mav.command_long_send(1, 1, 511, 0, msg_id, 50000, 0, 0, 0, 0, 0)
            self.pump(0.1)

    def pump(self, timeout=0.0):
        """read everything waiting, and keep reading for timeout seconds"""
        t_end = time.time() + timeout
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                if time.time() >= t_end:
                    return
                time.sleep(0.002)
                continue
            self.handle(msg)

    def handle(self, msg):
        t = msg.get_type()
        if msg.get_srcSystem() != 1:
            return
        if t == 'HEARTBEAT' and msg.get_srcComponent() == 1 and msg.autopilot != 8:
            self.st['hb'] = msg
        elif t == 'LOCAL_POSITION_NED':
            self.st['lp'] = msg
            self.st['boot_ms'] = msg.time_boot_ms
        elif t == 'ATTITUDE':
            self.st['att'] = msg
            self.st['boot_ms'] = msg.time_boot_ms
        elif t == 'SYS_STATUS':
            self.st['volt'] = msg.voltage_battery / 1000.0
        elif t == 'STATUSTEXT':
            self.texts.append(msg.text)
        elif t == 'COMMAND_ACK':
            self.acks[msg.command] = msg.result

    def now(self):
        self.pump(0)
        return (self.st['boot_ms'] or 0) / 1000.0

    def wait(self, dt):
        """dt of the vehicle's clock (SITL may run faster than real time)"""
        t0 = self.now()
        while self.now() - t0 < dt:
            self.pump(0.01)

    def ready(self):
        self.pump(0)
        return all(self.st[k] is not None for k in ('hb', 'lp', 'att'))

    def armed_mode(self):
        hb = self.st['hb']
        armed = bool(hb.base_mode & 128)
        return armed, ('GUIDED' if hb.custom_mode == GUIDED else 'MODE%d' % hb.custom_mode)

    def pose(self):
        lp, att = self.st['lp'], self.st['att']
        return lp.y, lp.x, -lp.z, wrap_pi(math.pi / 2 - att.yaw), att.roll, att.pitch

    def voltage(self):
        return self.st['volt']

    def slam_ok(self):
        return True

    def command(self, cmd, p1=0.0, p2=0.0, p7=0.0):
        self.acks.pop(cmd, None)
        self.m.mav.command_long_send(1, 1, cmd, 0, p1, p2, 0, 0, 0, 0, p7)
        t0 = time.time()
        while time.time() - t0 < 3:
            self.pump(0.05)
            if cmd in self.acks:
                return self.acks[cmd] == 0
        return False

    def pos_target(self, e, n, u, ve, vn, vu, yaw):
        self.m.mav.set_position_target_local_ned_send(
            0, 1, 1, 1, 64 | 128 | 256 | 2048, n, e, -u, vn, ve, -vu, 0, 0, 0,
            wrap_pi(math.pi / 2 - yaw), 0)

    def att_target(self, roll, pitch, yaw):
        q = euler_to_quat(roll, pitch, wrap_pi(math.pi / 2 - yaw))     # NED/FRD
        self.m.mav.set_attitude_target_send(0, 1, 1, 1 | 2 | 4, q, 0, 0, 0, 0.5)

    def new_texts(self):
        self.pump(0)
        out, self.texts = self.texts, []
        return out


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def quat_to_euler(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# ---------------------------------------------------------------------------------------------
# The routine
# ---------------------------------------------------------------------------------------------

class Stop(Exception):
    """ends the run: reason, and whether it is a fault"""

    def __init__(self, reason, fault=False):
        super().__init__(reason)
        self.reason, self.fault = reason, fault


class Routine:
    def __init__(self, be, cfg):
        self.be, self.cfg = be, cfg
        self.flag = None                  # 'hold' or 'land', set by a signal
        self.in_guided = False
        self.target = None                # the last commanded (e, n, u, yaw)

    # --- checks run on every tick ----------------------------------------------------------
    def tick(self):
        for text in self.be.new_texts():
            emit('fc', text=text)
        armed, mode = self.be.armed_mode()
        if not armed:
            raise Stop('disarmed')
        if self.in_guided and mode != 'GUIDED':
            raise Stop('pilot took over (%s)' % mode)
        if self.flag == 'hold':
            self.hold_here()
            raise Stop('hold requested: hovering in place, pilot take over')
        if self.flag == 'land':
            self.flag = None
            self.land()
            raise Stop('land requested: landed')

    def hold_here(self):
        e, n, u, yaw, _, _ = self.be.pose()
        for _ in range(5):
            self.be.pos_target(e, n, u, 0, 0, 0, yaw)
            self.be.wait(STREAM_DT)
        self.target = (e, n, u, yaw)

    # --- building blocks -------------------------------------------------------------------
    def settle(self, e, n, u, yaw, timeout_s, what):
        """hold the target until within POS_TOL_M and YAW_TOL_DEG for SETTLE_S"""
        t0 = self.be.now()
        good_since = None
        while True:
            self.tick()
            self.be.pos_target(e, n, u, 0, 0, 0, yaw)
            pe, pn, pu, pyaw, _, _ = self.be.pose()
            err = math.sqrt((pe - e) ** 2 + (pn - n) ** 2 + (pu - u) ** 2)
            yerr = abs(math.degrees(wrap_pi(pyaw - yaw)))
            now = self.be.now()
            if err <= POS_TOL_M and yerr <= YAW_TOL_DEG:
                good_since = good_since or now
                if now - good_since >= SETTLE_S:
                    self.target = (e, n, u, yaw)
                    return round(err, 2), round(yerr, 1)
            else:
                good_since = None
            if now - t0 > timeout_s:
                self.hold_here()
                raise Stop('%s not reached in %.0f s (%.2f m, %.0f deg off): holding' % (what, timeout_s, err, yerr),
                           fault=True)
            self.be.wait(STREAM_DT)

    def fly_line(self, e1, n1, u1, yaw1, what):
        e0, n0, u0, yaw0 = self.target
        dist = math.sqrt((e1 - e0) ** 2 + (n1 - n0) ** 2 + (u1 - u0) ** 2)
        v = self.cfg['speed_ms']
        dyaw = wrap_pi(yaw1 - yaw0)
        t_move = max(dist / v, abs(math.degrees(dyaw)) / self.cfg['yaw_rate_degs'], STREAM_DT)
        t0 = self.be.now()
        while True:
            self.tick()
            s = min(1.0, (self.be.now() - t0) / t_move)
            if dist > 1e-6:
                k = v / dist if s < 1.0 else 0.0
                ve, vn, vu = (e1 - e0) * k, (n1 - n0) * k, (u1 - u0) * k
            else:
                ve = vn = vu = 0.0
            self.be.pos_target(e0 + (e1 - e0) * s, n0 + (n1 - n0) * s, u0 + (u1 - u0) * s, ve, vn, vu,
                               wrap_pi(yaw0 + dyaw * s))
            if s >= 1.0:
                break
            self.be.wait(STREAM_DT)
        return self.settle(e1, n1, u1, yaw1, t_move + 15, what)

    def land(self):
        if not self.be.command(21):                                      # MAV_CMD_NAV_LAND
            raise Stop('land command refused', fault=True)
        emit('step', part='land', name='landing')
        t0 = self.be.now()
        while self.be.now() - t0 < 90:
            for text in self.be.new_texts():
                emit('fc', text=text)
                if 'landing complete' in text.lower() or 'landed' in text.lower():
                    return
            armed, mode = self.be.armed_mode()
            if not armed:
                return
            if mode != 'GUIDED':
                raise Stop('pilot took over (%s)' % mode)
            self.be.wait(0.1)
        raise Stop('landing not complete in 90 s', fault=True)

    # --- the run -------------------------------------------------------------------------------
    def preflight(self, env):
        t0 = time.time()
        while not self.be.ready():
            if time.time() - t0 > 15:
                raise Stop('no state or position from the FC', fault=True)
            self.be.wait(0.1)
        armed, mode = self.be.armed_mode()
        if not armed:
            raise Stop('not armed: the pilot arms in MANUAL first', fault=True)
        v = self.be.voltage()
        if self.cfg['battery_min_v'] > 0 and v is not None and 0 < v < self.cfg['battery_min_v']:
            raise Stop('battery %.1f V below %.1f V' % (v, self.cfg['battery_min_v']), fault=True)
        if self.cfg['slam_check'] and not self.be.slam_ok():
            raise Stop('SLAM odometry is stale', fault=True)
        emit('preflight', armed=armed, mode=mode, battery_v=v, envelope=env)

    def run(self, steps):
        n = len(steps)
        geo = {}
        for i, st in enumerate(steps):
            emit('step', i=i, n=n, part=st['part'], name=st['name'])
            res = self.do_step(st, geo)
            emit('done', i=i, n=n, part=st['part'], name=st['name'], **(res or {}))

    def do_step(self, st, geo):
        k = st['kind']
        if k == 'takeoff':
            return self.takeoff(geo)
        c_e, c_n, c_u, yaw0 = geo['centre']
        fwd = (math.cos(yaw0), math.sin(yaw0))
        right = (math.sin(yaw0), -math.cos(yaw0))

        def point(s):
            f, r = s.get('fwd', 0.0), s.get('right', 0.0)
            return (c_e + f * fwd[0] + r * right[0] + s.get('east', 0.0),
                    c_n + f * fwd[1] + r * right[1] + s.get('north', 0.0),
                    c_u + s.get('up', 0.0))
        if k == 'move':
            e, n, u = point(st)
            err, _ = self.fly_line(e, n, u, yaw0, st['name'])
            pe, pn, pu, _, _, _ = self.be.pose()
            return {'pos_err_m': err, 'height_m': round(pu - geo['ground_u'], 2)}
        if k == 'yaw':
            e, n, u = point({'fwd': st.get('hold_fwd', 0.0)})
            yaw = wrap_pi(yaw0 + math.radians(st['yaw_deg']))
            _, yerr = self.fly_line(e, n, u, yaw, st['name'])
            return {'yaw_err_deg': yerr}
        if k == 'orbit':
            return self.orbit(geo, st['dir'])
        if k == 'doublet':
            return self.doublet(geo, st['axis'], st['deg'])
        if k == 'land':
            self.land()
            return {}
        if k == 'hold':
            self.hold_here()
            raise Stop('routine done: holding for the pilot')
        raise Stop('unknown step %s' % k, fault=True)

    def takeoff(self, geo):
        e, n, u, yaw, _, _ = self.be.pose()
        geo['ground_u'] = u
        if not self.be.command(176, 1, GUIDED):                          # MAV_CMD_DO_SET_MODE
            raise Stop('GUIDED refused', fault=True)
        t0 = self.be.now()
        while self.be.armed_mode()[1] != 'GUIDED':
            if self.be.now() - t0 > 3:
                raise Stop('mode did not change to GUIDED', fault=True)
            self.be.wait(0.1)
        self.in_guided = True
        h = self.cfg['height_m']
        if not self.be.command(22, p7=h):                                 # MAV_CMD_NAV_TAKEOFF
            raise Stop('takeoff refused (moving, or no altitude estimate?)', fault=True)
        t0 = self.be.now()
        while True:
            self.tick()
            pe, pn, pu, pyaw, _, _ = self.be.pose()
            if pu - u >= h - 0.2:
                break
            if self.be.now() - t0 > 30:
                raise Stop('takeoff did not reach %.1f m in 30 s' % h, fault=True)
            self.be.wait(0.1)
        self.target = (e, n, u + h, yaw)
        err, _ = self.settle(e, n, u + h, yaw, 15, 'takeoff height')
        geo['centre'] = (e, n, u + h, yaw)
        pe, pn, pu, _, _, _ = self.be.pose()
        return {'pos_err_m': err, 'height_m': round(pu - u, 2)}

    def orbit(self, geo, direction):
        c_e, c_n, c_u, yaw0 = geo['centre']
        r = self.cfg['orbit_radius_m']
        oe, on = c_e, c_n                                                   # centred on the hover point
        theta0 = yaw0                                                       # the vehicle starts r ahead
        sign = -1.0 if direction == 'cw' else 1.0
        w = self.cfg['speed_ms'] / r
        sweep = 2 * math.pi * self.cfg['orbit_turns']
        t0 = self.be.now()
        rad_err = 0.0
        while True:
            self.tick()
            a = min(sweep, w * (self.be.now() - t0))
            th = theta0 + sign * a
            moving = a < sweep
            ve = -sign * math.sin(th) * self.cfg['speed_ms'] if moving else 0.0
            vn = sign * math.cos(th) * self.cfg['speed_ms'] if moving else 0.0
            self.be.pos_target(oe + r * math.cos(th), on + r * math.sin(th), c_u, ve, vn, 0.0,
                               wrap_pi(th + math.pi))
            pe, pn, _, _, _, _ = self.be.pose()
            rad_err = max(rad_err, abs(math.hypot(pe - oe, pn - on) - r))
            if not moving:
                break
            self.be.wait(STREAM_DT)
        self.target = (oe + r * math.cos(theta0), on + r * math.sin(theta0), c_u, wrap_pi(theta0 + math.pi))
        self.settle(*self.target, 15, 'orbit end')
        return {'max_radius_err_m': round(rad_err, 2)}

    def doublet(self, geo, axis, deg):
        c_e, c_n, c_u, yaw0 = geo['centre']
        self.settle(c_e, c_n, c_u, yaw0, 20, 'centre before the doublet')
        a = math.radians(deg)
        peak, low = 0.0, 0.0
        for sign in (1.0, -1.0):
            t0 = self.be.now()
            while self.be.now() - t0 < self.cfg['doublet_s']:
                self.tick()
                roll = sign * a if axis == 'roll' else 0.0
                pitch = sign * a if axis == 'pitch' else 0.0
                yaw = yaw0 + (sign * a if axis == 'yaw' else 0.0)
                self.be.att_target(roll, pitch, yaw)
                _, _, pu, pyaw, proll, ppitch = self.be.pose()
                got = {'roll': proll, 'pitch': ppitch, 'yaw': wrap_pi(pyaw - yaw0)}[axis]
                peak = max(peak, abs(math.degrees(got)))
                low = min(low, pu - c_u)
                self.be.wait(ATT_DT)
        # level, then back to position hold at the centre
        t0 = self.be.now()
        while self.be.now() - t0 < 0.5:
            self.tick()
            self.be.att_target(0.0, 0.0, yaw0)
            self.be.wait(ATT_DT)
        e, n, u, yaw, _, _ = self.be.pose()
        self.target = (e, n, u, yaw)
        self.fly_line(c_e, c_n, c_u, yaw0, 'back to centre after the doublet')
        return {'peak_deg': round(peak, 1), 'height_drop_m': round(-low, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default='{}', help='JSON settings (missing keys take the defaults)')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--backend', choices=('mavros', 'mavlink'), default='mavros')
    ap.add_argument('--url', default='tcp:127.0.0.1:5763')
    ap.add_argument('--tag', default='', help='ignored; lets the GUI find this process')
    # extra arguments are ignored: revolute-gui appends a bare tag (--rg-flight-routine) so that
    # Hold and Land now can find this process
    args, _ = ap.parse_known_args()

    cfg = merge_config(json.loads(args.config))
    steps, env, problems = build_plan(cfg)
    emit('plan', steps=[{'part': s['part'], 'name': s['name']} for s in steps], envelope=env,
         problems=problems, config=cfg)
    if args.dry_run:
        return 0 if not problems else 2
    if problems:
        emit('end', ok=False, reason='refused: ' + '; '.join(problems))
        return 2

    be = MavrosBackend() if args.backend == 'mavros' else MavlinkBackend(args.url)
    rt = Routine(be, cfg)
    signal.signal(signal.SIGINT, lambda *_: setattr(rt, 'flag', 'hold'))
    signal.signal(signal.SIGTERM, lambda *_: setattr(rt, 'flag', 'hold'))
    signal.signal(signal.SIGUSR1, lambda *_: setattr(rt, 'flag', 'land'))
    try:
        rt.preflight(env)
        rt.run(steps)
    except Stop as ex:
        emit('end', ok=not ex.fault, reason=ex.reason)
        return 1 if ex.fault else 0
    emit('end', ok=True, reason='routine complete')
    return 0


if __name__ == '__main__':
    sys.exit(main())
