#!/usr/bin/env python3
"""RViz rectangle editor and position-controlled Nav2 layer switch."""
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion
from visualization_msgs.msg import (Marker, MarkerArray, InteractiveMarker,
                                    InteractiveMarkerControl as Control,
                                    InteractiveMarkerFeedback as Feedback)
from interactive_markers import InteractiveMarkerServer, MenuHandler
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from .geometry import Zone, load_zones, save_zones


def quaternion(yaw):
    return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def yaw_of(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class LayerTarget:
    """One outstanding RPC per node; poll actual value to recover Nav2 restarts."""
    def __init__(self, node, target, parameter):
        self.node, self.target, self.parameter = node, target, parameter
        self.get = node.create_client(GetParameters, target.rstrip('/') + '/get_parameters')
        self.set = node.create_client(SetParameters, target.rstrip('/') + '/set_parameters')
        self.pending = None
        self.actual = None
        self.next_poll = 0.0
        self.last_warning = 0.0
        self.error = ''

    def warn(self, message):
        self.error = message
        if time.monotonic() - self.last_warning > 5:
            self.node.get_logger().warning(f'{self.target}: {message}')
            self.last_warning = time.monotonic()

    def step(self, desired):
        now = time.monotonic()
        if self.pending:
            kind, sent_value, future, started = self.pending
            if not future.done():
                if now - started > 3:
                    self.actual = None
                    self.warn('parameter service response pending; state is unconfirmed')
                # A timed-out SetParameters may still execute. Never overlap writes.
                return
            self.pending = None
            try:
                response = future.result()
                if kind == 'get':
                    if len(response.values) != 1 or response.values[0].type != ParameterType.PARAMETER_BOOL:
                        raise RuntimeError(f'{self.parameter} is missing or not bool')
                    self.actual = response.values[0].bool_value
                else:
                    if len(response.results) != 1 or not response.results[0].successful:
                        reason = response.results[0].reason if response.results else 'empty response'
                        raise RuntimeError(reason)
                    self.actual = sent_value
                    self.node.get_logger().info(f'{self.target}: {self.parameter} = {sent_value}')
                self.error = ''
                self.next_poll = now + 2
            except Exception as exc:
                self.actual = None
                self.warn(str(exc))
                self.next_poll = now + 1
                return
        if not self.get.service_is_ready() or not self.set.service_is_ready():
            self.actual = None
            self.warn('waiting for Nav2 parameter services')
            return
        if self.actual is not None and self.actual != desired:
            req = SetParameters.Request()
            req.parameters = [Parameter(self.parameter, value=desired).to_parameter_msg()]
            self.pending = ('set', desired, self.set.call_async(req), now)
        elif now >= self.next_poll:
            req = GetParameters.Request(names=[self.parameter])
            self.pending = ('get', None, self.get.call_async(req), now)

    def confirmed(self, desired):
        return self.actual == desired and self.pending is None and not self.error


class ObstacleZoneNode(Node):
    def __init__(self, *, editor=False):
        super().__init__('obstacle_zone_node')
        defaults = {
            'zones_file': '~/obstacle_zones.yaml', 'frame_id': 'map',
            'robot_frame': 'base_link',
            'target_nodes': ['/local_costmap/local_costmap'],
            'layer_parameter': 'obstacle_layer.enabled', 'check_rate': 10.0,
            'tf_max_age': 1.0, 'exit_margin': 0.0,
            'default_width': 2.0, 'default_height': 2.0,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value
        self.path, self.frame, self.robot_frame = p('zones_file'), p('frame_id'), p('robot_frame')
        self.edit, self.switch = editor, not editor
        self.max_age, self.margin = p('tf_max_age'), p('exit_margin')
        self.width, self.height = p('default_width'), p('default_height')
        numbers = [p('check_rate'), self.max_age, self.margin, self.width, self.height]
        if not all(math.isfinite(v) for v in numbers):
            raise ValueError('numeric parameters must be finite')
        if p('check_rate') <= 0 or self.max_age <= 0 or self.margin < 0 or min(self.width, self.height) < 0.1:
            raise ValueError('invalid rate, TF age, margin or rectangle size')
        targets = p('target_nodes')
        if self.switch and not targets:
            raise ValueError('target_nodes must not be empty')
        self.zones = load_zones(self.path, self.frame)
        self.inside = set()
        self.desired = True
        self.stopping = False
        self.last_tf_warning = 0.0
        self.tf = Buffer() if self.switch else None
        self.listener = TransformListener(self.tf, self) if self.switch else None
        self.targets = [LayerTarget(self, n, p('layer_parameter')) for n in dict.fromkeys(targets)] if self.switch else []
        self.server = InteractiveMarkerServer(self, 'obstacle_zones') if self.edit else None
        self.menu = MenuHandler()
        self.menu.insert('Duplicate', callback=self.duplicate)
        self.menu.insert('Delete', callback=self.delete)
        self.menu.insert('Save YAML', callback=self.menu_save)
        self.markers = self.create_publisher(MarkerArray, '/obstacle_zones/rectangles', 10)
        self.status = self.create_publisher(String, '/obstacle_zones/status', 10)
        self.save_srv = self.create_service(Trigger, '~/save', self.save_callback)
        self.reload_srv = self.create_service(Trigger, '~/reload', self.reload_callback)
        if self.edit:
            self.click_sub = self.create_subscription(PointStamped, '/clicked_point', self.clicked, 10)
        self.rebuild()
        self.timer = self.create_timer(1 / p('check_rate'), self.tick) if self.switch else None
        self.display_timer = self.create_timer(0.5, self.publish)
        self.get_logger().info(f'{len(self.zones)} zones loaded; edit={self.edit}, switch={self.switch}')

    def new_name(self):
        names = {z.name for z in self.zones}
        i = 1
        while f'zone_{i}' in names:
            i += 1
        return f'zone_{i}'

    def clicked(self, msg):
        if msg.header.frame_id != self.frame:
            self.get_logger().warning(f'Publish Point must use Fixed Frame={self.frame}')
            return
        if not all(math.isfinite(v) for v in (msg.point.x, msg.point.y)):
            return
        self.zones.append(Zone(self.new_name(), msg.point.x, msg.point.y, 0.0, self.width, self.height))
        self.rebuild()

    def get_zone(self, name):
        return next((z for z in self.zones if z.name == name), None)

    def duplicate(self, fb):
        z = self.get_zone(fb.marker_name)
        if z:
            self.zones.append(Zone(self.new_name(), z.x + 0.5, z.y + 0.5, z.yaw, z.width, z.height))
            self.rebuild()

    def delete(self, fb):
        self.zones = [z for z in self.zones if z.name != fb.marker_name]
        self.rebuild()

    def menu_save(self, fb):
        try:
            self.save()
        except Exception as exc:
            self.get_logger().error(str(exc))

    def save(self):
        save_zones(self.path, self.frame, self.zones)
        self.get_logger().info(f'Saved {self.path}')

    def save_callback(self, req, res):
        try:
            self.save()
            res.success, res.message = True, f'Saved {self.path}'
        except Exception as exc:
            res.success, res.message = False, str(exc)
        return res

    def reload_callback(self, req, res):
        try:
            zones = load_zones(self.path, self.frame)
            self.zones = zones
            self.rebuild()
            res.success, res.message = True, f'Reloaded {self.path}'
        except Exception as exc:
            res.success, res.message = False, str(exc)
        return res

    def body(self, z):
        m = Marker()
        m.type = Marker.CUBE
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = z.width, z.height, 0.025
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.65, 1.0, 0.25
        return m

    def base_marker(self, name, description, x, y, yaw=0.0):
        m = InteractiveMarker()
        m.header.frame_id = self.frame
        m.name, m.description, m.scale = name, description, 0.8
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.08
        m.pose.orientation = quaternion(yaw)
        return m

    def plane_control(self, name):
        c = Control()
        c.name = name
        # Interactive control X axis points along world Z; MOVE_PLANE is XY.
        c.orientation = Quaternion(y=math.sqrt(0.5), w=math.sqrt(0.5))
        c.orientation_mode = Control.FIXED
        c.interaction_mode = Control.MOVE_PLANE
        c.always_visible = True
        return c

    def insert_zone(self, z, with_handle=True):
        m = self.base_marker(z.name, z.name, z.x, z.y, z.yaw)
        move = self.plane_control('move_xy')
        move.markers.append(self.body(z))
        m.controls.append(move)
        rotate = self.plane_control('rotate_yaw')
        rotate.interaction_mode = Control.ROTATE_AXIS
        m.controls.append(rotate)
        self.server.insert(m, feedback_callback=self.feedback)
        self.menu.apply(self.server, m.name)
        if with_handle:
            self.insert_handle(z)

    def insert_handle(self, z):
        x, y = z.world(z.width / 2, z.height / 2)
        m = self.base_marker(z.name + '__size', 'size', x, y)
        m.scale = 0.3
        c = self.plane_control('resize_xy')
        handle = Marker()
        handle.type = Marker.CUBE
        handle.pose.orientation.w = 1.0
        handle.scale.x = handle.scale.y = handle.scale.z = 0.18
        handle.color.r, handle.color.g, handle.color.a = 1.0, 0.7, 1.0
        c.markers.append(handle)
        m.controls.append(c)
        self.server.insert(m, feedback_callback=self.feedback)

    def rebuild(self):
        self.inside.clear()
        if self.edit:
            self.server.clear()
            for z in self.zones:
                self.insert_zone(z)
        if self.server is not None:
            self.server.applyChanges()
        self.publish()

    def feedback(self, fb):
        if fb.header.frame_id != self.frame or fb.event_type != Feedback.POSE_UPDATE:
            return
        resize = fb.marker_name.endswith('__size')
        name = fb.marker_name[:-6] if resize else fb.marker_name
        z = self.get_zone(name)
        if z is None:
            return
        p = fb.pose.position
        yaw = yaw_of(fb.pose.orientation)
        if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
            return
        if resize:
            u, v = z.local(p.x, p.y)
            z.width, z.height = max(0.1, 2*u), max(0.1, 2*v)
            # Update the body without reinserting the handle being dragged.
            self.insert_zone(z, with_handle=False)
            hx, hy = z.world(z.width / 2, z.height / 2)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = hx, hy, 0.08
            pose.orientation.w = 1.0
            self.server.setPose(fb.marker_name, pose)
        else:
            z.x, z.y, z.yaw = p.x, p.y, yaw
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = z.x, z.y, 0.08
            pose.orientation = quaternion(z.yaw)
            self.server.setPose(z.name, pose)
            self.insert_handle(z)
        if self.server is not None:
            self.server.applyChanges()

    def tick(self):
        tf_ok, xy = False, None
        if not self.stopping:
            try:
                t = self.tf.lookup_transform(self.frame, self.robot_frame, Time())
                stamp = Time.from_msg(t.header.stamp)
                age = (self.get_clock().now() - stamp).nanoseconds / 1e9
                if stamp.nanoseconds != 0 and (age > self.max_age or age < -self.max_age):
                    raise RuntimeError(f'TF age {age:.2f}s exceeds tf_max_age')
                x, y = t.transform.translation.x, t.transform.translation.y
                if not all(math.isfinite(v) for v in (x, y)):
                    raise RuntimeError('nonfinite TF position')
                tf_ok, xy = True, [x, y]
                self.inside = {z.name for z in self.zones if z.contains(
                    x, y, self.margin if z.name in self.inside else 0.0)}
            except (TransformException, RuntimeError) as exc:
                self.inside.clear()
                if time.monotonic() - self.last_tf_warning > 5:
                    self.get_logger().warning(f'TF unavailable; requesting enabled=True: {exc}')
                    self.last_tf_warning = time.monotonic()
        self.desired = not bool(self.inside) if tf_ok else True
        if self.switch:
            for target in self.targets:
                target.step(self.desired)
        self.status.publish(String(data=json.dumps({
            'tf_ok': tf_ok, 'robot_xy': xy, 'inside_zones': sorted(self.inside),
            'switch_active': self.switch, 'desired_enabled': self.desired,
            'targets': {t.target: {'enabled': t.actual, 'pending': bool(t.pending),
                                  'error': t.error} for t in self.targets},
        })))

    def publish(self):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for i, z in enumerate(self.zones):
            m = Marker()
            m.header.frame_id = self.frame
            m.ns, m.id, m.type = 'zone_outline', i, Marker.LINE_STRIP
            m.pose.orientation.w = 1.0
            m.scale.x = 0.04
            m.color.r, m.color.g, m.color.a = (1.0 if z.name in self.inside else 0.1), 0.8, 1.0
            for u, v in [(-1,-1),(1,-1),(1,1),(-1,1),(-1,-1)]:
                x, y = z.world(u*z.width/2, v*z.height/2)
                m.points.append(Point(x=x, y=y, z=0.09))
            array.markers.append(m)
            label = Marker()
            label.header.frame_id = self.frame
            label.ns, label.id, label.type = 'zone_label', i, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = z.x, z.y, 0.3
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = z.name
            array.markers.append(label)
        self.markers.publish(array)

    def restore(self):
        if not self.switch:
            return
        self.stopping = True
        self.inside.clear()
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and rclpy.ok():
            self.tick()
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(t.confirmed(True) for t in self.targets):
                self.get_logger().info('All target layers restored to enabled=True')
                return
        self.get_logger().error('Restoration unconfirmed. Set obstacle_layer.enabled true manually.')


def _run(args=None, *, editor=False):
    # Keep ROS services alive during Ctrl+C restoration.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = [False]
    old = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        old[sig] = signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    node = None
    try:
        node = ObstacleZoneNode(editor=editor)
        while rclpy.ok() and not stop[0]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.restore()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old.items():
            signal.signal(sig, handler)


def edit_main(args=None):
    _run(args, editor=True)


def run_main(args=None):
    _run(args, editor=False)


if __name__ == '__main__':
    run_main()
