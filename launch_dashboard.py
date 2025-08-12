#!/usr/bin/env python3

import os
import sys
import time
import json
import subprocess
import threading
import queue
import signal
import argparse
import socket
from datetime import datetime, timedelta
from collections import deque, defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import re

# Web interface imports
from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO, emit

# ROS2 imports for monitoring
try:
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import Log
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("Warning: ROS2 not available. Monitoring features will be limited.")

@dataclass
class NodeConfig:
    """Configuration for a ROS2 node"""
    name: str
    command: str
    topics_pub: List[str] = field(default_factory=list)
    topics_sub: List[str] = field(default_factory=list)
    expected_freq: Dict[str, float] = field(default_factory=dict)
    color: str = "#3498db"
    position: Tuple[int, int] = (0, 0)

@dataclass
class NodeStatus:
    """Runtime status of a node"""
    pid: Optional[int] = None
    running: bool = False
    start_time: Optional[datetime] = None
    status_led: str = "gray"  # gray, green, yellow, red
    last_error: Optional[str] = None
    last_warning: Optional[str] = None
    error_count: int = 0
    warning_count: int = 0
    last_error_time: Optional[datetime] = None
    last_warning_time: Optional[datetime] = None
    topic_frequencies: Dict[str, float] = field(default_factory=dict)
    logs: deque = field(default_factory=lambda: deque(maxlen=1000))
    
@dataclass
class TopicInfo:
    """Information about a topic"""
    name: str
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)
    frequency: float = 0.0
    msg_type: str = ""

class ProcessManager:
    """Manages ROS2 node processes"""
    
    def __init__(self):
        self.processes: Dict[str, subprocess.Popen] = {}
        self.log_queues: Dict[str, queue.Queue] = {}
        self.node_configs: Dict[str, NodeConfig] = {}
        self.node_status: Dict[str, NodeStatus] = {}
        self.topic_info: Dict[str, TopicInfo] = {}  # Track topic information
        self.topic_hz_processes: Dict[str, subprocess.Popen] = {}  # Topic frequency monitors
        self.topic_hz_threads: Dict[str, threading.Thread] = {}  # Topic frequency reader threads
        self.shutdown_event = threading.Event()
        self.topic_update_thread = None
        
    def add_node(self, config: NodeConfig):
        """Add a node configuration"""
        self.node_configs[config.name] = config
        self.node_status[config.name] = NodeStatus()
        self.log_queues[config.name] = queue.Queue()
        
    def start_node(self, node_name: str) -> bool:
        """Start a specific node"""
        if node_name not in self.node_configs:
            return False
            
        if node_name in self.processes and self.processes[node_name].poll() is None:
            print(f"Node {node_name} is already running")
            return False
            
        config = self.node_configs[node_name]
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '42'
        
        try:
            # Use process group to track all child processes
            process = subprocess.Popen(
                config.command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
                universal_newlines=True,
                bufsize=1,
                preexec_fn=os.setsid  # Create new process group
            )
            
            self.processes[node_name] = process
            status = self.node_status[node_name]
            status.pid = process.pid
            status.running = True
            status.start_time = datetime.now()
            status.status_led = "green"
            
            # Start log monitoring threads
            threading.Thread(target=self._monitor_output, args=(node_name, process.stdout, 'stdout'), daemon=True).start()
            threading.Thread(target=self._monitor_output, args=(node_name, process.stderr, 'stderr'), daemon=True).start()
            
            return True
        except Exception as e:
            print(f"Failed to start {node_name}: {e}")
            return False
            
    def stop_node(self, node_name: str) -> bool:
        """Stop a specific node and all its child processes"""
        if node_name not in self.processes:
            return False
            
        process = self.processes[node_name]
        if process.poll() is None:
            try:
                # First, try to find all ROS2 nodes started by this process
                # This is especially important for launch files
                pgid = os.getpgid(process.pid)
                
                # Send SIGINT (Ctrl+C) to the process group for graceful shutdown
                os.killpg(pgid, signal.SIGINT)
                
                # Wait for graceful shutdown
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # If graceful shutdown fails, send SIGTERM
                    print(f"Node {node_name} didn't stop gracefully, sending SIGTERM...")
                    os.killpg(pgid, signal.SIGTERM)
                    
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        # Last resort: force kill
                        print(f"Force killing node {node_name}...")
                        os.killpg(pgid, signal.SIGKILL)
                        process.wait()
                        
            except ProcessLookupError:
                # Process already terminated
                pass
            except Exception as e:
                print(f"Error stopping node {node_name}: {e}")
                # Fallback to simple kill
                try:
                    process.kill()
                except:
                    pass
                    
        del self.processes[node_name]
        status = self.node_status[node_name]
        status.running = False
        status.status_led = "gray"
        status.pid = None
        status.topic_frequencies.clear()
        return True
        
    def _monitor_output(self, node_name: str, stream, stream_type: str):
        """Monitor process output for logs"""
        status = self.node_status[node_name]
        
        for line in stream:
            if self.shutdown_event.is_set():
                break
                
            timestamp = datetime.now()
            log_entry = {
                'timestamp': timestamp.isoformat(),
                'type': stream_type,
                'message': line.strip()
            }
            
            status.logs.append(log_entry)
            self.log_queues[node_name].put(log_entry)
            
            # Check for errors and warnings
            lower_line = line.lower()
            if 'error' in lower_line or 'fatal' in lower_line:
                status.error_count += 1
                status.last_error = line.strip()
                status.last_error_time = timestamp
                self._update_led_status(node_name)
            elif 'warning' in lower_line or 'warn' in lower_line:
                status.warning_count += 1
                status.last_warning = line.strip()
                status.last_warning_time = timestamp
                self._update_led_status(node_name)
                
    def _update_led_status(self, node_name: str):
        """Update LED status based on recent errors/warnings"""
        status = self.node_status[node_name]
        now = datetime.now()
        
        if not status.running:
            status.status_led = "gray"
        elif status.last_error_time and (now - status.last_error_time) < timedelta(seconds=10):
            status.status_led = "red"
        elif status.last_warning_time and (now - status.last_warning_time) < timedelta(seconds=10):
            status.status_led = "yellow"
        else:
            status.status_led = "green"
            
    def get_node_logs(self, node_name: str, tail: int = 100) -> List[dict]:
        """Get recent logs for a node"""
        if node_name not in self.node_status:
            return []
        return list(self.node_status[node_name].logs)[-tail:]
        
    def update_topic_info(self):
        """Update topic information and manage topic frequency monitors"""
        while not self.shutdown_event.is_set():
            try:
                # Get all topics that should be monitored
                topics_to_monitor = set()
                for node_name, config in self.node_configs.items():
                    if self.node_status[node_name].running:
                        topics_to_monitor.update(config.topics_pub)
                
                # Start/stop topic frequency monitors as needed
                self._manage_topic_monitors(topics_to_monitor)
                    
            except Exception as e:
                print(f"Error updating topic info: {e}")
                
            time.sleep(2)  # Check every 2 seconds
            
    def _parse_topic_list(self, output: str):
        """Parse ros2 topic list -v output"""
        lines = output.strip().split('\n')
        current_section = None
        
        for line in lines:
            line = line.strip()
            if not line:
                continue
                
            if 'Published topics:' in line:
                current_section = 'published'
            elif 'Subscribed topics:' in line:
                current_section = 'subscribed'
            elif line.startswith('*'):
                # Parse topic line
                parts = line[1:].strip().split(' [', 1)
                if len(parts) >= 1:
                    topic_name = parts[0].strip()
                    msg_type = parts[1].rstrip(']') if len(parts) > 1 else ''
                    
                    if topic_name not in self.topic_info:
                        self.topic_info[topic_name] = TopicInfo(name=topic_name, msg_type=msg_type)
                    else:
                        self.topic_info[topic_name].msg_type = msg_type
                        
            elif line and not line.startswith('*') and current_section:
                # This might be a node name under a topic
                pass
                
    def _update_topic_frequency(self, topic_name: str):
        """Update frequency for a specific topic"""
        try:
            # Use ros2 topic hz to get frequency
            result = subprocess.run(
                f'timeout 0.5 ros2 topic hz {topic_name} --window 10 2>/dev/null',
                shell=True,
                capture_output=True,
                text=True
            )
            
            # Parse output for average rate
            for line in result.stdout.split('\n'):
                if 'average rate:' in line:
                    # Extract the frequency value
                    match = re.search(r'average rate: ([0-9.]+)', line)
                    if match:
                        freq = float(match.group(1))
                        self.topic_info[topic_name].frequency = freq
                        
                        # Update node status with frequency
                        for node_name, config in self.node_configs.items():
                            if topic_name in config.topics_pub:
                                self.node_status[node_name].topic_frequencies[topic_name] = freq
                        break
                        
        except Exception as e:
            pass  # Silently ignore frequency update errors
            
    def _manage_topic_monitors(self, active_topics: set):
        """Start and stop topic frequency monitors based on active topics"""
        # Stop monitors for topics that are no longer active
        for topic in list(self.topic_hz_processes.keys()):
            if topic not in active_topics:
                self._stop_topic_monitor(topic)
                
        # Start monitors for new active topics
        for topic in active_topics:
            if topic not in self.topic_hz_processes:
                self._start_topic_monitor(topic)
                
    def _start_topic_monitor(self, topic: str):
        """Start a persistent ros2 topic hz monitor for a topic"""
        try:
            # Check if topic exists first
            check_result = subprocess.run(
                f'ros2 topic list | grep -x "{topic}"',
                shell=True,
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if check_result.returncode != 0:
                return  # Topic doesn't exist
            
            # Start persistent ros2 topic hz process
            hz_process = subprocess.Popen(
                f'ros2 topic hz {topic} --window 10',
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            self.topic_hz_processes[topic] = hz_process
            
            # Start thread to read from this process
            reader_thread = threading.Thread(
                target=self._read_topic_hz_output,
                args=(topic, hz_process),
                daemon=True
            )
            reader_thread.start()
            self.topic_hz_threads[topic] = reader_thread
            
            print(f"Started frequency monitor for {topic}")
            
        except Exception as e:
            print(f"Error starting topic monitor for {topic}: {e}")
            
    def _stop_topic_monitor(self, topic: str):
        """Stop the topic frequency monitor for a topic"""
        try:
            if topic in self.topic_hz_processes:
                process = self.topic_hz_processes[topic]
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                del self.topic_hz_processes[topic]
                
            if topic in self.topic_hz_threads:
                # Thread will exit when process terminates
                del self.topic_hz_threads[topic]
                
            # Clear frequency data for this topic
            for node_name, config in self.node_configs.items():
                if topic in config.topics_pub:
                    if topic in self.node_status[node_name].topic_frequencies:
                        del self.node_status[node_name].topic_frequencies[topic]
                        
            print(f"Stopped frequency monitor for {topic}")
            
        except Exception as e:
            print(f"Error stopping topic monitor for {topic}: {e}")
            
    def _read_topic_hz_output(self, topic: str, process: subprocess.Popen):
        """Read and parse output from a ros2 topic hz process"""
        try:
            for line in iter(process.stdout.readline, ''):
                if self.shutdown_event.is_set():
                    break
                    
                line = line.strip()
                if 'average rate:' in line:
                    # Parse the frequency
                    try:
                        rate_match = re.search(r'average rate: ([0-9.]+)', line)
                        if rate_match:
                            frequency = float(rate_match.group(1))
                            
                            # Update all nodes that publish this topic
                            for node_name, config in self.node_configs.items():
                                if topic in config.topics_pub:
                                    self.node_status[node_name].topic_frequencies[topic] = frequency
                                    
                            # Update topic info
                            if topic in self.topic_info:
                                self.topic_info[topic].frequency = frequency
                                
                    except (ValueError, AttributeError) as e:
                        continue
                        
        except Exception as e:
            print(f"Error reading topic hz output for {topic}: {e}")
        finally:
            # Clean up when thread exits
            if not self.shutdown_event.is_set():
                print(f"Topic hz reader thread for {topic} exited unexpectedly")
            
    def shutdown_all(self):
        """Shutdown all nodes and topic monitors"""
        self.shutdown_event.set()
        
        # Stop all topic frequency monitors
        for topic in list(self.topic_hz_processes.keys()):
            self._stop_topic_monitor(topic)
            
        # Stop all node processes
        for node_name in list(self.processes.keys()):
            self.stop_node(node_name)

class TopicMonitor(Node):
    """ROS2 node for monitoring topic frequencies"""
    
    def __init__(self, process_manager: ProcessManager):
        if not HAS_ROS2:
            return
            
        super().__init__('launch_dashboard_monitor')
        self.process_manager = process_manager
        self.topic_stats = defaultdict(lambda: {
            'publishers': set(),  # Track multiple publishers
            'msg_count': 0,
            'last_msg_time': None,
            'last_calc_time': None,
            'frequency': 0.0
        })
        self.subscribers = {}
        self.topic_to_nodes = defaultdict(set)  # Map topics to publishing nodes
        
        # Create timer to discover topics and calculate frequencies
        self.create_timer(1.0, self.update_topics)
        self.create_timer(0.5, self.calculate_frequencies)
        
    def update_topics(self):
        """Discover and subscribe to topics"""
        try:
            # Get list of topics
            topic_list = self.get_topic_names_and_types()
            
            for topic_name, topic_types in topic_list:
                # Skip system topics
                if topic_name.startswith('/rosout') or topic_name.startswith('/parameter_events'):
                    continue
                    
                # Check if we need to monitor this topic
                should_monitor = False
                for node_name, config in self.process_manager.node_configs.items():
                    if topic_name in config.topics_pub:
                        should_monitor = True
                        self.topic_to_nodes[topic_name].add(node_name)
                        
                if should_monitor and topic_name not in self.subscribers:
                    # Create a generic callback
                    def make_callback(topic):
                        def callback(msg):
                            self.on_message_received(topic)
                        return callback
                    
                    # Try to subscribe (simplified - would need proper type handling in production)
                    try:
                        # For now, we'll just track that the topic exists
                        self.topic_stats[topic_name]['publishers'] = self.get_publishers_info_by_topic(topic_name)
                    except Exception as e:
                        self.get_logger().debug(f'Could not subscribe to {topic_name}: {e}')
                        
        except Exception as e:
            self.get_logger().error(f'Error updating topics: {e}')
    
    def on_message_received(self, topic_name: str):
        """Called when a message is received on a topic"""
        now = self.get_clock().now()
        stats = self.topic_stats[topic_name]
        stats['msg_count'] += 1
        stats['last_msg_time'] = now
        
    def calculate_frequencies(self):
        """Calculate topic frequencies based on message counts"""
        now = self.get_clock().now()
        
        # Use ros2 topic hz command to get accurate frequencies
        for topic_name in self.topic_stats.keys():
            try:
                # Run ros2 topic hz in a subprocess to get frequency
                result = subprocess.run(
                    f'timeout 0.5 ros2 topic hz {topic_name} --window 10 2>/dev/null | grep "average rate" | grep -oE "[0-9]+\.[0-9]+"',
                    shell=True,
                    capture_output=True,
                    text=True
                )
                
                if result.stdout.strip():
                    freq = float(result.stdout.strip())
                    self.topic_stats[topic_name]['frequency'] = freq
                    
                    # Update frequency for all nodes publishing this topic
                    for node_name in self.topic_to_nodes[topic_name]:
                        if node_name in self.process_manager.node_status:
                            self.process_manager.node_status[node_name].topic_frequencies[topic_name] = freq
                            
            except Exception as e:
                self.get_logger().debug(f'Could not get frequency for {topic_name}: {e}')

# Flask web application
app = Flask(__name__)
app.config['SECRET_KEY'] = 'smartdrive-dashboard-2024'
socketio = SocketIO(app, cors_allowed_origins="*")

process_manager = ProcessManager()
topic_monitor = None

@app.route('/')
def index():
    """Serve the main dashboard page"""
    return render_template('dashboard.html')

@app.route('/api/nodes')
def get_nodes():
    """Get all node configurations and status"""
    nodes = []
    for name, config in process_manager.node_configs.items():
        status = process_manager.node_status[name]
        nodes.append({
            'name': name,
            'command': config.command,
            'topics_pub': config.topics_pub,
            'topics_sub': config.topics_sub,
            'expected_freq': config.expected_freq,
            'color': config.color,
            'position': config.position,
            'running': status.running,
            'pid': status.pid,
            'status_led': status.status_led,
            'error_count': status.error_count,
            'warning_count': status.warning_count,
            'topic_frequencies': status.topic_frequencies
        })
    return jsonify(nodes)

@app.route('/api/topics')
def get_topics():
    """Get all topic information including publishers"""
    topics = {}
    
    # Gather topic info from node configs and actual ROS2 state
    for topic_name, topic_info in process_manager.topic_info.items():
        # Find which nodes publish to this topic
        publishers = []
        for node_name, config in process_manager.node_configs.items():
            if topic_name in config.topics_pub and process_manager.node_status[node_name].running:
                publishers.append(node_name)
                
        topics[topic_name] = {
            'name': topic_name,
            'publishers': publishers,
            'frequency': topic_info.frequency,
            'msg_type': topic_info.msg_type
        }
        
    # Also include configured topics that might not be active yet
    for node_name, config in process_manager.node_configs.items():
        for topic in config.topics_pub:
            if topic not in topics:
                topics[topic] = {
                    'name': topic,
                    'publishers': [node_name] if process_manager.node_status[node_name].running else [],
                    'frequency': process_manager.node_status[node_name].topic_frequencies.get(topic, 0),
                    'msg_type': ''
                }
                
    return jsonify(list(topics.values()))

@app.route('/api/node/<node_name>/start', methods=['POST'])
def start_node(node_name):
    """Start a specific node"""
    success = process_manager.start_node(node_name)
    return jsonify({'success': success})

@app.route('/api/node/<node_name>/stop', methods=['POST'])
def stop_node(node_name):
    """Stop a specific node"""
    success = process_manager.stop_node(node_name)
    return jsonify({'success': success})

@app.route('/api/node/<node_name>/logs')
def get_node_logs(node_name):
    """Get logs for a specific node"""
    tail = request.args.get('tail', 100, type=int)
    logs = process_manager.get_node_logs(node_name, tail)
    return jsonify(logs)

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    emit('connected', {'data': 'Connected to launch dashboard'})

@socketio.on('subscribe_logs')
def handle_subscribe_logs(data):
    """Subscribe to real-time logs for a node"""
    node_name = data.get('node_name')
    if node_name in process_manager.log_queues:
        # Start a thread to send logs to this client
        threading.Thread(target=send_logs_to_client, args=(node_name, request.sid), daemon=True).start()

def send_logs_to_client(node_name: str, client_id: str):
    """Send real-time logs to a specific client"""
    log_queue = process_manager.log_queues.get(node_name)
    if not log_queue:
        return
        
    while True:
        try:
            log_entry = log_queue.get(timeout=1)
            socketio.emit('log_update', {
                'node_name': node_name,
                'log': log_entry
            }, room=client_id)
        except queue.Empty:
            continue
        except Exception:
            break

def broadcast_status_updates():
    """Broadcast node status updates to all clients"""
    while not process_manager.shutdown_event.is_set():
        time.sleep(1)
        
        # Update LED statuses
        for node_name in process_manager.node_configs:
            process_manager._update_led_status(node_name)
            
        # Broadcast status
        status_data = {}
        for name, status in process_manager.node_status.items():
            status_data[name] = {
                'running': status.running,
                'status_led': status.status_led,
                'error_count': status.error_count,
                'warning_count': status.warning_count,
                'topic_frequencies': status.topic_frequencies
            }
        socketio.emit('status_update', status_data)

def setup_nodes():
    """Setup node configurations in launch order"""
    
    # Define all nodes in the correct launch order with actual topic names
    nodes = [
        # 1. Multi sensor (ZED camera, IMU, LiDAR)
        NodeConfig(
            name="multi_sensor",
            command="ros2 launch multi_sensor_launcher multi_sensor.launch.py",
            topics_pub=["/zed/zed_node/point_cloud/cloud_registered", "/zed/zed_node/left/image_rect_color", "/imu/data", "/scan"],
            expected_freq={"/zed/zed_node/point_cloud/cloud_registered": 15, "/zed/zed_node/left/image_rect_color": 30, "/imu/data": 100, "/scan": 10},
            color="#3498db",
            position=(100, 100)
        ),
        # 2. Vehicle description (TF tree)
        NodeConfig(
            name="vehicle_description",
            command="ros2 launch psd_vehicle_description view_psd_vehicle.launch.py",
            topics_pub=["/tf", "/tf_static"],
            color="#f39c12",
            position=(100, 200)
        ),
        # 3. VESC driver (motor control) - no specific topics mentioned
        NodeConfig(
            name="vesc_driver",
            command="ros2 launch vesc_driver vesc_driver_node.launch.py",
            topics_pub=[],  # No specific topics mentioned
            topics_sub=["/cmd_vel"],
            expected_freq={},
            color="#2ecc71",
            position=(300, 100)
        ),
        # 4. Camera perception
        NodeConfig(
            name="camera_perception",
            command=f"""ros2 run psd_perception camera_node --ros-args \
                -p engine_path:=/home/psd/psd_ws/src/psd_perception/best.engine \
                -p conf_threshold:=0.5 \
                -p input_size:=640 \
                -p debug:=false \
                -p debug_csv_path:=/media/psd/internalSSD/debugCamera/dashboard_{datetime.now().strftime('%y%m%d_%H%M%S')}.csv \
                -p centroid_mode:=2 \
                -p filter:=false""",
            topics_pub=["/detected_bb", "/possible_cones_xyz"],
            expected_freq={"/detected_bb": 30, "/possible_cones_xyz": 30},
            color="#e74c3c",
            position=(500, 100)
        ),
        # 5. SLAM
        NodeConfig(
            name="slam",
            command="ros2 launch psd_slam graph_slam.launch.py",
            topics_pub=["/vehicle_pose"],
            topics_sub=["/scan", "/imu/data"],
            expected_freq={"/vehicle_pose": 10},
            color="#9b59b6",
            position=(300, 300)
        ),
        # 6. Path planning
        NodeConfig(
            name="path_planning",
            command="ros2 run psd_path_planning exploration_standalone",
            topics_pub=["/trajectory_waypoints"],
            topics_sub=["/vehicle_pose", "/possible_cones_xyz"],
            expected_freq={"/trajectory_waypoints": 5},
            color="#1abc9c",
            position=(500, 300)
        ),
        # 7. MPC controller
        NodeConfig(
            name="psd_mpc",
            command="ros2 launch psd_mpc acados_mpc.launch.py",
            topics_pub=["/cmd_vel"],
            topics_sub=["/vehicle_pose", "/trajectory_waypoints"],
            expected_freq={"/cmd_vel": 20},
            color="#e67e22",
            position=(700, 200)
        )
    ]
    
    for node in nodes:
        process_manager.add_node(node)

def get_network_ips():
    """Get all network IP addresses"""
    ips = []
    try:
        # Get hostname
        hostname = socket.gethostname()
        # Get all IPs associated with hostname
        host_ips = socket.gethostbyname_ex(hostname)[2]
        # Filter out localhost
        ips = [ip for ip in host_ips if not ip.startswith("127.")]
        
        # Alternative method using socket to find actual network interfaces
        if not ips:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                # Connect to a public DNS server to find our IP
                s.connect(("8.8.8.8", 80))
                ips = [s.getsockname()[0]]
            except Exception:
                pass
            finally:
                s.close()
    except Exception as e:
        print(f"Warning: Could not get network IPs: {e}")
    
    return ips if ips else ["localhost"]

def print_access_info(port):
    """Print dashboard access information"""
    print("\n" + "="*60)
    print(" " * 15 + "DASHBOARD ACCESS INFORMATION")
    print("="*60)
    print(f"\nLocal Access:")
    print(f"  → http://localhost:{port}")
    
    network_ips = get_network_ips()
    if network_ips and network_ips != ["localhost"]:
        print(f"\nNetwork Access (from other devices on the same network):")
        for ip in network_ips:
            print(f"  → http://{ip}:{port}")
    
    print("\n" + "="*60)
    print("\nDashboard is running. Press Ctrl+C to stop all nodes and exit.\n")

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='SmartDrive Launch Dashboard')
    parser.add_argument('--port', type=int, default=5000, help='Web server port')
    parser.add_argument('--no-browser', action='store_true', help='Do not open browser automatically')
    parser.add_argument('--auto-start', action='store_true', help='Auto-start all nodes on launch')
    args = parser.parse_args()
    
    # Setup signal handlers
    def signal_handler(sig, frame):
        print("\nShutting down...")
        process_manager.shutdown_all()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Setup nodes
    setup_nodes()
    
    # Auto-start nodes if requested
    if args.auto_start:
        print("Auto-starting all nodes...")
        for node_name in process_manager.node_configs:
            process_manager.start_node(node_name)
            time.sleep(1)  # Delay between launches
    
    # Start status broadcast thread
    threading.Thread(target=broadcast_status_updates, daemon=True).start()
    
    # Start topic info update thread
    process_manager.topic_update_thread = threading.Thread(target=process_manager.update_topic_info, daemon=True)
    process_manager.topic_update_thread.start()
    
    # Initialize ROS2 monitoring if available
    if HAS_ROS2:
        try:
            rclpy.init()
            topic_monitor = TopicMonitor(process_manager)
            threading.Thread(target=lambda: rclpy.spin(topic_monitor), daemon=True).start()
        except Exception as e:
            print(f"Warning: Could not initialize ROS2 monitoring: {e}")
    
    # Print access information
    print_access_info(args.port)
    
    # Start web server
    socketio.run(app, host='0.0.0.0', port=args.port, debug=False, log_output=False)

def test_topic_frequencies():
    """Test function to check if topics exist and get their frequencies"""
    setup_nodes()
    
    print("Testing topic frequencies...")
    print("=" * 50)
    
    for node_name, config in process_manager.node_configs.items():
        print(f"\n{node_name}:")
        if not config.topics_pub:
            print("  (no published topics configured)")
            continue
            
        for topic in config.topics_pub:
            try:
                # Check if topic exists
                result = subprocess.run(
                    f'ros2 topic list | grep -x "{topic}"',
                    shell=True,
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                
                if result.returncode == 0:
                    print(f"  ✓ {topic} (exists)")
                else:
                    print(f"  ✗ {topic} (not found)")
                    
            except Exception as e:
                print(f"  ? {topic} (error: {e})")
                
    print("\nTopic frequency test completed.")

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--test-topics':
        test_topic_frequencies()
    else:
        main()