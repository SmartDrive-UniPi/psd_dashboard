# SmartDrive Launch Dashboard

A web-based dashboard for managing and monitoring ROS2 nodes in the SmartDrive autonomous vehicle system.

## Features

- **Real-time Node Monitoring**: View status, logs, and performance metrics of all ROS2 nodes
- **Interactive Pipeline Visualization**: Visual graph showing node connections and data flow
- **Node Management**: Start/stop individual nodes or all nodes with proper sequencing
- **Live Topic Monitoring**: Real-time frequency monitoring for published topics
- **Error and Warning Tracking**: Visual indicators for node health and issues
- **Log Viewer**: Real-time log streaming with filtering capabilities

## Prerequisites

- Python 3.8+
- ROS2 (Humble or later)
- Flask and Flask-SocketIO
- Modern web browser

## Installation

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Ensure ROS2 environment is properly sourced:
```bash
source /opt/ros/humble/setup.bash
source ~/psd_ws/install/setup.bash
```

## Usage

### Starting the Dashboard

```bash
# Basic usage
python3 launch_dashboard.py

# Custom port
python3 launch_dashboard.py --port 8080

# Auto-start all nodes on launch
python3 launch_dashboard.py --auto-start

# Don't open browser automatically
python3 launch_dashboard.py --no-browser
```

### Accessing the Dashboard

Once started, the dashboard will be available at:
- Local access: `http://localhost:5000`
- Network access: `http://<your-ip>:5000`

The dashboard will display access URLs when started.

### Node Configuration

The dashboard manages the following ROS2 nodes in the correct launch sequence:

1. **multi_sensor** - ZED camera, IMU, and LiDAR
2. **vehicle_description** - Robot description and TF tree
3. **vesc_driver** - Motor control interface
4. **camera_perception** - Object detection and perception
5. **slam** - Simultaneous localization and mapping
6. **path_planning** - Route planning and exploration
7. **psd_mpc** - Model predictive controller

## Dashboard Interface

### Main View

- **Header**: Contains SmartDrive logo, title, and global start/stop controls
- **Pipeline View**: Interactive visualization of node connections and status
- **Node List**: Detailed status panel with individual node controls

### Node Status Indicators

- 🟢 **Green**: Node running normally
- 🟡 **Yellow**: Node running with warnings
- 🔴 **Red**: Node has errors
- ⚫ **Gray**: Node stopped

### Controls

- **Start All Nodes**: Launches nodes in proper sequence with delays
- **Stop All Nodes**: Gracefully shuts down nodes in reverse order
- **Individual Controls**: Start, stop, or view logs for specific nodes
- **Log Viewer**: Click any node to view real-time logs

## Technical Details

### Architecture

- **Flask Backend**: Serves web interface and API endpoints
- **WebSocket Communication**: Real-time updates for status and logs
- **Process Management**: Handles ROS2 node lifecycle with proper cleanup
- **Topic Monitoring**: Tracks topic frequencies and publishers
- **Visualization**: Uses vis.js network library for pipeline graph

### API Endpoints

- `GET /` - Main dashboard interface
- `GET /api/nodes` - Node configurations and status
- `GET /api/topics` - Topic information and frequencies
- `POST /api/node/<name>/start` - Start specific node
- `POST /api/node/<name>/stop` - Stop specific node
- `GET /api/node/<name>/logs` - Get node logs

### WebSocket Events

- `status_update` - Broadcast node status changes
- `topic_update` - Topic frequency updates
- `log_update` - Real-time log streaming
- `subscribe_logs` - Subscribe to specific node logs

## Configuration

Node configurations are defined in the `setup_nodes()` function in `launch_dashboard.py`. Each node includes:

- Launch command
- Published/subscribed topics
- Expected topic frequencies
- Display color and position

## Troubleshooting

### Common Issues

1. **ROS2 not found**: Ensure ROS2 is properly installed and sourced
2. **Port already in use**: Use `--port` flag to specify different port
3. **Nodes won't start**: Check ROS_DOMAIN_ID (set to 42) and workspace setup
4. **No topic frequencies**: Verify nodes are publishing and topics exist

### Testing Topics

```bash
# Test if topics exist and get frequencies
python3 launch_dashboard.py --test-topics
```

### Logs

Node logs are captured and displayed in the dashboard. For debugging:
- Check individual node logs in the dashboard
- Monitor console output from `launch_dashboard.py`
- Verify ROS2 node status with `ros2 node list`

## Development

### File Structure

```
psd_dashboard/
├── launch_dashboard.py     # Main application
├── requirements.txt        # Python dependencies
├── templates/
│   ├── dashboard.html     # Web interface
│   └── logo_smartdrive.jpeg  # SmartDrive logo
└── README.md             # This file
```

### Adding New Nodes

1. Define node configuration in `setup_nodes()`
2. Specify topics and expected frequencies
3. Set display properties (color, position)
4. Update launch sequence if needed

## License

Part of the SmartDrive autonomous vehicle project.