#!/usr/bin/env python3
"""
HTTP server for viewing camera feed in browser.

Serves the latest annotated image via HTTP so you can view it
in your Windows browser over SSH/network.

Perfect for headless systems where you can't use GUI viewers.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import http.server
import socketserver
import threading
import os
import tempfile
from pathlib import Path
from datetime import datetime
from typing import Optional
import io


class ImageHTTPServer(Node):
    """HTTP server that serves latest annotated image."""
    
    def __init__(self):
        super().__init__('image_http_server')
        
        # Parameters
        self.declare_parameter('input_topic', '/detection_visualization/image_annotated')
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')  # Listen on all interfaces
        
        input_topic = self.get_parameter('input_topic').value
        port = self.get_parameter('port').value
        host = self.get_parameter('host').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Latest image (as bytes)
        self.latest_image_bytes: Optional[bytes] = None
        self.latest_image_lock = threading.Lock()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        # HTTP Server setup
        self.port = port
        self.host = host
        self.httpd = None
        self.server_thread = None
        
        # Start HTTP server in background thread
        self.start_http_server()
        
        # Get Jetson IP address
        import socket
        try:
            # Get local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
        except:
            local_ip = "localhost"
        
        self.get_logger().info(
            f"Image HTTP server initialized:\n"
            f"  Input topic: {input_topic}\n"
            f"  Listening on: {host}:{port}\n"
            f"\n"
            f"🌐 View in your Windows browser:\n"
            f"   http://{local_ip}:{port}/\n"
            f"   or\n"
            f"   http://localhost:{port}/ (if viewing on Jetson)\n"
            f"\n"
            f"📝 The page auto-refreshes every 0.5 seconds\n"
            f"   You'll see the latest annotated camera feed with bounding boxes!\n"
        )
    
    def image_callback(self, msg: Image):
        """Convert latest image to JPEG bytes for serving."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert to JPEG bytes
            _, img_bytes = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            img_bytes = img_bytes.tobytes()
            
            # Store latest image
            with self.latest_image_lock:
                self.latest_image_bytes = img_bytes
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}", exc_info=True)
    
    def start_http_server(self):
        """Start HTTP server in background thread."""
        # Create handler factory that captures parent reference
        def make_handler(parent_node):
            class ImageHandler(http.server.SimpleHTTPRequestHandler):
                def do_GET(self):
                    """Handle GET requests."""
                    if self.path == '/' or self.path == '/index.html':
                        self.send_response(200)
                        self.send_header('Content-type', 'text/html')
                        self.end_headers()
                        
                        # HTML page with auto-refresh
                        html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Detection Visualization</title>
    <meta http-equiv="refresh" content="0.5">
    <style>
        body {{ margin: 0; padding: 20px; background: #1e1e1e; color: #fff; font-family: Arial, sans-serif; }}
        h1 {{ text-align: center; color: #4CAF50; }}
        .container {{ text-align: center; max-width: 100%; }}
        img {{ max-width: 100%; height: auto; border: 2px solid #4CAF50; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.3); }}
        .info {{ margin-top: 10px; color: #aaa; font-size: 12px; }}
        .timestamp {{ color: #888; font-size: 10px; }}
    </style>
</head>
<body>
    <h1>🎥 Detection Visualization</h1>
    <div class="container">
        <img src="/image.jpg?t={datetime.now().timestamp()}" alt="Detection Visualization" />
        <div class="info">
            Auto-refreshing every 0.5 seconds
            <div class="timestamp">Last update: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</div>
        </div>
    </div>
</body>
</html>"""
                        self.wfile.write(html.encode())
                        
                    elif self.path.startswith('/image.jpg'):
                        # Serve latest image
                        with parent_node.latest_image_lock:
                            img_bytes = parent_node.latest_image_bytes
                        
                        if img_bytes is None:
                            self.send_response(503)
                            self.send_header('Content-type', 'text/plain')
                            self.end_headers()
                            self.wfile.write(b'No image available yet')
                        else:
                            self.send_response(200)
                            self.send_header('Content-type', 'image/jpeg')
                            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                            self.send_header('Pragma', 'no-cache')
                            self.send_header('Expires', '0')
                            self.end_headers()
                            self.wfile.write(img_bytes)
                    else:
                        self.send_response(404)
                        self.end_headers()
                
                def log_message(self, format, *args):
                    """Suppress HTTP server log messages."""
                    pass  # Comment out this line if you want to see HTTP requests
            
            return ImageHandler
        
        try:
            handler_class = make_handler(self)
            self.httpd = socketserver.TCPServer((self.host, self.port), handler_class)
            self.httpd.allow_reuse_address = True
            
            # Start server in background thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()
            
            self.get_logger().info(f"HTTP server started on {self.host}:{self.port}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start HTTP server: {e}", exc_info=True)


def main(args=None):
    rclpy.init(args=args)
    
    node = ImageHTTPServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.httpd:
            node.httpd.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
