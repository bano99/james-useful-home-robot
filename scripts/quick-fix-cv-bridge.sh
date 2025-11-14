#!/bin/bash

# Quick fix for cv_bridge Python compatibility
# This creates a Python-only cv_bridge wrapper that works with CUDA OpenCV

set -e

echo "=========================================="
echo "Quick cv_bridge Python Fix"
echo "=========================================="
echo ""
echo "This creates a Python wrapper for cv_bridge that works with CUDA OpenCV"
echo ""

# Create a Python cv_bridge wrapper
mkdir -p ~/.local/lib/python3.8/site-packages/cv_bridge_cuda

cat > ~/.local/lib/python3.8/site-packages/cv_bridge_cuda/__init__.py <<'EOF'
"""
cv_bridge wrapper for CUDA OpenCV compatibility
This provides basic cv_bridge functionality without C++ boost bindings
"""

import cv2
import numpy as np
from sensor_msgs.msg import Image
import sys

class CvBridge:
    """
    Pure Python implementation of cv_bridge for CUDA OpenCV compatibility
    """
    
    def __init__(self):
        pass
    
    def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
        """
        Convert ROS Image message to OpenCV image
        
        Args:
            img_msg: sensor_msgs/Image message
            desired_encoding: Desired OpenCV encoding (bgr8, rgb8, mono8, etc.)
        
        Returns:
            OpenCV image (numpy array)
        """
        if desired_encoding == "passthrough":
            desired_encoding = img_msg.encoding
        
        # Get image dimensions
        height = img_msg.height
        width = img_msg.width
        
        # Convert based on encoding
        if img_msg.encoding == "bgr8":
            dtype = np.uint8
            channels = 3
        elif img_msg.encoding == "rgb8":
            dtype = np.uint8
            channels = 3
        elif img_msg.encoding == "mono8" or img_msg.encoding == "8UC1":
            dtype = np.uint8
            channels = 1
        elif img_msg.encoding == "mono16" or img_msg.encoding == "16UC1":
            dtype = np.uint16
            channels = 1
        elif img_msg.encoding == "bgra8":
            dtype = np.uint8
            channels = 4
        elif img_msg.encoding == "rgba8":
            dtype = np.uint8
            channels = 4
        elif img_msg.encoding == "32FC1":
            dtype = np.float32
            channels = 1
        else:
            raise ValueError(f"Unsupported encoding: {img_msg.encoding}")
        
        # Reshape data
        if channels == 1:
            cv_image = np.frombuffer(img_msg.data, dtype=dtype).reshape(height, width)
        else:
            cv_image = np.frombuffer(img_msg.data, dtype=dtype).reshape(height, width, channels)
        
        # Convert encoding if needed
        if img_msg.encoding == "rgb8" and desired_encoding == "bgr8":
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        elif img_msg.encoding == "bgr8" and desired_encoding == "rgb8":
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        return cv_image
    
    def cv2_to_imgmsg(self, cv_image, encoding="bgr8", header=None):
        """
        Convert OpenCV image to ROS Image message
        
        Args:
            cv_image: OpenCV image (numpy array)
            encoding: Image encoding (bgr8, rgb8, mono8, etc.)
            header: Optional ROS header
        
        Returns:
            sensor_msgs/Image message
        """
        img_msg = Image()
        
        if header is not None:
            img_msg.header = header
        
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = encoding
        
        if len(cv_image.shape) == 2:
            # Grayscale
            img_msg.step = img_msg.width
            channels = 1
        else:
            # Color
            channels = cv_image.shape[2]
            img_msg.step = img_msg.width * channels
        
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tobytes()
        
        return img_msg
    
    def compressed_imgmsg_to_cv2(self, compressed_msg, desired_encoding="passthrough"):
        """
        Convert ROS CompressedImage message to OpenCV image
        
        Args:
            compressed_msg: sensor_msgs/CompressedImage message
            desired_encoding: Desired OpenCV encoding
        
        Returns:
            OpenCV image (numpy array)
        """
        np_arr = np.frombuffer(compressed_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if desired_encoding == "rgb8":
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        return cv_image
    
    def cv2_to_compressed_imgmsg(self, cv_image, dst_format="jpg"):
        """
        Convert OpenCV image to ROS CompressedImage message
        
        Args:
            cv_image: OpenCV image (numpy array)
            dst_format: Compression format (jpg, png)
        
        Returns:
            sensor_msgs/CompressedImage message
        """
        from sensor_msgs.msg import CompressedImage
        
        msg = CompressedImage()
        msg.format = dst_format
        
        if dst_format == "jpg" or dst_format == "jpeg":
            msg.data = cv2.imencode('.jpg', cv_image)[1].tobytes()
        elif dst_format == "png":
            msg.data = cv2.imencode('.png', cv_image)[1].tobytes()
        else:
            raise ValueError(f"Unsupported format: {dst_format}")
        
        return msg

# For compatibility, also expose at module level
def imgmsg_to_cv2(img_msg, desired_encoding="passthrough"):
    bridge = CvBridge()
    return bridge.imgmsg_to_cv2(img_msg, desired_encoding)

def cv2_to_imgmsg(cv_image, encoding="bgr8", header=None):
    bridge = CvBridge()
    return bridge.cv2_to_imgmsg(cv_image, encoding, header)

EOF

# Create a script to use this wrapper
cat > ~/.local/bin/use-cuda-cv-bridge <<'EOF'
#!/bin/bash
# Use CUDA-compatible cv_bridge
export PYTHONPATH=~/.local/lib/python3.8/site-packages:$PYTHONPATH
exec "$@"
EOF

chmod +x ~/.local/bin/use-cuda-cv-bridge

echo ""
echo "=========================================="
echo "Quick Fix Complete!"
echo "=========================================="
echo ""
echo "A Python-only cv_bridge has been installed."
echo ""
echo "Option 1: Use in Python scripts"
echo "  from cv_bridge_cuda import CvBridge"
echo ""
echo "Option 2: Add to PYTHONPATH (recommended)"
echo "  export PYTHONPATH=~/.local/lib/python3.8/site-packages:\$PYTHONPATH"
echo ""
echo "Test it:"
echo "  python3 -c 'from cv_bridge_cuda import CvBridge; print(\"OK\")'"
echo ""
echo "Note: This is a pure Python implementation. For full C++ performance,"
echo "run: ./fix-cv-bridge-cuda.sh to rebuild cv_bridge from source."
echo ""

