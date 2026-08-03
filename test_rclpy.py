import sys
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
sys.path.insert(0, '/opt/ros/humble/local/lib/python3.10/dist-packages')
try:
    import rclpy
    print("rclpy OK")
except Exception as e:
    print(f"ERROR: {e}")
