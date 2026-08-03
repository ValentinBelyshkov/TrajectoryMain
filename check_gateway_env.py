import os
import sys

print("PYTHONPATH:", os.environ.get('PYTHONPATH', 'NOT SET'))
print("LD_LIBRARY_PATH:", os.environ.get('LD_LIBRARY_PATH', 'NOT SET'))
print("PATH:", os.environ.get('PATH', 'NOT SET')[:200])

try:
    import rclpy
    print("rclpy: IMPORT OK")
except Exception as e:
    print(f"rclpy: IMPORT FAILED: {e}")
