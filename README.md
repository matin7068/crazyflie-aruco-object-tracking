# crazyflie-aruco-object-tracking
This project enables a Crazyflie drone with an ArUco marker to autonomously navigate toward a fixed yellow reference marker. Using computer vision and ZeroMQ, the system tracks both markers and adjusts the drone's position until alignment is achieved. The drone lands automatically upon reaching the target.

## Features
- Tracks both an ArUco marker and a fixed yellow reference marker.
- Uses OpenCV for computer vision and ZeroMQ for real-time data communication.
- Commands the Crazyflie to move toward the reference marker autonomously.
- Keeps running until manually terminated.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/matin7068/crazyflie-aruco-object-tracking.git
   cd crazyflie-aruco-object-tracking

## Install Dependencies
```bash
   pip install -r requirements.txt
```
   
## Usage
- Start the ArUco marker detection script (publisher).
- Run the Crazyflie subscriber script to receive position data and control the drone:
   ```bash
   python subscriber.py


# Requirements
- Python 3.x
- OpenCV
- ZeroMQ (pyzmq)
- Crazyflie Python API (cflib)
