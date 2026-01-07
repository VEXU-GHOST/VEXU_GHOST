from ultralytics import YOLO

# 1. Load your best-performing model from training
model = YOLO("/home/ghost/VEXU_GHOST/best.pt")

# 2. Export to TensorRT (.engine)
# imgsz=[480, 640] ensures it matches your RealSense exactly (Height, Width)
# half=True is the "Magic Button" for 2x speed on Jetson
# workspace=4 gives TensorRT room to find the best math paths
model.export(
    format='engine', 
    device=0, 
    half=True, 
    imgsz=[480, 640],
    workspace=4,
    simplify=True
)