# 3D-scan-and-reconstruction-realsense-d435i
3D scan and reconstruction system using Intel® RealSense™ Depth Camera D435i

This project works in the system:
- Intel® RealSense™ Depth Camera D435i with its tripod
- Open 3D 12.0
- Python 3.8
- Ubuntu 20.04 LTS
- Processor: Intel Core i5-9600k OC @5.1GHz
- Motherboard: Asus ROG Strix Z390-E Gaming
- RAM: Corsair Vengeance LPX 32GB DDR4 OC @4200MHz
- GPU: NVIDIA GeForce RTX2060 Super, with CUDA 11.5
- Storage: Samsung 850 Evo SATA III 1TB
- Turn table with known RPM
- Object about the turn table with the maximum recommended height of 25 cm

Procedure:
1. Place the camera 30 cm in front of the object.
2. Turn on the turn table with the object to be scanned on it.
3. Run `3D-Scan-Record-RGBD.py`.
4. Input the distance (cm) from the object's centre to the camera.
5. Input the object's maximum radius from the centre.
6. Let the camera scan the object until 360-degree rotation achieved (hit `spacebar` to pause and `esc` to exit). The program will save the capture in the `dataset` directory.
7. Run `3D-Construct-RGBD.py` to begin the reconstruction system.
8. Program will show the results of the `.ply` file from the reconstruction system.
