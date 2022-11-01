# 3D-scan-and-reconstruction-realsense-d435i
3D scan and reconstruction system using Intel® RealSense™ Depth Camera D435i

This project works in the system:
- Intel® RealSense™ Depth Camera D435i with its tripod
- Open 3D 12.0
- Python 3.8
- Ubuntu 20.04 LTS
- Processor: Intel® Core™ i5-9600K OC @5.1GHz
- Motherboard: Asus ROG Strix Z390-E Gaming
- RAM: Corsair Vengeance LPX 32GB DDR4 OC @4200MHz
- GPU: NVIDIA GeForce RTX2060 Super, with CUDA 11.5
- Storage: Samsung 850 Evo SATA III 1TB
- Turn table (set the first loop of frame as the indicator of one rotation by changing the `frame_loop_at`)
- Object above the turn table with the maximum recommended height of 25 cm

Procedure:
1. Place the camera 30 cm in front of the object.
2. Turn on the turn table with the object to be scanned on it.
3. Run `3D-Scan-Record-RGBD.py`.
4. Input the distance (cm) from the object's centre to the camera.
5. Input the object's maximum radius (cm) from the centre.
6. Let the camera scan the object until the 360-degree rotation is achieved (hit `spacebar` to pause and `ESC` to exit). The program will exit automatically and save the capture in the `dataset` directory.
7. Run `3D-Construct-RGBD.py` to begin the reconstruction system.
8. Program will show the results of the `.ply` file from the reconstruction system.

Note: if the object is quite complex in terms of the surface (rough surface with many indentations), the scanning process might need to be done twice, from a higher-level point-of-view and lower-level point-of-view to ensure all surfaces are already captured and not leaving holes in the reconstructed model.

Scan position example:

![Screenshot from 2022-10-31 22-43-22](https://user-images.githubusercontent.com/26058697/199151046-d4d7414f-839a-487c-9beb-714deefa5afa.png)

Depth scanning before (top) and after (bottom) optimization of the camera preset:

![Screenshot from 2022-10-31 21-06-55](https://user-images.githubusercontent.com/26058697/199151128-0f753cb9-3154-4647-a280-232bdab6b97c.png)

Result from default and custom/optimized camera preset and reconstruction configuration:

![Screenshot from 2022-10-31 21-07-56](https://user-images.githubusercontent.com/26058697/199151269-efa706c8-ad44-42eb-b1cb-76efcaa801c0.png)

Time elapsed for each of the reconstruction pipelines and its RMSE (Root Mean Square Error):

![Screenshot from 2022-10-31 21-09-58](https://user-images.githubusercontent.com/26058697/199151400-f32b206d-0b76-4fd2-97b3-c31a90466377.png)
