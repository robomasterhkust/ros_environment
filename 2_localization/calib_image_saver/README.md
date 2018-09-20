# calib_image_saver

A small tool to collect images for calibration. The program will detect the chessboard in the image and save the image with chessboard detected.

## Download code  

Enter your catkin work space:

```
cd YOUR_PATH/catkin_ws/src
  
git clone https://github.com/gaowenliang/calib_image_saver.git
```
## Install

```
cd YOUR_PATH/catkin_ws/

catkin_make
```

## Run

Modify the launch file:

* /image_input: The image topic name, such as "/pg_17221069/image_raw"
* image_path: The path to save images, such as "/home/ubuntu/images". You need to make sure the path exist.
* board_width: Chessboard point size.
* board_height: Chessboard point size.

```
<launch>
    <node pkg="calib_image_saver" type="singleImageSaver" name="saver" output="screen">
        <remap from="/image_input" to="YOUR_IMAGE_TOPIC"/>
        <param name="image_path" type="string" value="YOUR_IMAGE_SAVE_PATH"/>
        <param name="image_name" type="string" value="IMG_"/>
        <param name="rate" type="int" value="9"/>
        <param name="board_width" type="int" value="9"/>
        <param name="board_height" type="int" value="6"/>
        <param name="is_use_OpenCV" type="bool" value="true"/>
        <param name="is_show" type="bool" value="true"/>
    </node>
</launch>
```

Make sure the detected chessboard points (yellow) fully fill the image as dense as possible.

<img src="doc/Distributed.jpg">

