# CARLA_Panorama_image

This file make 4 RGB image to 1 Panorama image

Car model :: Tesla model 3

Camera resolution :: 1024,1024

Panorama resolution :: (3336,760)

###

'util/point.json' file has pixel position for cutting image.
If you want to change the resolution and 'f value'' in 'subscriber.py', you should reestimate the position and f value

Sample Image(4 images)

![1](https://user-images.githubusercontent.com/98318559/150777125-a3a5db6d-6c09-423d-817c-0248717911b4.png)

Panorama Image

![Pano](https://user-images.githubusercontent.com/98318559/150776300-fc609d7c-b0a1-4114-8df5-7e40aea06174.jpg)

Output Image

![Pano_cut](https://user-images.githubusercontent.com/98318559/150776333-c954b199-1c96-4059-97ab-39ede71e5cab.jpg)

### Requirments

carla: URL "https://carla.readthedocs.io/en/latest/start_quickstart/"

ros(Melodic): URL "http://wiki.ros.org/melodic/Installation/Ubuntu"

Checked at Ubuntu 18.04 + ros Melodic

### How to run

```shell
python ros_publisher.py
```

open another window

```shell
python ros_subscriber.py
```

Default settings of subscriber is publishing mode that publish Panorama image.
You can choose to turn on Publish mode or Save mode

If you turn on the Save mode

```shell
python ros_subscriber.py --publish False --save True
```

At first, it will make a directory that named with 'YYYYMMDD_HHMMSS'

After that, the panorama image will be saved.
