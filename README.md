# CARLA_Panorama_image

This file make 4 RGB image to 1 Panorama image

Car model :: Tesla model 3
Camera resolution :: (1024,1024)
Camera FOV :: 90'

Panorama resolution :: (3336,760)

###

'util/point.json' file has ouxek oisutuib fir cutting image.
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

# 풀이

```python
n = int(input())
data = list(map(int, input().split()))
data.sort()

result = 0
count = 0

for i in data:
    # 임시그룹에 사람의 수 추가
    count += 1
    if count >= i:
        # 사람의 수와 제일 마지막에 추가된 사람의 수  비교
        result += 1
        count = 0
        # 완성그룹(result)에 1 추가, 임시그룹의 인원 제거
print(result)
```

- 나는 사람을 직접 temp 리스트에 추가하면서 체크했는데, 풀이에서는 count에 인원을 더하면서 체크했다
- 메모리와 속도에서 더 빠를듯(리스트 없음, len() 안구해도 됨)

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
