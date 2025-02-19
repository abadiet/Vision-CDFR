# Vision-CDFR

Vision Camera Code for the French Robotics Cup 🏆🇫🇷

> [!WARNING]
> This project is under development and is not yet fully optimized.

> [!NOTE]
> The tests are made on our setup, with pretty bad lights: the detection will be better the day of the event.

![Demo](https://github.com/abadiet/Vision-CDFR/blob/a1ba7d0f6d00c2c8612211e17dad896237baedb1/resources/demo.gif)

## Usage

```
mkdir build
cd build
cmake .. -DCUDA=ON
make
./Vision <base_image> <video_stream_in> <video_stream_out> 
```

## Dependencies

- [OpenCV](https://opencv.org)

## Benchmark

| Architecture            | FPS    |
|-------------------------|--------|
| Apple M3 (CPU Only)     | 20     |
| Jetson Orin Nano Super  | *soon* |
