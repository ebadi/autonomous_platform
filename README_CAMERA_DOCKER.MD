Before running the OAK-D camera node, build the docker with:

```bash
docker build --build-arg USE_RVIZ=1 -t depthai-ros .
```

To start OAK-D camera nodes enter the High_Level_Control_Computer directory and run:

```bash
./launch_depthai_ros.sh
```

This will launch the camera at the resolution 208 x  156, the resolution can be change by changing rgbScaleDinominator and rgbScaleNumerator in 'launch_depthai_ros.sh', however there is a latancy at higher resolutions for the depth image. Possible resolutions can be seen in the table below:

4056 x 3040 *  2/13 -->  624 x  468
4056 x 3040 *  2/39 -->  208 x  156
4056 x 3040 *  2/51 -->  160 x  120
4056 x 3040 *  4/13 --> 1248 x  936
4056 x 3040 *  4/26 -->  624 x  468
4056 x 3040 *  4/29 -->  560 x  420
4056 x 3040 *  4/35 -->  464 x  348
4056 x 3040 *  4/39 -->  416 x  312
4056 x 3040 *  6/13 --> 1872 x 1404
4056 x 3040 *  6/39 -->  624 x  468
4056 x 3040 *  7/25 --> 1136 x  852
4056 x 3040 *  8/26 --> 1248 x  936
4056 x 3040 *  8/39 -->  832 x  624
4056 x 3040 *  8/52 -->  624 x  468
4056 x 3040 *  8/58 -->  560 x  420
4056 x 3040 * 10/39 --> 1040 x  780
4056 x 3040 * 10/59 -->  688 x  516
4056 x 3040 * 12/17 --> 2864 x 2146
4056 x 3040 * 12/26 --> 1872 x 1404
4056 x 3040 * 12/39 --> 1248 x  936
4056 x 3040 * 13/16 --> 3296 x 2470
4056 x 3040 * 14/39 --> 1456 x 1092
4056 x 3040 * 14/50 --> 1136 x  852
4056 x 3040 * 14/53 --> 1072 x  804
4056 x 3040 * 16/39 --> 1664 x 1248
4056 x 3040 * 16/52 --> 1248 x  936
