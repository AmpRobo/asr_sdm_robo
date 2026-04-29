# asr_sdm_esdf_map
> Incremental ESDF Map for motion planning

Euclidean Signed Distance Field (ESDF) is useful for online motion planning of aerial robots
since it can easily query the distance_ and gradient information against obstacles.
Fast incrementally built ESDF map is the bottleneck for conducting real-time motion planning.
In this paper, we investigate this problem and propose a mapping system called *Fiesta* to build
global ESDF map incrementally. By introducing two independent updating queues for inserting and
deleting obstacles separately, and using Indexing Data Structures and Doubly Linked Lists for
map maintenance, our algorithm updates as few as possible nodes using a BFS framework. Our ESDF
map has high computational performance and produces near-optimal results.
We show our method outperforms other up-to-date methods in term of performance and accuracy
by both theory and experiments. We integrate Fiesta into a completed quadrotor system and validate
it by both simulation and onboard experiments. We release our method as open-source software for the community. 

The paper of this method is submitted to the 2019 IEEE/RSJ International Conference on
Intelligent Robots and Systems (IROS 2019).  The draft is shown on arxiv
[here](https://arxiv.org/abs/1903.02144).

## Installation
### Required Library
- Eigen3
- PCL
- OpenCV2
- ROS2

## Usage example

```sh
ros2 launch asr_sdm_esdf_map cow_and_lady_launch.py
ros2 bag play data.bag
```

Cow and lady data set can be downloaded [here](http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/data.bag).
A `rviz` will be opened with the visualization of occupancy grid map and a slice of esdf map.

_For more examples, usage and FAQ, please refer to the [Wiki][wiki].

## Contributing

1. Fork it (<https://github.com/hlx1996/Fiesta/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

<!-- Markdown link & img_ dfn's -->
[wiki]: https://github.com/hlx1996/Fiesta/wiki
