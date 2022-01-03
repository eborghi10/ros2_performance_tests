# Throughput test in ROS 2

Launch ROS 2 nodes under test:

```bash
ros2 launch throughput_test test.launch.py
```

This command will automatically start a tracing session using [`ros2_tracing`](https://gitlab.com/ros-tracing/ros2_tracing/-/tree/master) and save it to `$HOME/.ros/tracing/my-tracing-session`.

**NOTE:** Since this requires some privileged access which we don't have inside Docker, I had to make some changes to the `ros2_tracing` project.

If the [`lttng-senssiond`](https://man7.org/linux/man-pages/man8/lttng-sessiond.8.html) service daemon is not running, you can do it manually with:

```bash
lttng-sessiond --no-kernel -v

ros2 trace
```

To postprocess the generated files with [`tracetools_analysis`](https://gitlab.com/ros-tracing/tracetools_analysis/-/tree/foxy), run these commands:

```bash
ros2 trace-analysis convert $HOME/.ros/tracing/my-tracing-session

ros2 trace-analysis process $HOME/.ros/tracing/my-tracing-session > $HOME/.ros/tracing/my-tracing-session/processed
```

You can visualize this data using a Jupyter Notebook using: `jupyter-notebook` and then clicking on the provided URL.
