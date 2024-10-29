# Documentation

The main documentation effort is found here in the `doc` folder.
The current documentation for `main` can be found here [![Documentation](https://img.shields.io/badge/Documentation-Github_Pages-blue)](https://fzi-forschungszentrum-informatik.github.io/ros2_ros_bt_py/index.html)

For building the documentation locally simply execute the following commands in your shell to get
a browsable HTML documentation:

```bash
$ cd ros_bt_py/doc
$ make html
$ cd build
$ python -m http.server & xdg-open http://localhost:8000/html
```
