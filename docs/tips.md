# 1&emsp;Helpful Functions to Use
The following functions may be helpful for the task.

| Function | Description |
| --- | --- |
| `std::sin(x)` | Returns the sine of `x`. |
| `std::cos(x)` | Returns the cos of `x`. |
| `std::abs(x)` | Returns the absolute value of `x`. |
| `std::pow(x, p)` | Returns $x^p$. |
| `std::atan2(y, x)` | Returns the angle between a vector $(x,y)$ from the positive $x$-axis. The angle $\theta$ is $-\pi \le \theta < \pi$. |
| `std::clamp(x, lo, hi)` | Returns `x` if `x` is between the value `lo` and `hi` inclusive, `lo` if `x` < `lo`, or `hi` if `x` > `hi`. |
| `ee4308::sgn(x)` | Finds the sign of a value `x`, which returns -1, 0, or 1. |
| `ee4308::limitAngle(a)` | Finds the equivalent angle of `a` (in radians) by rotating it by $2\pi$ until it is $-\pi \le a < \pi$. |
| `ee4308::getYawFromQuaternion(q)` | Finds the yaw angle (radians) of a Quaternion object `q` containing the properties `x`, `y`, `z`, and `w`. |
| `ee4308::getQuaternionFromYaw(yaw, &q)` | Converts the `yaw` angle (radians) to a quaternion `q` in-place. |
| `ee4308::initParam(...)` | Initializes a ROS2 parameter to be read from the parameter file. |
| `std::reverse(v.begin(), v.end())` | Reverses a `std::vector` `v` in place. |

# 2&emsp;Using Eigen
Assume `X` and `Y` are `Eigen` matrices.
More information can be found in https://eigen.tuxfamily.org/dox-3.2/group__QuickRefPage.html.

| Syntax | Description |
| --- | --- |
| `Eigen::Vector2d X` | Declares a $2\times1$ `double` matrix `X`. | 
| `Eigen::Vector3d X` | Declares a $3\times1$ `double` matrix `X`. | 
| `Eigen::Vector4d X` | Declares a $4\times1$ `double` matrix `X`. | 
| `Eigen::Matrix2d X` | Declares a $2\times2$ `double` matrix `X`. | 
| `Eigen::Matrix3d X` | Declares a $3\times3$ `double` matrix `X`. | 
| `Eigen::Matrix4d X` | Declares a $4\times4$ `double` matrix `X`. | 
| `Eigen::MatrixXd X` | Declares a dynamically-sized matrix `X`. |
| `X.resize(r, c)` | Resizes a dynamically-sized matrix `X` to `r` rows and `c` columns. |
| `Eigen::Matrix2d::Constant(v)` | Returns a `Eigen::Matrix2d` matrix filled with a scalar value `v`. |
| `Eigen::Matrix2d::Zero()` | Returns a `Eigen::Matrix2d` matrix filled with zeros. |
| `X * Y` | Does matrix multiplication. The sizes must be compatible. |
| `X.transpose()` | Returns a copy of the matrix `X` transposed, without changing `X`. |
| `X.inverse()` | Returns a copy of the matrix `X` inversed, without changing `X`. |
| `X.row(i)` | Returns a reference to the `i`-th row of `X`. `i` starts from zero. |
| `X.column(i)` | Returns a reference to the `i`-th column of `X`. `i` starts from zero. |
| `X.rows()` | Returns the number of rows in `X`. |
| `X.columns()` | Returns the number of columns in `X`. |
| `X(i,j)` | Returns the `i`-th row and `j`-column value of the matrix `X`. `i` and `j` starts from zero. |
| `X(i)` | Returns the `i`-th value of `X`, asssuming `X` is a `Eigen::Vector??` type. |

# 3&emsp;Troubleshooting
1. Use VSCode's Intellisense to aid the programming process, such as displaying the function and property hints.
2. To create a new function, both the header file and source file must be modified.
3. To create a new ROS2 parameter, the parameter file, header file, and source file, must be modified.
4. To troubleshoot, use `std::cout << variable << std::endl` to print the variable value to the terminal.
5. On the terminal, use the `clear` command if there are too many lines.
5. If the program runs too slowly for your computer, you may use the following to build the workspace:
  ```bash
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
6. If Intellisense is highlighting `RCLCPP_INFO_STREAM` indicating that `strlen` cannot be found, build the workspace with:
  ```bash
  colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  ```
  You can combine the above two `--cmake-args` by appending the command above with ` -DCMAKE_BUILD_TYPE=Release`.
7. Sometimes, modifications may not perform the correct way after building. To rebuild cleanly, remove the `build`, `install` and `log` folders before building:
  ```bash
  rm -rf build install log
  colcon build --symlink-install
  ```
  You may append `colcon build` with other `cmake-args`.
