# colcon

colcon is an iteration on the ROS build tools catkin_make, catkin_make_isolated, catkin_tools and ament_tools.

## install colcon

```bash 
sudo apt install python3-colcon-common-extensions
```

## colcon manuel

```bash 
colcon -h

# usage: colcon [-h] [--log-base LOG_BASE] [--log-level LOG_LEVEL]
#               {build,extension-points,extensions,graph,info,list,metadata,test,test-result,version-check} ...

# options:
#   -h, --help            show this help message and exit
#   --log-base LOG_BASE   The base path for all log directories (default: ./log, to disable: /dev/null)
#   --log-level LOG_LEVEL
#                         Set log level for the console output, either by numeric or string value (default: warning)

# colcon verbs:
#   build                 Build a set of packages
#   extension-points      List extension points
#   extensions            List extensions
#   graph                 Generate a visual representation of the dependency graph
#   info                  Package information
#   list                  List packages, optionally in topological ordering
#   metadata              Manage metadata of packages
#   test                  Test a set of packages
#   test-result           Show the test results generated when testing a set of
#                         packages
#   version-check         Compare local package versions with PyPI

#   {build,extension-points,extensions,graph,info,list,metadata,test,test-result,version-check}
#                         call `colcon VERB -h` for specific help

# Environment variables:
#   CMAKE_COMMAND         The full path to the CMake executable
#   COLCON_ALL_SHELLS     Flag to enable all shell extensions
#   COLCON_COMPLETION_LOGFILE
#                         Set the logfile for completion time
#   COLCON_DEFAULTS_FILE  Set path to the yaml file containing the default values
#                         for the command line arguments (default:
#                         $COLCON_HOME/defaults.yaml)
#   COLCON_DEFAULT_EXECUTOR
#                         Select the default executor extension
#   COLCON_DEFAULT_OUTPUT_STYLE
#                         Select the default output style extension
#   COLCON_EXTENSION_BLOCKLIST
#                         Block extensions which should not be used
#   COLCON_HOME           Set the configuration directory (default: ~/.colcon)
#   COLCON_LOG_LEVEL      Set the log level (debug|10, info|20, warn|30,
#                         error|40, critical|50, or any other positive numeric
#                         value)
#   COLCON_WARNINGS       Set the warnings filter similar to PYTHONWARNINGS
#                         except that the module entry is implicitly set to
#                         'colcon.*'
#   CTEST_COMMAND         The full path to the CTest executable
#   POWERSHELL_COMMAND    The full path to the PowerShell executable

# For more help and usage tips, see https://colcon.readthedocs.io
```

## ROS2 workspace structure

```bash 
├── colcon_ws   # ros2 workspace
│   ├── build   # intermediate files are stored
│   ├── install # each package will be installed to
│   ├── log     # logging information
│   └── src             
│       └── hello_world_publisher # ros2 package(s)
```

## create ros2 workspace

```bash 
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## colcon build

builds packages in the ros2 workspace

```bash 
colcon build

# allows the installed files to be changed by changing the files in the source space
colcon build --symlink-install

# build packages parallel
colcon build --executor parallel

# build the packages one by one 
colcon build --executor sequential

## build in debug mode
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## colcon test

run tests in the ros2 workspace

```bash 
colcon test
```

## setup colcon_cd

```bash 
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
```

## colcon tab completion

```bash 
sudo apt-get install python3-colcon-argcomplete
```

## TIPS

### COLCON_IGNORE

use for not including the build

### avoid tests

```bash 
colcon build --cmake-args -DBUILD_TESTING=0
```

### run a single particular test

```bash 
colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
```

### colcon mixins

```bash
sudo apt install python3-colcon-mixin

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

# then try
colcon build --mixin debug
```