(base) chopin@chopin-OMEN-ubuntu:~$ mkdir ins_eskf_ws
(base) chopin@chopin-OMEN-ubuntu:~$ cd ins_eskf_ws/
(base) chopin@chopin-OMEN-ubuntu:~/ins_eskf_ws$ mkdir src
(base) chopin@chopin-OMEN-ubuntu:~/ins_eskf_ws$ cd src
(base) chopin@chopin-OMEN-ubuntu:~/ins_eskf_ws/src$ git clone https://github.com/leo6862/ins_eskf_kitti.git
Cloning into 'ins_eskf_kitti'...
fatal: unable to access 'https://github.com/leo6862/ins_eskf_kitti.git/': gnutls_handshake() failed: The TLS connection was non-properly terminated.
(base) chopin@chopin-OMEN-ubuntu:~/ins_eskf_ws/src$ git clone https://github.com/leo6862/ins_eskf_kitti.git
Cloning into 'ins_eskf_kitti'...
remote: Enumerating objects: 292, done.
remote: Counting objects: 100% (292/292), done.
remote: Compressing objects: 100% (155/155), done.
remote: Total 292 (delta 133), reused 291 (delta 132), pack-reused 0
Receiving objects: 100% (292/292), 9.92 MiB | 1.08 MiB/s, done.
Resolving deltas: 100% (133/133), done.
(base) chopin@chopin-OMEN-ubuntu:~/ins_eskf_ws/src$ cd ..
(base) chopin@chopin-OMEN-ubuntu:~/ins_eskf_ws$ catkin_make
Base path: /home/chopin/ins_eskf_ws
Source space: /home/chopin/ins_eskf_ws/src
Build space: /home/chopin/ins_eskf_ws/build
Devel space: /home/chopin/ins_eskf_ws/devel
Install space: /home/chopin/ins_eskf_ws/install
Creating symlink "/home/chopin/ins_eskf_ws/src/CMakeLists.txt" pointing to "/opt/ros/noetic/share/catkin/cmake/toplevel.cmake"
####
#### Running command: "cmake /home/chopin/ins_eskf_ws/src -DCATKIN_DEVEL_PREFIX=/home/chopin/ins_eskf_ws/devel -DCMAKE_INSTALL_PREFIX=/home/chopin/ins_eskf_ws/install -G Unix Makefiles" in "/home/chopin/ins_eskf_ws/build"
####
-- The C compiler identification is GNU 9.4.0
-- The CXX compiler identification is GNU 9.4.0
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Using CATKIN_DEVEL_PREFIX: /home/chopin/ins_eskf_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/chopin/catkin_ws/install_isolated;/home/chopin/work_space/slam_ifly/devel;/opt/ros/noetic
-- This workspace overlays: /home/chopin/catkin_ws/install_isolated;/home/chopin/work_space/slam_ifly/devel;/opt/ros/noetic
-- Found PythonInterp: /home/chopin/anaconda3/bin/python3 (found suitable version "3.11.5", minimum required is "3") 
-- Using PYTHON_EXECUTABLE: /home/chopin/anaconda3/bin/python3
-- Using Debian Python package layout
-- Could NOT find PY_em (missing: PY_EM) 
CMake Error at /opt/ros/noetic/share/catkin/cmake/empy.cmake:30 (message):
  Unable to find either executable 'empy' or Python module 'em'...  try
  installing the package 'python3-empy'
Call Stack (most recent call first):
  /opt/ros/noetic/share/catkin/cmake/all.cmake:164 (include)
  /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:20 (include)
  CMakeLists.txt:58 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/chopin/ins_eskf_ws/build/CMakeFiles/CMakeOutput.log".
Invoking "cmake" failed
