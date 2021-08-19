We will follow the format laid out in lab04 for our development structure. This process will be detailed below with explanation for how and why each step is performed.

The code implementation phase will focus on the src directory in the repository and should have the following structure:

```
src
  -camera_package
    --CMakeLists.txt
    --package.xml
    --include
      ---camera_include.h
    --src
      ---camera_src.cpp
  -moveit_tutorials
    --CMakeLists.txt
    --package.xml
    --include
      ---moveit_tutorials_include.h
    --src
      ---moveit_tutorials_src.cpp
  -fmauch_universal_robot
    --CMakeLists.txt
    --package.xml
    --include
      ---fmauch_universal_robot_include.h
    --src
      ---fmauch_universal_robot_src.cpp
```

Note that it is best practice to branch off from the code_implementation branch into branches that focus on specific aspects of the code, e.g. a branch
for image detection and another for image detection.

The src directory will contain the packages we are developing and not workspaces. The reason for this is to prevent clutter within the commit history. This clutter results
from catkin_make updating many files in your workspace. This means that we are making our own catkin workspaces and using symbolic links to get the packages into our workspaces. The steps to do this will be detailed below.

Clone the repository to your virtual machine. I recommend you clone it into you home directory. You can ensure you do this by typing cd into your terminal and pressing enter
before cloning. To clone the repo run the following command in the terminal:

```
git clone -b code_implementation https://github.com/UNSW-MTRN4230-T2-2021/major-project-mtrn4230_h12a_momsfriendlyrobotcompany.git
```

Create your catkin workspace. Again I recommend you do this in the home directory. Type cd and press enter in you terminal to get to your home directory. Your workspace name
is up to you, I will be using group_project_ws from now on. To create your workspace, run the following commands in your terminal one at a time:

```
mkdir -p ~/group_project_ws/src
cd ~/group_project_ws/
catkin_make
```

Source setup.bash file so that when you run commands meant to build your package, it knows that they come from your workspace. **YOU WILL NEED TO DO THIS EVERYTIME YOU CLOSE YOUR TERMINAL.** To source your setup.bash, run the following commands:

```
cd ~/group_project_ws/
source devel/setup.bash
```

Symbolically link the packages from the repository into your workspace. Note that I am assuming you cloned the repository and made your workspace in the home directory. If you didn't do this you will need to obtain the file path of each of the files yourself to perform this step. To symbolically link the packages into your workspace run the following commands. Only link in the packages that you will be developing under to prevent potentially broken code from affecting your catkin_make:

```
ln -s ~/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/nodes ~/group_project_ws/src
ln -s ~/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo ~/group_project_ws/src
ln -s ~/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/ur_description ~/group_project_ws/src
ln -s ~/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/ur_kinematics ~/group_project_ws/src
ln -s ~/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/models ~/group_project_ws/src
ln -s ~/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/ur5e_epick_moveit_config ~/group_project_ws/src
ln -s ~/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/cv_bridge ~/group_project_ws/src
```

Note that the ln -s means to do a symbolic link, the first file path is for the package inside the repository, the second file path is where we want it (in our src folder of our workspace).

Run the following install commands to ensure tesseract is on your system, if prompted the password of the system is 777:
```
sudo apt install tesseract-ocr 
sudo apt install libtesseract-dev
```

Finalise everything by running the following commands:

```
cd ~/group_project_ws/
catkin_make
source devel/setup.bash
```

To run our code package:

```
roslaunch lab09_gazebo demo.launch
```

Then, open a new tab with ctrl+shift+t:

```
source devel/setup.bash
roslaunch nodes integrated.launch
```

Ideally nothing is broken but if something is broken, hopefully it's just because of unfinished source code

You are now ready to go. To finalise all changes and makWhen developing you code, make sure you are putting your code in the correct package, i.e movement code files should go into the movement package and camera code files should go into the camera package.
