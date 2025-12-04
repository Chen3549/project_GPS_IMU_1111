# linux install
方法一：使用包管理器安装
这种方式比较简单，适合快速安装使用。

步骤
1.更新软件包列表
打开终端，输入以下命令来更新系统的软件包列表：

sudo apt update
2.安装gtest和gmock
输入以下命令来安装Google Test和Google Mock：

sudo apt install libgtest-dev libgmock-dev
3.编译并安装库
虽然通过包管理器安装了gtest和gmock的源码，但这些库并未编译安装。需要手动编译并安装这些库，执行以下命令：

cd /usr/src/googletest 
sudo mkdir build 
cd build 
sudo cmake .. 
sudo make 
sudo make install

方法二：从源码编译安装
如果你需要使用最新版本的gtest，或者想要对编译过程进行更多的控制，可以从源码编译安装。

步骤
# 1.克隆gtest仓库 打开终端，使用git命令克隆Google Test的官方仓库：
git clone https://github.com/google/googletest.git 
cd googletest
# 2.创建构建目录并进入
mkdir build 
cd build
# 3.使用CMake配置项目
cmake ..
# 4.编译项目
make
# 5.安装库 如果你希望将编译好的库安装到系统目录中，可以使用以下命令：
sudo make install
