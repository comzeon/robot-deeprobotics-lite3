# Livox 源码 vendor 目录

`docker build` 的 WSL2/构建 VM 访问不到 GitHub（`Failed to connect to github.com:443 … Connection timed out`），所以这两个仓库**先在本机下载好**，构建时 `COPY` 进镜像，不依赖构建网络。

## 下载命令（在本机终端跑，这机器能访问 GitHub）

```bash
# 在 robot-deeprobotics-lite3 包根目录下：
mkdir -p container/vendor

# Livox-SDK2 — 固定 v1.3.1（MID-360S 兼容要求 >= v1.3.1）
git clone --depth 1 --branch v1.3.1 https://github.com/Livox-SDK/Livox-SDK2.git container/vendor/Livox-SDK2

# livox_ros_driver2 — 固定 1.2.6（首个支持 MID-360S 的版本）
git clone --depth 1 --branch 1.2.6 https://github.com/Livox-SDK/livox_ros_driver2.git container/vendor/livox_ros_driver2
```

完成后 `container/vendor/` 应包含：
```
container/vendor/Livox-SDK2/            （含 CMakeLists.txt、sdk_core/ 等）
container/vendor/livox_ros_driver2/     （含 build.sh、config/、launch_ROS2/ 等）
```

## 构建

```bash
bash container/start.sh
```

Dockerfile 会把这两个目录 `COPY` 进去编译（`Livox-SDK2` → `cmake && make install` 到 /usr/local；`livox_ros_driver2` → `build.sh humble` colcon 到 /livox_ws/install）。

## 注意事项

- 目录不存在时 `docker build` 会直接报 `COPY failed: no source files were specified`——先跑上面的下载命令。
- `container/vendor/*` 已被 `.gitignore` 忽略，源码**不会**提交进 git 仓库（体积大、且是你的本地构建输入）。
- 如需换镜像源（GitHub 也连不上的机器），可把上面的 URL 换成 Gitee 镜像，例如：
  - `https://gitee.com/jwch/Livox-SDK2.git`（SDK2）
  - `https://gitee.com/xlhou/livox_ros_driver2.git`（driver2，注意核对版本含 MID-360S 支持）
