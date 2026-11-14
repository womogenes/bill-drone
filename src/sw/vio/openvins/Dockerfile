# OpenVINS without ROS and this runner, for machines whose OpenCV is not 4 (Arch has 5): docker build -t bill-openvins sw/vio/openvins
# Run: docker run --rm -v $HOME:$HOME bill-openvins run_euroc <dataset>/estimator_config.yaml <dataset> <out.csv>  (absolute paths)
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends build-essential cmake git ca-certificates libeigen3-dev libopencv-dev libopencv-contrib-dev libboost-all-dev libceres-dev && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 https://github.com/rpng/open_vins.git /ov && mkdir /ov/ov_msckf/build && cd /ov/ov_msckf/build && cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=OFF && make -j$(nproc) ov_msckf_lib
COPY CMakeLists.txt run_euroc.cpp /runner/
RUN mkdir /runner/build && cd /runner/build && cmake .. -DCMAKE_BUILD_TYPE=Release -DOPENVINS=/ov -DOPENVINS_BUILD=/ov/ov_msckf/build && make && cp run_euroc /usr/local/bin/
ENV LD_LIBRARY_PATH=/ov/ov_msckf/build
