FROM quay.io/fawkesrobotics/fawkes-builder:f37-ros2

COPY . /workdir
WORKDIR /workdir/build
RUN /bin/bash -l -c "cmake ..  ; make -j doxygen ; make -j"
WORKDIR /workdir
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
