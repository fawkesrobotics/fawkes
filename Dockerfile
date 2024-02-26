FROM quay.io/fawkesrobotics/fawkes-builder:f39-ros2

COPY . /workdir
WORKDIR /workdir/build
RUN /bin/bash -l -c "cmake ..  ; make -j doxygen --quiet; make -j$(nproc)"
WORKDIR /workdir
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
