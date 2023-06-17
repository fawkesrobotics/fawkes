FROM quay.io/fawkesrobotics/fawkes-builder:f37

COPY . /workdir
WORKDIR /workdir/build
RUN cmake ..  ; make -j doxygen ; make -j
WORKDIR /workdir
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
