#	Copyright (c) 2019, Andre Gaschler
#	All rights reserved.
#
#	Redistribution and use in source and binary forms, with or without
# modification,
#	are permitted provided that the following conditions are met:
#
#	* Redistributions of source code must retain the above copyright notice,
# this
#	  list of conditions and the following disclaimer.
#
#	* Redistributions in binary form must reproduce the above copyright
# notice, this
#	  list of conditions and the following disclaimer in the documentation
# and/or
#	  other materials provided with the distribution.
#
#	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND
#	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED
#	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR
#	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES
#	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES;
#	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON
#	ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS
#	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

FROM ubuntu:18.04
RUN apt-get update && \
    apt-get install -y sudo nano build-essential cmake ninja-build wget \
        libeigen3-dev libcgal-dev libcoin80-dev \
        libqt4-dev libqt4-opengl-dev libsoqt4-dev libqhull-dev \
        libpthread-stubs0-dev && \
    rm -rf /var/lib/apt/lists/*
RUN wget https://github.com/google/googletest/archive/release-1.8.0.tar.gz
RUN tar -xzf release-1.8.0.tar.gz && \
    cd googletest-release-1.8.0 && \
    mkdir -p build && cd build && \
    cmake .. && make -j8 && make install
COPY . /src/
RUN mkdir /build && \
    cd /build && \
    cmake -GNinja -DCMAKE_BUILD_TYPE="RelWithDebInfo" \
        -DMORE_COMPILE_ERRORS=On \
        /src
RUN cd /build && \
    ninja -j8
RUN cd /build && \
    ctest --verbose

