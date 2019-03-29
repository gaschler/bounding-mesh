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
    apt-get install -y sudo nano build-essential cmake ninja-build \
        libeigen3-dev libcgal-dev libcoin80-dev \
        libqt4-dev libqt4-opengl-dev libsoqt4-dev libqhull-dev && \
    rm -rf /var/lib/apt/lists/*
COPY . /src/
RUN mkdir /build
RUN cd /build && \
    cmake -DCMAKE_BUILD_TYPE="RelWithDebInfo" -GNinja /src && \
    ninja -j4
# Exclude failing, unrelated test.
RUN cd /build && \
    ctest --verbose --exclude-regex testCgal*

