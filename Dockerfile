FROM gcc:9.1

RUN apt-get update && apt-get install --no-install-recommends -y \
    libeigen3-dev \
    libyaml-cpp-dev \
    libjsoncpp-dev \
    python3-pip \
    zip \
    cppad \
    cmake \
    texlive-latex-base \
    texlive-latex-extra \
    texlive-fonts-recommended \
    cm-super \
    dvipng \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/
RUN pip3 install --requirement /tmp/requirements.txt

# Install Ipopt for MPC (ignore warnings thrown during installation)
RUN wget --no-check-certificate https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip
RUN unzip Ipopt-3.12.7.zip -d /opt && rm Ipopt-3.12.7.zip
COPY install_ipopt.sh /opt/install_ipopt.sh
RUN bash /opt/install_ipopt.sh /opt/Ipopt-3.12.7/
ENV LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:/usr/local/lib

COPY entrypoint.sh /entrypoint.sh
RUN mkdir /workspace
ENV PATH /workspace/build/Release:${PATH}
WORKDIR /workspace

ENTRYPOINT [ "/entrypoint.sh" ]
