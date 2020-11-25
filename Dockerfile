FROM ubuntu:18.04

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    curl \
    wget \
    zip \
    unzip \
    git \
    gfortran \
    pkg-config \
    python3.6 \
    python3-pip \
    cppad \
    texlive-latex-base \
    texlive-latex-extra \
    texlive-fonts-recommended \
    cm-super \
    dvipng \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/
RUN pip3 install --requirement /tmp/requirements.txt

# Install latest cmake version min 3.18
RUN wget --no-check-certificate https://github.com/Kitware/CMake/releases/download/v3.18.4/cmake-3.18.4-Linux-x86_64.tar.gz
RUN tar -xvzf cmake-3.18.4-Linux-x86_64.tar.gz -C /opt && rm cmake-3.18.4-Linux-x86_64.tar.gz
RUN ln -s /opt/cmake-3.18.4-Linux-x86_64/bin/* /usr/local/bin
RUN ln -s /opt/cmake-3.18.4-Linux-x86_64/share/cmake-3.18 /usr/share/cmake-3.18

# Install Ipopt for MPC (ignore warnings thrown during installation)
RUN wget --no-check-certificate https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip
RUN unzip Ipopt-3.12.7.zip -d /opt && rm Ipopt-3.12.7.zip
COPY scripts/bash/install_ipopt.sh /opt/install_ipopt.sh
RUN bash /opt/install_ipopt.sh /opt/Ipopt-3.12.7/
ENV LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:/usr/local/lib

COPY scripts/bash/entrypoint.sh /mpc-entrypoint.sh
RUN mkdir /workspace
WORKDIR /workspace

ENTRYPOINT [ "/mpc-entrypoint.sh" ]
