FROM ruby:3.2-slim

# Install git and build essentials
RUN apt-get update && \
    apt-get install -y git build-essential && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install required gems
RUN gem install rake bundler

# Clone CMock repository and set up Unity
RUN git clone https://github.com/ThrowTheSwitch/CMock.git /cmock && \
    mkdir -p /cmock/vendor && \
    cd /cmock/vendor && \
    git clone https://github.com/ThrowTheSwitch/Unity.git unity && \
    cd unity && \
    git checkout v2.5.2 && \
    cd /cmock && \
    bundle install

# Set up working directory and permissions
WORKDIR /app

# Copy HAL header files
COPY BNO085_SPI_Library.h ./
COPY cmock_config.yml ./

# Create a script to run CMock
COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

ENTRYPOINT ["docker-entrypoint.sh"]
