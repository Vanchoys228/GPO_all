FROM cyberbotics/webots:R2025a-ubuntu22.04 AS build
USER root
RUN apt-get update && apt-get install --yes --no-install-recommends build-essential && rm -rf /var/lib/apt/lists/*
WORKDIR /project
COPY webots/controllers/youbot_web ./controllers/youbot_web
RUN make -C controllers/youbot_web -j2

FROM python:3.12-slim-bookworm AS assets
COPY docker/cache-webots-assets.py /cache-webots-assets.py
COPY webots/worlds/youbot_only.wbt /world.wbt
RUN python /cache-webots-assets.py

FROM cyberbotics/webots:R2025a-ubuntu22.04
USER root
RUN useradd --uid 1000 --create-home simulator && mkdir -p /data/webots && chown simulator:simulator /data/webots
WORKDIR /project
COPY --from=build /project/controllers/youbot_web/youbot_web ./controllers/youbot_web/youbot_web
COPY webots/worlds/youbot_only.wbt ./worlds/youbot_only.wbt
COPY --from=assets --chown=1000:1000 /cache /home/simulator/.cache/Cyberbotics/Webots
COPY docker/start-webots.sh /usr/local/bin/start-project-webots
COPY docker/webots-healthcheck.py /usr/local/bin/webots-healthcheck.py
RUN sed -i 's/\r$//' /usr/local/bin/start-project-webots && chmod +x /usr/local/bin/start-project-webots && \
    sed -i 's/position 32.430003995926356 31.59802927772458 14.95951242560089/position 4.8645 4.7397 2.2439/' /project/worlds/youbot_only.wbt && \
    chown -R simulator:simulator /project
ENV WEB_STATE_DIR=/data/webots
USER simulator
EXPOSE 1234
ENTRYPOINT ["/usr/local/bin/start-project-webots"]
