FROM cyberbotics/webots:R2025a-ubuntu22.04

WORKDIR /project

COPY webots ./webots

RUN make -C webots/controllers/youbot_web

ENTRYPOINT ["xvfb-run", "--auto-servernum", "webots", "--stdout", "--stderr", "--batch", "--mode=fast"]
CMD ["/project/webots/worlds/youbot_only.wbt"]
