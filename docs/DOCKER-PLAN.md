# Final deployment stage

Approved design: four independent Linux containers (frontend, planning, route,
telemetry), with the gateway and graphical Webots on Windows. One foreground
launcher owns the processes it starts and stops Compose without deleting volumes.

- [ ] Replace legacy combined bridge Compose service; isolated persistent mission volume, secret file, health checks, bounded logs.
- [ ] Fix production frontend build inputs and verify actual image builds.
- [ ] Add Windows launcher with prerequisite checks, generated persistent token, controller build, readiness, cleanup and duplicate-start protection.
- [ ] Exercise container planning, persisted missions, restart, telemetry and real Webots completion/cancellation.
- [ ] Document startup, shutdown, data migration and deployment limitations; run checks and push.

Implementation and review are performed personally, without subagents.
