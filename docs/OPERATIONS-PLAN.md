# Service operations implementation plan

Approved scope: prepare independent services for operation before Docker.

- [ ] Add offline repository export/import with exclusive ownership, checksum validation and restore into a new directory. Keep mission and gateway backups paired; simulator files require a separate stopped-Webots copy.
- [ ] Support GATEWAY_TOKEN_FILE, reject ambiguous configuration, keep credentials out of source control.
- [ ] Bound shutdown time, preserve failure status and verify signal behavior.
- [ ] Document stdout/stderr log retention, backup/restore, deployment placement and recovery limitations.
- [ ] Run unit tests, lint, service smoke and Linux CI.

Implementation is performed personally without agents. Tests precede behavior changes. Docker and the full-project launcher remain a later stage.
